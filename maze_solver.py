import rp2
from machine import Pin, ADC, PWM, Timer
import math
import time
from maze import Maze, Heading, NORTH, EAST, SOUTH, WEST

# ==========================================
# --- EASY TUNING SETTINGS ---
# ==========================================

# ------------------------------------------
# 1. KINEMATICS
# ------------------------------------------
WHEEL_DIAMETER_MM = 45.1         # Tuned: was 44.0, bot drove 195mm instead of 200mm
TICKS_PER_REV = 700.0
TRACK_WIDTH_MM = 93.0           # Tuned: if under-turning got worse at 85, we need to go UP from 90
MM_PER_TICK = (WHEEL_DIAMETER_MM * math.pi) / TICKS_PER_REV

# ------------------------------------------
# 2. MAZE GEOMETRY (from competition rules)
# ------------------------------------------
# Competition spec: 8x8 cells, each 20x20cm, walls 5mm thick, 9cm high
#
# Cell pitch (post-to-post):      200mm
# Wall thickness:                   5mm
# Usable corridor width:          200 - 5 = 195mm
# Bot IR sensor spacing:           70mm (left-to-right)
# Gap on each side when centered: (195 - 70) / 2 = 62.5mm
#
CELL_DISTANCE_MM = 200.0        # Center-to-center = cell pitch (TUNE on real maze!)
WALL_THICKNESS_MM = 5.0         # Competition spec: 0.5cm walls
CORRIDOR_WIDTH_MM = 195.0       # Usable space = 200 - 5
IR_SPACING_MM = 70.0            # Your left-to-right IR sensor distance
BOT_LENGTH_MM = 90.0            # Your bot's length (front to back)

# Calibration: distance to drive forward from back wall to cell center
# = (corridor / 2) - (bot_length / 2) = (195 / 2) - (90 / 2) = 52.5mm
CALIBRATION_FWD_MM = (CORRIDOR_WIDTH_MM - BOT_LENGTH_MM) / 2.0  # ~52.5mm

WALL_CENTER_TARGET_MM = 63      # Ideal IR reading when centered: (195-70)/2 ≈ 63mm
WALL_CENTER_MIN_MM = 50         # Dead zone lower bound — closer than this, steer AWAY
WALL_CENTER_MAX_MM = 75         # Dead zone upper bound — farther than this, steer CLOSER
                                 # Between 50-75mm: bot is "centered enough", no correction
WALL_DETECT_RANGE_MM = 95       # Max possible side-wall distance: 195-70 = 125mm
                                 # Use 95mm for safe detection with margin for noise
WALL_THRESHOLD_MM = 100         # IR reading below this = wall present for maze mapping
                                 # Slightly higher than DETECT_RANGE so we don't miss walls
FRONT_WALL_STOP_MM = 70         # Front wall: stop/slow when this close (half corridor ~97mm)

# ------------------------------------------
# 3. CONTROL LOOP CONSTANTS
# ------------------------------------------
POSITION_KP = 1.0
POSITION_KI = 0.0
POSITION_KD = 0.5

TURN_KP = 2.0
TURN_KI = 0.0
TURN_KD = 1.0

HEADING_KP = 2.5
HEADING_KI = 0.0
HEADING_KD = 0.6

WALL_KP = 1.5
WALL_KI = 0.0
WALL_KD = 0.8

MAX_BASE_PWM = 95.0  # Maxed out! (leaves 5% headroom for PID corrections)
TURN_BASE_PWM = 70.0 # Faster turns


# ==========================================
# --- HARDWARE MODULES ---
# ==========================================

@rp2.asm_pio()
def quadrature_encoder():
    wrap_target()
    wait(1, pin, 0)
    jmp(pin, "fwd_high")
    jmp(x_dec, "done_high")
    label("fwd_high")
    mov(x, invert(x))
    jmp(x_dec, "fwd_next_high")
    label("fwd_next_high")
    mov(x, invert(x))
    label("done_high")
    in_(x, 32)
    push(noblock)
    wait(0, pin, 0)
    jmp(pin, "rev_low")
    mov(x, invert(x))
    jmp(x_dec, "fwd_next_low")
    label("fwd_next_low")
    mov(x, invert(x))
    jmp("done_low")
    label("rev_low")
    jmp(x_dec, "done_low")
    label("done_low")
    in_(x, 32)
    push(noblock)
    wrap()


class EncoderReader:
    def __init__(self, sm_id, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.sm = rp2.StateMachine(sm_id, quadrature_encoder, freq=125_000_000, in_base=self.pin_a, jmp_pin=self.pin_b)
        self.sm.exec("set(x, 0)")
        self.sm.exec("set(pindirs, 0)")
        self.sm.active(1)
        self.last_count = 0

    def get_count(self):
        while self.sm.rx_fifo():
            val = self.sm.get()
            if val & 0x80000000:
                self.last_count = val - 0x100000000
            else:
                self.last_count = val
        return self.last_count


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits
        self.target = 0
        self.prev_error = 0
        self.integral = 0

    def reset(self):
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt_seconds):
        error = self.target - current_value
        p_term = self.kp * error
        self.integral += (error * dt_seconds)
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt_seconds
        d_term = self.kd * derivative
        output = p_term + i_term + d_term
        self.prev_error = error
        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)
        return output


# ==========================================
# --- MAZE SOLVER BOT ---
# ==========================================

class MazeSolver:
    """
    Full maze-solving micromouse with 3-phase algorithm:
      Phase 1: Explore from (0,0) to center
      Phase 2: Return from center to (0,0)
      Phase 3: Speed run to center
    """

    def __init__(self):
        # ---- State Machine ----
        # IDLE: waiting for next command
        # DRIVE: moving forward one cell
        # TURN: pivoting to new heading
        self.state = 'IDLE'

        # ---- Maze State ----
        self.maze = Maze()
        self.cell_x = 0
        self.cell_y = 0
        self.abs_heading = Heading.NORTH  # 0=N, 1=E, 2=S, 3=W
        self.visited = set()

        # ---- Hardware: UI ----
        self.led = Pin(14, Pin.OUT)
        self.btn = Pin(15, Pin.IN, Pin.PULL_UP)

        # ---- Hardware: Motors ----
        self.pwm_r = PWM(Pin(2))
        self.pwm_r.freq(20000)
        self.r_ain1 = Pin(3, Pin.OUT)
        self.r_ain2 = Pin(4, Pin.OUT)

        self.pwm_l = PWM(Pin(7))
        self.pwm_l.freq(20000)
        self.l_bin1 = Pin(6, Pin.OUT)
        self.l_bin2 = Pin(5, Pin.OUT)
        self.stop_motors()

        # ---- Hardware: Encoders ----
        self.enc_right = EncoderReader(0, 10, 11)
        self.enc_left = EncoderReader(1, 13, 12)
        self.ticks_r = 0
        self.ticks_l = 0

        # ---- Hardware: IR Sensors ----
        self.adc_l = ADC(Pin(26))
        self.adc_f = ADC(Pin(27))
        self.adc_r = ADC(Pin(28))
        self.dist_l = 999
        self.dist_f = 999
        self.dist_r = 999

        # IR averaging
        self.BUFFER_SIZE = 5
        self.buf_l = [0] * self.BUFFER_SIZE
        self.buf_f = [0] * self.BUFFER_SIZE
        self.buf_r = [0] * self.BUFFER_SIZE
        self.buf_idx = 0

        # IR Blind-Zone Tracking
        self.IR_MIN_MM = 40
        self.IR_BLINDZONE_THRESH = 60
        self.prev_dist_l = 999
        self.prev_dist_f = 999
        self.prev_dist_r = 999

        # ---- PID Controllers ----
        self.heading_pid = PIDController(kp=HEADING_KP, ki=HEADING_KI, kd=HEADING_KD, output_limits=(-40, 40))
        self.wall_pid = PIDController(kp=WALL_KP, ki=WALL_KI, kd=WALL_KD, output_limits=(-15, 15))
        self.distance_pid = PIDController(kp=POSITION_KP, ki=POSITION_KI, kd=POSITION_KD, output_limits=(-MAX_BASE_PWM, MAX_BASE_PWM))
        self.turn_pid = PIDController(kp=TURN_KP, ki=TURN_KI, kd=TURN_KD, output_limits=(-TURN_BASE_PWM, TURN_BASE_PWM))

        # ---- Motion Targets ----
        self.target_distance_mm = 0.0
        self.target_heading_deg = 0.0
        self.target_base_pwm = 0.0

        # ---- Pose Estimation ----
        self.pos_distance_mm = 0.0
        self.pos_heading_deg = 0.0

        self.last_pwm_r = 0
        self.last_pwm_l = 0

        # ---- Timer ----
        self.control_timer = Timer(-1)
        self.running = False

    # ==========================================
    # Hardware Low-Level
    # ==========================================

    def stop_motors(self):
        self.pwm_r.duty_u16(0)
        self.pwm_l.duty_u16(0)
        self.r_ain1.value(0)
        self.r_ain2.value(0)
        self.l_bin1.value(0)
        self.l_bin2.value(0)

    def set_speeds(self, right_pct, left_pct):
        right_pct = max(-100, min(100, right_pct))
        left_pct = max(-100, min(100, left_pct))
        self.last_pwm_r = right_pct
        self.last_pwm_l = left_pct

        if right_pct > 0:
            self.r_ain1.value(0)
            self.r_ain2.value(1)
        elif right_pct < 0:
            self.r_ain1.value(1)
            self.r_ain2.value(0)
        else:
            self.r_ain1.value(0)
            self.r_ain2.value(0)
        self.pwm_r.duty_u16(int(abs(right_pct) / 100 * 65535))

        if left_pct > 0:
            self.l_bin1.value(1)
            self.l_bin2.value(0)
        elif left_pct < 0:
            self.l_bin1.value(0)
            self.l_bin2.value(1)
        else:
            self.l_bin1.value(0)
            self.l_bin2.value(0)
        self.pwm_l.duty_u16(int(abs(left_pct) / 100 * 65535))

    def _adc_to_mm(self, adc_val):
        if adc_val < 1000: return 999
        volts = (adc_val / 65535.0) * 3.3
        if volts < 0.4: return 999
        return max(40, min(999, int((120.0 / volts) - 8.0)))

    def _fix_blindzone(self, new_dist, prev_dist):
        if prev_dist <= self.IR_MIN_MM and new_dist > self.IR_BLINDZONE_THRESH:
            return self.IR_MIN_MM
        return new_dist

    def _update_sensors(self):
        self.ticks_r = self.enc_right.get_count()
        self.ticks_l = self.enc_left.get_count()

        self.buf_l[self.buf_idx] = self.adc_l.read_u16()
        self.buf_f[self.buf_idx] = self.adc_f.read_u16()
        self.buf_r[self.buf_idx] = self.adc_r.read_u16()
        self.buf_idx = (self.buf_idx + 1) % self.BUFFER_SIZE

        avg_l = sum(self.buf_l) // self.BUFFER_SIZE
        avg_f = sum(self.buf_f) // self.BUFFER_SIZE
        avg_r = sum(self.buf_r) // self.BUFFER_SIZE

        raw_l = self._adc_to_mm(avg_l)
        raw_f = self._adc_to_mm(avg_f)
        raw_r = self._adc_to_mm(avg_r)

        self.dist_l = self._fix_blindzone(raw_l, self.prev_dist_l)
        self.dist_f = self._fix_blindzone(raw_f, self.prev_dist_f)
        self.dist_r = self._fix_blindzone(raw_r, self.prev_dist_r)

        self.prev_dist_l = self.dist_l
        self.prev_dist_f = self.dist_f
        self.prev_dist_r = self.dist_r

    def _reset_encoders(self):
        """Reset encoder counters and all PID state for a fresh movement."""
        self.enc_right.sm.exec("set(x, 0)")
        self.enc_left.sm.exec("set(x, 0)")
        self.enc_right.last_count = 0
        self.enc_left.last_count = 0
        self.ticks_r = 0
        self.ticks_l = 0
        self.heading_pid.reset()
        self.wall_pid.reset()
        self.distance_pid.reset()
        self.turn_pid.reset()
        self.pos_distance_mm = 0.0
        self.pos_heading_deg = 0.0
        self.target_base_pwm = 0.0

    # ==========================================
    # Control Loop (500 Hz Timer Callback)
    # ==========================================

    def control_loop(self, timer):
        dt = 0.002
        self._update_sensors()

        # Update odometry
        self.pos_distance_mm = ((self.ticks_l + self.ticks_r) / 2.0) * MM_PER_TICK
        self.pos_heading_deg = (((self.ticks_l - self.ticks_r) * MM_PER_TICK) / TRACK_WIDTH_MM) * (180.0 / math.pi)

        if self.state == 'DRIVE':
            # Distance PID
            self.distance_pid.target = self.target_distance_mm
            raw_speed = self.distance_pid.compute(self.pos_distance_mm, dt)

            MIN_PWM = 35.0  # Increased from 20 to overcome motor stiction (no more push needed!)
            RAMP_DISTANCE_MM = 30.0   # Gradually accelerate over the first 30mm
            DECEL_DISTANCE_MM = 30.0  # Gradually decelerate in the last 30mm
            distance_error = self.target_distance_mm - self.pos_distance_mm

            # STOP THRESHOLD: Increased to 3.0mm to stop over-calibrating
            if abs(distance_error) < 3.0:
                self.target_base_pwm = 0.0
                self.state = 'IDLE'
                self.set_speeds(0, 0)
                return
            else:
                if raw_speed > 0:
                    self.target_base_pwm = raw_speed + MIN_PWM
                else:
                    self.target_base_pwm = raw_speed - MIN_PWM

            # Smooth ramp-up: limit speed during the first 30mm of travel
            if self.pos_distance_mm < RAMP_DISTANCE_MM:
                ramp_factor = self.pos_distance_mm / RAMP_DISTANCE_MM  # 0.0 → 1.0
                ramp_factor = max(0.30, ramp_factor)  # Start at 30% power minimum (was 15%)
                self.target_base_pwm *= ramp_factor

            # Smooth deceleration ramp: reduce speed in the last 30mm for clean stops
            remaining = self.target_distance_mm - self.pos_distance_mm
            if 0 < remaining < DECEL_DISTANCE_MM:
                decel_factor = remaining / DECEL_DISTANCE_MM  # 1.0 → 0.0
                decel_factor = max(0.30, decel_factor)  # Don't go below 30% (motor stiction)
                self.target_base_pwm *= decel_factor

            # Wall following PID (centering correction)
            # Dead-zone: if within 50-75mm of each wall, bot is centered enough — no correction.
            # This prevents constant hunting/oscillation and makes motion much smoother.
            wall_steering_offset = 0
            if self.dist_l < WALL_DETECT_RANGE_MM and self.dist_r < WALL_DETECT_RANGE_MM and self.dist_f > FRONT_WALL_STOP_MM:
                # Both walls visible — center between them
                diff = self.dist_l - self.dist_r
                if abs(diff) > 10:  # Dead zone: ±10mm imbalance is OK
                    wall_steering_offset = self.wall_pid.compute(diff, dt)
                else:
                    self.wall_pid.reset()
            elif self.dist_l < WALL_DETECT_RANGE_MM and self.dist_f > FRONT_WALL_STOP_MM:
                # Only left wall — maintain within acceptable range
                if self.dist_l < WALL_CENTER_MIN_MM:
                    # Too close to left wall — steer right
                    position_error = self.dist_l - WALL_CENTER_TARGET_MM
                    wall_steering_offset = self.wall_pid.compute(position_error, dt)
                elif self.dist_l > WALL_CENTER_MAX_MM:
                    # Too far from left wall — steer left
                    position_error = self.dist_l - WALL_CENTER_TARGET_MM
                    wall_steering_offset = self.wall_pid.compute(position_error, dt)
                else:
                    # In the sweet spot (50-75mm) — no correction needed
                    self.wall_pid.reset()
            elif self.dist_r < WALL_DETECT_RANGE_MM and self.dist_f > FRONT_WALL_STOP_MM:
                # Only right wall — maintain within acceptable range
                if self.dist_r < WALL_CENTER_MIN_MM:
                    # Too close to right wall — steer left
                    position_error = -(self.dist_r - WALL_CENTER_TARGET_MM)
                    wall_steering_offset = self.wall_pid.compute(position_error, dt)
                elif self.dist_r > WALL_CENTER_MAX_MM:
                    # Too far from right wall — steer right
                    position_error = -(self.dist_r - WALL_CENTER_TARGET_MM)
                    wall_steering_offset = self.wall_pid.compute(position_error, dt)
                else:
                    self.wall_pid.reset()
            else:
                self.wall_pid.reset()

            # Heading PID
            heading_error = self.ticks_l - self.ticks_r
            self.heading_pid.target = wall_steering_offset
            correction = self.heading_pid.compute(heading_error, dt)

            self.set_speeds(self.target_base_pwm - correction,
                            self.target_base_pwm + correction)

        elif self.state == 'TURN':
            self.turn_pid.target = self.target_heading_deg
            turn_speed = self.turn_pid.compute(self.pos_heading_deg, dt)

            MIN_TURN_PWM = 40.0  # Increased from 25 to overcome motor stiction during turns
            heading_error_deg = self.target_heading_deg - self.pos_heading_deg

            # STOP THRESHOLD: Increased to 4.0 degrees to stop over-calibrating end of turn
            if abs(heading_error_deg) < 4.0:
                self.set_speeds(0, 0)
                self.state = 'IDLE'
                return
            else:
                if turn_speed > 0:
                    turn_speed += MIN_TURN_PWM
                else:
                    turn_speed -= MIN_TURN_PWM
                self.set_speeds(-turn_speed, turn_speed)

        elif self.state == 'IDLE':
            self.set_speeds(0, 0)

    # ==========================================
    # High-Level Movement Commands
    # ==========================================

    def move_forward_distance(self, distance_mm):
        """Drive forward an exact distance in mm, then STOP."""
        self._reset_encoders()
        self.target_distance_mm = distance_mm
        self.state = 'DRIVE'

        # Timeout: ~0.5mm/ms + 3s safety margin
        timeout_ms = int(abs(distance_mm) / 0.5) + 3000
        start = time.ticks_ms()
        while self.state != 'IDLE':
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                print("STUCK! Aborting move.")
                self.state = 'IDLE'
                break
            time.sleep(0.01)

        self.set_speeds(0, 0)
        
    def move_forward_continuous(self, distance_mm):
        """
        Drive forward continuously. If already driving, just adds the distance
        without stopping the motors, enabling smooth multi-cell motion.
        """
        if self.state == 'DRIVE':
            # Already driving! Just add to the target
            self.target_distance_mm += distance_mm
        else:
            # Starting from a stop
            self._reset_encoders()
            self.target_distance_mm = distance_mm
            self.state = 'DRIVE'
            
        # Wait until we hit the *new* target
        while self.state != 'IDLE':
            time.sleep(0.01)
            
        # We don't force set_speeds(0,0) here, allowing the next command
        # to catch it while it's still moving if we call this in a fast loop.
        # But for exact cell stops, `state='IDLE'` in the control loop will stop it.

    def move_forward_one_cell(self):
        """Drive exactly one maze cell (CELL_DISTANCE_MM) forward."""
        self.move_forward_distance(CELL_DISTANCE_MM)

    def calibrate_start_position(self):
        """
        Auto-center the bot in cell (0,0) at startup.
        
        Placement: bot rear roughly near the SOUTH WALL, facing NORTH.
        Cell (0,0) has walls on South, West, and East — perfect for calibration!
        
        Strategy: Creep forward slowly while reading side IRs. Stop when
        both side readings are within the acceptable centering range.
        This is error-tolerant — doesn't matter exactly where you place the bot.
        """
        print("=== CALIBRATING START POSITION ===")

        # First: drive most of the way using a rough estimate
        # Use 80% of calculated distance to avoid overshooting
        rough_distance = CALIBRATION_FWD_MM * 0.8
        print(f"  Rough advance: {rough_distance:.1f}mm")
        self.move_forward_distance(rough_distance)
        time.sleep(0.2)

        # Now: fine-tune using sensor feedback
        # Creep forward in tiny steps until side readings are in range
        CREEP_STEP_MM = 5.0        # Small 5mm steps for precision
        MAX_CREEP_MM = 60.0        # Safety: don't creep more than this total
        total_crept = 0.0

        print("  Fine-tuning with sensors...")
        while total_crept < MAX_CREEP_MM:
            # Fresh sensor reading
            for _ in range(self.BUFFER_SIZE + 2):
                self._update_sensors()
                time.sleep(0.005)

            l, r = self.dist_l, self.dist_r
            print(f"    L={l}mm R={r}mm", end="")

            # Check if both side readings are in the sweet spot
            l_ok = WALL_CENTER_MIN_MM <= l <= WALL_CENTER_MAX_MM
            r_ok = WALL_CENTER_MIN_MM <= r <= WALL_CENTER_MAX_MM
            diff_ok = abs(l - r) < 20  # Left and right roughly balanced

            if l_ok and r_ok and diff_ok:
                print(" ✓ CENTERED!")
                break
            elif l < WALL_CENTER_MIN_MM or r < WALL_CENTER_MIN_MM:
                # Too close to a wall — we've gone too far or are misaligned
                print(" (too close to wall — stopping)")
                break
            else:
                # Not centered yet — creep forward a bit more
                print(" → creeping...")
                self.move_forward_distance(CREEP_STEP_MM)
                total_crept += CREEP_STEP_MM
                time.sleep(0.1)

        # Final report
        for _ in range(self.BUFFER_SIZE + 2):
            self._update_sensors()
            time.sleep(0.005)

        print(f"  Final: L={self.dist_l}mm  R={self.dist_r}mm  (target: {WALL_CENTER_MIN_MM}-{WALL_CENTER_MAX_MM}mm)")
        print("=== CALIBRATION DONE ===")
        self.blink_led(2, delay=0.1)
        time.sleep(0.3)

    def turn_to_heading(self, target_heading):
        """Pivot in place to face target_heading (absolute). Updates self.abs_heading."""
        diff = (target_heading - self.abs_heading) % 4

        if diff == 0:
            return  # Already facing correct direction

        # Calculate degrees to turn
        if diff == 1:
            degrees = -90.0   # Turn right (negative heading_deg)
        elif diff == 2:
            degrees = 180.0   # 180 turn (either direction works)
        elif diff == 3:
            degrees = 90.0    # Turn left (positive heading_deg)

        self._reset_encoders()
        self.target_heading_deg = degrees
        self.state = 'TURN'

        # Timeout: 5 seconds max for any turn
        timeout_ms = 5000
        start = time.ticks_ms()
        while self.state != 'IDLE':
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                print("STUCK during turn! Aborting.")
                self.state = 'IDLE'
                break
            time.sleep(0.01)

        self.set_speeds(0, 0)
        time.sleep(0.05)  # Settle time

        self.abs_heading = target_heading

    # ==========================================
    # Maze Logic — Wall Sensing
    # ==========================================

    def read_walls(self):
        """Read IR sensors and determine wall presence (True/False) for left, front, right."""
        # The 500Hz control loop already maintains fresh averaged sensor data.
        # Just wait ~10 control loop cycles to ensure readings are from
        # our current stopped position, not stale from mid-motion.
        time.sleep(0.02)

        wall_left = self.dist_l < WALL_THRESHOLD_MM
        wall_front = self.dist_f < WALL_THRESHOLD_MM
        wall_right = self.dist_r < WALL_THRESHOLD_MM

        return wall_left, wall_front, wall_right

    def scan_and_update(self):
        """Read walls, update maze model, mark visited."""
        wall_left, wall_front, wall_right = self.read_walls()
        self.maze.update_cell_walls(self.cell_x, self.cell_y, self.abs_heading,
                                     wall_left, wall_front, wall_right)
        self.visited.add((self.cell_x, self.cell_y))

        print(f"Cell ({self.cell_x},{self.cell_y}) H:{self.abs_heading} | "
              f"L:{wall_left} F:{wall_front} R:{wall_right}")

    # ==========================================
    # Maze Logic — Flood Fill
    # ==========================================

    def flood_fill_multi(self, goals):
        """BFS flood fill from multiple goal cells. Uses index-based queue for O(1) dequeue."""
        for x in range(self.maze.width):
            for y in range(self.maze.height):
                self.maze.distances[x][y] = 255

        queue = []
        head = 0
        for gx, gy in goals:
            self.maze.distances[gx][gy] = 0
            queue.append((gx, gy))

        while head < len(queue):
            cx, cy = queue[head]
            head += 1
            current_dist = self.maze.distances[cx][cy]

            for heading in [Heading.NORTH, Heading.EAST, Heading.SOUTH, Heading.WEST]:
                if not self.maze.has_wall(cx, cy, 1 << heading):
                    nx = cx + Heading.D_X[heading]
                    ny = cy + Heading.D_Y[heading]

                    if 0 <= nx < self.maze.width and 0 <= ny < self.maze.height:
                        if self.maze.distances[nx][ny] == 255:
                            self.maze.distances[nx][ny] = current_dist + 1
                            queue.append((nx, ny))

    def get_best_move_smart(self):
        """Choose neighbor with lowest distance, preferring unvisited cells."""
        x, y = self.cell_x, self.cell_y
        best_distance = 255
        best_heading = self.abs_heading
        best_visited = True

        for heading in [Heading.NORTH, Heading.EAST, Heading.SOUTH, Heading.WEST]:
            if not self.maze.has_wall(x, y, 1 << heading):
                nx = x + Heading.D_X[heading]
                ny = y + Heading.D_Y[heading]

                if 0 <= nx < self.maze.width and 0 <= ny < self.maze.height:
                    neighbor_dist = self.maze.distances[nx][ny]
                    neighbor_visited = (nx, ny) in self.visited

                    if (neighbor_dist < best_distance or
                        (neighbor_dist == best_distance and not neighbor_visited and best_visited) or
                        (neighbor_dist == best_distance and neighbor_visited == best_visited and heading == self.abs_heading)):
                        best_distance = neighbor_dist
                        best_heading = heading
                        best_visited = neighbor_visited

        return best_heading

    # ==========================================
    # Navigation — Move One Cell in a Direction
    # ==========================================

    def navigate_one_cell(self, target_heading):
        """Turn to face target_heading and drive forward one cell. Updates cell position."""
        self.turn_to_heading(target_heading)
        self.move_forward_one_cell()

        # Update cell position
        self.cell_x += Heading.D_X[self.abs_heading]
        self.cell_y += Heading.D_Y[self.abs_heading]

    # ==========================================
    # 3-Phase Maze Solver
    # ==========================================

    def phase1_explore(self, goals):
        """Phase 1: Navigate from (0,0) to center, mapping walls along the way."""
        print("=== PHASE 1: EXPLORE TO CENTER ===")

        while (self.cell_x, self.cell_y) not in goals:
            # 1. Scan walls at current cell
            self.scan_and_update()

            # 2. Flood fill from goals
            self.flood_fill_multi(goals)

            # 3. Get smart move (prefers unvisited)
            best = self.get_best_move_smart()

            # 4. Move one cell
            print(f"  -> Moving towards heading {best}")
            self.navigate_one_cell(best)

        # Scan the goal cell too
        self.scan_and_update()
        print(f"=== REACHED CENTER at ({self.cell_x},{self.cell_y})! ===")
        print(f"  Explored {len(self.visited)} cells")

    def phase2_return(self):
        """Phase 2: Navigate from center back to (0,0), scanning more walls."""
        print("=== PHASE 2: RETURN TO START ===")
        start_goal = [(0, 0)]

        while (self.cell_x, self.cell_y) != (0, 0):
            # Scan walls (still mapping on the way back)
            self.scan_and_update()

            # Flood fill targeting start
            self.flood_fill_multi(start_goal)

            # Shortest path home — no detours through unvisited cells
            best = self.maze.get_best_move(self.cell_x, self.cell_y, self.abs_heading)
            self.navigate_one_cell(best)

        self.scan_and_update()
        print(f"=== BACK AT START ===")
        print(f"  Total cells explored: {len(self.visited)}")

        # Persist the maze map to flash so it survives power loss
        self.maze.save_to_file()

    def _compute_optimal_path(self, goals):
        """Pre-compute the full optimal path from current position to goals using flood fill."""
        self.flood_fill_multi(goals)
        path = []
        cx, cy = self.cell_x, self.cell_y
        heading = self.abs_heading

        while (cx, cy) not in goals:
            best = self.maze.get_best_move(cx, cy, heading)
            path.append(best)
            cx += Heading.D_X[best]
            cy += Heading.D_Y[best]
            heading = best

        print(f"  Optimal path: {len(path)} moves")
        return path

    def phase3_speed_run(self, goals):
        """Phase 3: Optimal speed run from (0,0) to center using mapped maze.
        Pre-computes the path, merges straight-line segments for continuous
        motion, and skips per-cell flood fill for maximum speed."""
        print("=== PHASE 3: SPEED RUN ===")

        # Pre-compute the entire optimal path
        path = self._compute_optimal_path(goals)

        if len(path) == 0:
            print("  Already at goal!")
            return

        # Execute with merged straight segments
        i = 0
        while i < len(path):
            heading = path[i]

            # Count consecutive moves in the same direction
            run_length = 1
            while i + run_length < len(path) and path[i + run_length] == heading:
                run_length += 1

            # Turn to face the direction (if needed)
            self.turn_to_heading(heading)

            # Drive the full straight segment in one go
            total_mm = CELL_DISTANCE_MM * run_length
            if run_length > 1:
                print(f"  >> Straight run: {run_length} cells ({total_mm:.0f}mm) heading {heading}")
            self.move_forward_distance(total_mm)

            # Update cell position
            for _ in range(run_length):
                self.cell_x += Heading.D_X[heading]
                self.cell_y += Heading.D_Y[heading]

            i += run_length

        print(f"=== SPEED RUN COMPLETE at ({self.cell_x},{self.cell_y})! ===")

    # ==========================================
    # Main Entry Point
    # ==========================================

    def blink_led(self, times, delay=0.1):
        for _ in range(times):
            self.led.value(1)
            time.sleep(delay)
            self.led.value(0)
            time.sleep(delay)

    def start(self):
        """Wait for button, then run the 3-phase maze solver."""
        print("=== MICROMOUSE MAZE SOLVER ===")
        print("Press button to start...")

        led_state = 0
        while self.btn.value() == 1:
            self.led.value(led_state)
            led_state = 1 - led_state
            time.sleep(0.5)

        print("Button pressed! Starting in 1.5 seconds...")
        self.led.value(1)
        time.sleep(1.5)

        # Reset everything
        self._reset_encoders()
        self.cell_x = 0
        self.cell_y = 0
        self.abs_heading = Heading.NORTH
        self.visited = set()

        # Start the 500Hz control timer
        self.control_timer.init(period=2, mode=Timer.PERIODIC, callback=self.control_loop)
        self.running = True

        # --- AUTO-CALIBRATE: Center the bot in cell (0,0) ---
        self.calibrate_start_position()

        # 8x8 competition maze: center is 2x2 block
        goals = [(3, 3), (3, 4), (4, 3), (4, 4)]

        try:
            # Phase 1: Explore to center
            self.phase1_explore(goals)

            # Pause briefly at center
            self.blink_led(5)
            time.sleep(1.0)

            # Phase 2: Return to start
            self.phase2_return()

            # Pause at start — wait for second button press for speed run
            self.stop_motors()
            self.blink_led(10, delay=0.05)
            print("Press button for SPEED RUN...")

            while self.btn.value() == 1:
                self.led.value(led_state)
                led_state = 1 - led_state
                time.sleep(0.3)

            print("Speed run starting in 1 second...")
            self.led.value(1)
            time.sleep(1.0)

            # Reset encoders for fresh run
            self._reset_encoders()

            # Phase 3: Speed run
            self.phase3_speed_run(goals)

            # Done!
            self.blink_led(3, delay=0.3)
            print("=== MAZE SOLVED! ===")

        except KeyboardInterrupt:
            pass

        self.stop()

    def stop(self):
        self.running = False
        self.control_timer.deinit()
        self.stop_motors()
        self.led.value(0)
        print("Stopped.")


# Main Execution
if __name__ == "__main__":
    solver = MazeSolver()
    solver.start()
