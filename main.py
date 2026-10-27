import time
from maze_solver import MazeSolver
from maze import Maze, Heading

# ==========================================
# Helpers for the Real Bot
# ==========================================

ALL_HEADINGS = [Heading.NORTH, Heading.EAST, Heading.SOUTH, Heading.WEST]

def log(msg):
    print(msg)

def turn_bot(solver, current_heading, target_heading):
    """Turns the real bot to face the target heading."""
    diff = (target_heading - current_heading) % 4
    if diff == 0:
        return target_heading

    # Wait for bot to fully stop before turning (Reduced from 0.1s to 0.02s)
    time.sleep(0.02)

    # Calculate degrees (Swapped based on your physical robot turning the opposite way!)
    degrees = 0
    if diff == 1:
        degrees = 90.0   # Right (Normally -90, but your motors/encoders turn it the other way)
    elif diff == 2:
        degrees = 180.0  # U-turn
    elif diff == 3:
        degrees = -90.0  # Left (Normally 90)
    
    log(f"Turning {degrees} degrees...")
    solver._reset_encoders()
    solver.target_heading_deg = degrees
    solver.state = 'TURN'
    
    # Wait until turn completes
    while solver.state != 'IDLE':
        time.sleep(0.01)
        if solver.btn.value() == 0:  # Emergency stop
            solver.stop()
            raise KeyboardInterrupt("Button stop!")
    # Small pause before next move (Reduced from 0.1s to 0.02s)
    time.sleep(0.02)
    return target_heading

def move_forward_bot(solver):
    """Drives the real bot exactly ONE cell forward."""
    log("Driving 1 cell forward...")
    solver._reset_encoders()
    solver.move_forward_one_cell()
    
    # Wait until drive completes
    while solver.state != 'IDLE':
        time.sleep(0.01)
        if solver.btn.value() == 0:  # Emergency stop
            solver.stop()
            raise KeyboardInterrupt("Button stop!")
            
    # Small pause to let bot settle before reading sensors (Reduced from 0.1s to 0.02s)
    time.sleep(0.02)

def scan_and_update(maze, x, y, heading, solver, visited):
    """Reads real IR sensors to update the maze map."""
    # Take multiple quick readings to filter noise
    for _ in range(5):
        solver._update_sensors()
        time.sleep(0.005)
        
    # Translate mm distance to True/False for walls
    from maze_solver import WALL_THRESHOLD_MM
    wall_left = solver.dist_l < WALL_THRESHOLD_MM
    wall_front = solver.dist_f < WALL_THRESHOLD_MM
    wall_right = solver.dist_r < WALL_THRESHOLD_MM
    
    maze.update_cell_walls(x, y, heading, wall_left, wall_front, wall_right)
    visited.add((x, y))
    
    log(f"Scan at ({x},{y}): L={wall_left}, F={wall_front}, R={wall_right}")

# (Flood fill and intelligent movement logic - identical to simulation)
def flood_fill_multi(maze, goals):
    for x in range(maze.width):
        for y in range(maze.height):
            maze.distances[x][y] = 255
    queue = []
    for gx, gy in goals:
        maze.distances[gx][gy] = 0
        queue.append((gx, gy))
    while len(queue) > 0:
        cx, cy = queue.pop(0)
        current_dist = maze.distances[cx][cy]
        for heading in ALL_HEADINGS:
            if not maze.has_wall(cx, cy, 1 << heading):
                nx = cx + Heading.D_X[heading]
                ny = cy + Heading.D_Y[heading]
                if 0 <= nx < maze.width and 0 <= ny < maze.height:
                    if maze.distances[nx][ny] == 255:
                        maze.distances[nx][ny] = current_dist + 1
                        queue.append((nx, ny))

def get_best_move_smart(maze, x, y, current_heading, visited):
    best_distance = 256
    best_heading = None
    best_visited = True
    for heading in ALL_HEADINGS:
        if not maze.has_wall(x, y, 1 << heading):
            nx = x + Heading.D_X[heading]
            ny = y + Heading.D_Y[heading]
            if 0 <= nx < maze.width and 0 <= ny < maze.height:
                neighbor_dist = maze.distances[nx][ny]
                neighbor_visited = (nx, ny) in visited
                if (best_heading is None or
                    neighbor_dist < best_distance or
                    (neighbor_dist == best_distance and not neighbor_visited and best_visited) or
                    (neighbor_dist == best_distance and neighbor_visited == best_visited and heading == current_heading)):
                    best_distance = neighbor_dist
                    best_heading = heading
                    best_visited = neighbor_visited
    return best_heading

# ==========================================
# Phase 1: EXPLORE
# ==========================================

def explore_to_goal(maze, x, y, heading, goals, visited, solver):
    log("=== PHASE 1: EXPLORE TO CENTER ===")
    recent_positions = []
    move_count = 0

    while (x, y) not in goals:
        scan_and_update(maze, x, y, heading, solver, visited)
        flood_fill_multi(maze, goals)

        # Loop detection (prevent 1,0 -> 1,1 bounce)
        recent_positions.append((x, y))
        if len(recent_positions) > 10: recent_positions.pop(0)

        if recent_positions.count((x, y)) >= 3:
            log(f"  LOOP detected at ({x},{y})! Forcing alternate move.")
            best_heading = None
            for h in ALL_HEADINGS:
                if not maze.has_wall(x, y, 1 << h):
                    nx, ny = x + Heading.D_X[h], y + Heading.D_Y[h]
                    if 0 <= nx < maze.width and 0 <= ny < maze.height:
                        if (nx, ny) not in recent_positions:
                            best_heading = h
                            break
            if best_heading is None:
                for h in ALL_HEADINGS:
                    if not maze.has_wall(x, y, 1 << h):
                        best_heading = h
                        break
            if best_heading is None:
                log(f"STUCK at ({x},{y})!")
                return None
            recent_positions.clear()
        else:
            best_heading = get_best_move_smart(maze, x, y, heading, visited)
            if best_heading is None:
                log(f"STUCK at ({x},{y})!")
                return None

        # Execute physical move
        heading = turn_bot(solver, heading, best_heading)
        move_forward_bot(solver)
        
        x += Heading.D_X[heading]
        y += Heading.D_Y[heading]
        move_count += 1
        log(f"  Moved to ({x},{y}) [{move_count}]")

    scan_and_update(maze, x, y, heading, solver, visited)
    log(f"=== REACHED GOAL at ({x},{y})! [{move_count} moves] ===")
    solver.blink_led(3, 0.1)
    return (x, y, heading)

# ==========================================
# Phase 2: RETURN
# ==========================================

def return_to_start(maze, x, y, heading, visited, solver):
    log("=== PHASE 2: RETURN TO START ===")
    start_goals = [(0, 0)]
    recent_positions = []
    
    while (x, y) != (0, 0):
        scan_and_update(maze, x, y, heading, solver, visited)
        flood_fill_multi(maze, start_goals)

        # Loop detection
        recent_positions.append((x, y))
        if len(recent_positions) > 10: recent_positions.pop(0)

        if recent_positions.count((x, y)) >= 3:
            log(f"  LOOP detected at ({x},{y}) during return! Forcing alternate.")
            best_heading = None
            for h in ALL_HEADINGS:
                if not maze.has_wall(x, y, 1 << h):
                    nx, ny = x + Heading.D_X[h], y + Heading.D_Y[h]
                    if 0 <= nx < maze.width and 0 <= ny < maze.height:
                        if (nx, ny) not in recent_positions:
                            best_heading = h
                            break
            if best_heading is None:
                for h in ALL_HEADINGS:
                    if not maze.has_wall(x, y, 1 << h):
                        best_heading = h
                        break
            if best_heading is None: return None
            recent_positions.clear()
        else:
            best_heading = get_best_move_smart(maze, x, y, heading, visited)
            if best_heading is None: return None

        heading = turn_bot(solver, heading, best_heading)
        move_forward_bot(solver)
        x += Heading.D_X[heading]
        y += Heading.D_Y[heading]

    scan_and_update(maze, x, y, heading, solver, visited)
    return heading

# ==========================================
# Phase 3: SPEED RUN (Continuous)
# ==========================================

def move_forward_continuous_bot(solver, num_cells):
    """Drives the bot forward multiple cells without stopping."""
    log(f"Driving {num_cells} cells continuously...")
    distance = num_cells * solver.CELL_DISTANCE_MM
    solver.move_forward_continuous(distance)
    
    # Wait until drive completes
    while solver.state != 'IDLE':
        time.sleep(0.01)
        if solver.btn.value() == 0:
            solver.stop()
            raise KeyboardInterrupt("Button stop!")
    time.sleep(0.1)

def speed_run(maze, heading, goals, visited, solver):
    log("=== PHASE 3: SPEED RUN (CONTINUOUS) ===")
    x, y = 0, 0
    flood_fill_multi(maze, goals)

    # 1. Pre-calculate the entire optimal path
    path_headings = []
    curr_x, curr_y = x, y
    curr_heading = heading
    
    while (curr_x, curr_y) not in goals:
        # We only follow paths we already visited to be safe
        best_dist = 256
        best_heading = None
        
        for h in ALL_HEADINGS:
            if not maze.has_wall(curr_x, curr_y, 1 << h):
                nx, ny = curr_x + Heading.D_X[h], curr_y + Heading.D_Y[h]
                if 0 <= nx < maze.width and 0 <= ny < maze.height:
                    if (nx, ny) in visited and maze.distances[nx][ny] < best_dist:
                        best_dist = maze.distances[nx][ny]
                        best_heading = h
                        
        if best_heading is None:
            log("Error: No safe known path found for speed run!")
            return False
            
        path_headings.append(best_heading)
        curr_x += Heading.D_X[best_heading]
        curr_y += Heading.D_Y[best_heading]

    log(f"Optimal path calculated: {len(path_headings)} steps.")

    # 2. Execute the path, chunking straight movements
    i = 0
    current_heading = heading
    
    while i < len(path_headings):
        target_heading = path_headings[i]
        
        # Turn if necessary
        if target_heading != current_heading:
            current_heading = turn_bot(solver, current_heading, target_heading)
            
        # Count how many forward cells we can do in a row
        forward_cells = 1
        while i + forward_cells < len(path_headings) and path_headings[i + forward_cells] == current_heading:
            forward_cells += 1
            
        # Execute the continuous drive
        move_forward_continuous_bot(solver, forward_cells)
        
        # Advance index
        i += forward_cells

    log("=== SPEED RUN REACHED CENTER! ===")
    return True

# ==========================================
# FULL COMPETITION RUNNER (Autonomous)
# ==========================================

import os

def has_saved_map():
    """Check if a saved maze map exists on flash."""
    try:
        f = open("maze_map.txt", "r")
        f.close()
        return True
    except:
        return False

def delete_saved_map():
    """Delete saved maze data from flash."""
    try: os.remove("maze_map.txt")
    except: pass
    try: os.remove("visited.txt")
    except: pass

def load_visited():
    """Load visited cells from flash."""
    visited = set()
    try:
        with open("visited.txt", "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) == 2:
                    visited.add((int(parts[0]), int(parts[1])))
    except:
        pass
    return visited

def wait_for_button_press(solver):
    """Wait for button press. Returns hold duration in ms."""
    # Wait for button to be pressed
    while solver.btn.value() == 1:
        time.sleep(0.01)
    
    # Button is now pressed — measure how long they hold it
    press_start = time.ticks_ms()
    while solver.btn.value() == 0:
        time.sleep(0.01)
    
    hold_ms = time.ticks_diff(time.ticks_ms(), press_start)
    time.sleep(0.5)  # Debounce
    return hold_ms

def led_pattern(solver, pattern, duration_s=2.0):
    """Play an LED pattern for a set duration.
    Patterns: 'solid', 'slow_blink', 'fast_blink', 'double_blink', 'off'
    """
    start = time.ticks_ms()
    state = False
    last_toggle = start
    blink_count = 0
    
    while time.ticks_diff(time.ticks_ms(), start) < duration_s * 1000:
        now = time.ticks_ms()
        
        if pattern == "solid":
            solver.led.value(1)
        elif pattern == "slow_blink":
            if time.ticks_diff(now, last_toggle) > 500:
                state = not state
                solver.led.value(1 if state else 0)
                last_toggle = now
        elif pattern == "fast_blink":
            if time.ticks_diff(now, last_toggle) > 100:
                state = not state
                solver.led.value(1 if state else 0)
                last_toggle = now
        elif pattern == "double_blink":
            if time.ticks_diff(now, last_toggle) > 150:
                blink_count += 1
                state = not state
                solver.led.value(1 if state else 0)
                last_toggle = now
                if blink_count % 4 == 0:  # Pause after 2 blinks
                    time.sleep(0.4)
        
        time.sleep(0.01)
    solver.led.value(0)

def main():
    log("=== MICROMOUSE BOT READY ===")
    solver = MazeSolver()
    
    try:
        # ============================================
        # BOOT: Check if a saved map exists
        # ============================================
        saved_map_exists = has_saved_map()
        
        if saved_map_exists:
            # MAP FOUND! LED = DOUBLE BLINK (means "I have a map!")
            log("Saved map found on flash!")
            log("  Short press = SPEED RUN (use saved map)")
            log("  Hold 3 sec  = DELETE map & start fresh exploration")
            led_pattern(solver, "double_blink", 3.0)
        else:
            # NO MAP. LED = SOLID (means "ready to explore")
            log("No saved map. Ready to explore.")
            log("  Press button = Start exploration")
            led_pattern(solver, "solid", 2.0)
        
        # Wait for button press and measure hold duration
        solver.led.value(1)  # LED on while waiting
        hold_ms = wait_for_button_press(solver)
        solver.led.value(0)
        
        log(f"Button held for {hold_ms}ms")
        
        # ============================================
        # DECIDE WHAT TO DO
        # ============================================
        
        if hold_ms >= 3000:
            # LONG PRESS (3+ sec) = RESET MAP
            log("RESETTING saved map!")
            delete_saved_map()
            # Confirm reset: 5 fast blinks
            for _ in range(5):
                solver.led.value(1); time.sleep(0.1)
                solver.led.value(0); time.sleep(0.1)
            saved_map_exists = False
            # Now fall through to exploration
        
        solver.control_timer.init(period=2, mode=solver.control_timer.PERIODIC, callback=solver.control_loop)
        
        maze = Maze()
        visited = set()
        goals = [(3, 3), (3, 4), (4, 3), (4, 4)]  # 8x8 Center
        
        if saved_map_exists and hold_ms < 3000:
            # ============================================
            # SPEED RUN MODE (map already saved)
            # ============================================
            log("Loading saved map for speed run...")
            maze.load_from_file("maze_map.txt")
            visited = load_visited()
            log(f"Loaded {len(visited)} explored cells.")
            
            # LED solid during speed run
            solver.led.value(1)
            speed_run(maze, Heading.NORTH, goals, visited, solver)
            
        else:
            # ============================================
            # EXPLORATION MODE (no map or map was reset)
            # ============================================
            log("Starting exploration...")
            solver.led.value(1)  # LED ON while exploring
            
            result = explore_to_goal(maze, 0, 0, Heading.NORTH, goals, visited, solver)
            if result is None:
                raise Exception("Explore failed")
            
            x, y, heading = result
            
            # Return to start
            heading = return_to_start(maze, x, y, heading, visited, solver)
            if heading is None:
                raise Exception("Return failed")
            
            # SAVE THE MAP TO FLASH!
            maze.save_to_file("maze_map.txt")
            with open("visited.txt", "w") as f:
                for vx, vy in visited:
                    f.write(f"{vx},{vy}\n")
            log(f"Map saved! {len(visited)} cells explored.")
            
            solver.target_base_pwm = 0
            solver.set_speeds(0, 0)
            
            # Wait for button press for speed run (slow blink = ready)
            solver.control_timer.deinit()
            log("Press button for Speed Run...")
            
            # Slow blink while waiting
            solver.led.value(0)
            while solver.btn.value() == 1:
                solver.led.value(1); time.sleep(0.3)
                solver.led.value(0); time.sleep(0.3)
            time.sleep(1.0)
            
            solver.control_timer.init(period=2, mode=solver.control_timer.PERIODIC, callback=solver.control_loop)
            solver.led.value(1)
            
            speed_run(maze, heading, goals, visited, solver)
        
        # ============================================
        # FINISHED! Fast blink forever
        # ============================================
        solver.control_timer.deinit()
        solver.target_base_pwm = 0
        solver.set_speeds(0, 0)
        log("COMPETITION RUN COMPLETE!")
        
        while True:
            solver.led.value(1); time.sleep(0.1)
            solver.led.value(0); time.sleep(0.1)
            
    except KeyboardInterrupt:
        log("Bot stopped manually!")
    except Exception as e:
        log(f"Error: {e}")
        # Error: 5 fast blinks, pause, repeat
        while True:
            for _ in range(5):
                solver.led.value(1); time.sleep(0.05)
                solver.led.value(0); time.sleep(0.05)
            time.sleep(1.0)
    finally:
        solver.stop()
        try: solver.control_timer.deinit()
        except: pass

if __name__ == "__main__":
    main()
