# Micromouse

**Autonomous maze-solving robot · Raspberry Pi Pico · MicroPython**

A competition-winning micromouse built with three Sharp IR sensors and two N20 encoder motors. It maps walls, plans toward the maze centre using flood fill, and uses encoder feedback and wall correction to control movement.

I built and integrated the robot except for the 3D-printed chassis, with AI assistance during code development and debugging.

## Highlights

- **Navigation:** 8 × 8 grid, flood-fill exploration and return to start.
- **Motion:** PIO encoder counting and PD feedback for distance, turning and wall correction.
- **Memory:** saves maze walls and visited cells to onboard flash.
- **Repeat run:** groups straight sections into longer moves; current code needs fixes described below.

<!-- Add your maze-solving video link here. Add the competition name, year and placement when available. -->

## How it works

```text
Sense walls → Update map → Flood fill → Turn and move
                              ↓
                    Reach centre → Return
                              ↓
                     Save map → Repeat run
```

Starts at `(0,0)` facing north; targets the central four cells. Unknown passages initially count as open. The physical robot solved a competition maze; additional routines in this source snapshot have not all been independently verified on hardware.

## Run

1. Back up the Pico's current files.
2. In Thonny, select the Pico's MicroPython interpreter.
3. Copy all three Python files to the Pico filesystem root.
4. Check wiring and calibration with motor power disconnected, then test with wheels lifted.
5. Run `main.py`, place the robot at the start facing north, and use the button.

| Button action | Result |
| --- | --- |
| Short press, no saved map | Explore, return and save |
| Short press, saved map | Attempt repeat run |
| Hold ≥3 seconds at initial prompt | Clear map and explore again |
| Press after exploration/return | Attempt repeat run |

**Use `main.py` as the entry point.** Running `maze_solver.py` directly invokes a different runner. Keep motor power accessible: the button does not reliably interrupt every blocking move.

## Files

| File | Purpose |
| --- | --- |
| `main.py` | Run sequence, button modes and navigation |
| `maze.py` | Wall map, flood fill and map storage |
| `maze_solver.py` | Sensors, PIO encoders and motor control |

Generated on the Pico: `maze_map.txt` and `visited.txt`.

<details>
<summary><strong>Hardware and GPIO wiring</strong></summary>

| Component | Purpose |
| --- | --- |
| Raspberry Pi Pico | MicroPython controller and navigation logic |
| Three Sharp analog IR sensors | Left, front and right wall sensing |
| Two N20 geared motors with encoders | Differential drive and wheel feedback |
| Dual motor driver | Separate PWM and direction control for each motor |
| External LED | Ready, running and completion indications |
| Push button | Start, saved-map reset and some movement interruption checks |
| 3D-printed chassis | Mechanical platform; fabricated separately |
| Battery and power regulation | Motor and controller supply; specifications to be documented |

Exact sensor models, motor-driver model, motor gearing and battery specifications are not identified in the source. The configured encoder count is a calibration value, not a verified motor datasheet specification.

## GPIO connections

These are **GPIO numbers, not physical header-pin numbers**, taken from `MazeSolver.__init__()`.

| Signal | Pico GPIO | Configuration |
| --- | --- | --- |
| Right motor PWM | GP2 | 20 kHz PWM |
| Right motor direction 1 | GP3 | Digital output |
| Right motor direction 2 | GP4 | Digital output |
| Left motor direction 2 | GP5 | Digital output |
| Left motor direction 1 | GP6 | Digital output |
| Left motor PWM | GP7 | 20 kHz PWM |
| Right encoder A / B | GP10 / GP11 | PIO inputs with pull-ups |
| Left encoder A / B | GP13 / GP12 | PIO inputs with pull-ups |
| Status LED | GP14 | Active-high output |
| Push button | GP15 | Input with pull-up; pressed = LOW |
| Left IR analog output | GP26 | ADC input |
| Front IR analog output | GP27 | ADC input |
| Right IR analog output | GP28 | ADC input |

Connect the button between GP15 and ground. Use an appropriate resistor for the external LED. Supply motors through the motor driver, share signal ground, and verify sensor/encoder voltage compatibility with the Pico before connecting them. Driver enable/standby wiring depends on the actual driver and is not specified in the code.

</details>

<details>
<summary><strong>Calibration and known limitations</strong></summary>

Most tuning values are near the top of `maze_solver.py`.

| Setting | Current value | Meaning |
| --- | --- | --- |
| `WHEEL_DIAMETER_MM` | 45.1 | Effective wheel diameter for odometry |
| `TICKS_PER_REV` | 700.0 | Assumed encoder counts per wheel revolution |
| `TRACK_WIDTH_MM` | 93.0 | Effective left/right wheel separation |
| `CELL_DISTANCE_MM` | 200.0 | Cell centre-to-centre travel |
| `WALL_THRESHOLD_MM` | 100 | Mapping threshold for wall presence |
| `WALL_DETECT_RANGE_MM` | 95 | Side-wall correction detection range |
| `WALL_CENTER_TARGET_MM` | 63 | Target side-wall clearance |
| `WALL_CENTER_MIN_MM` / `WALL_CENTER_MAX_MM` | 50 / 75 | Side-wall correction dead band |
| `MAX_BASE_PWM` | 95 | Distance-controller output limit before later offsets/clamping |
| `TURN_BASE_PWM` | 70 | Turn-controller output limit before minimum-drive offset |

Use measured distances to tune wheel scale, then turns to tune effective track width. Check encoder direction before tuning gains. Calibrate `_adc_to_mm()` against the actual Sharp sensor model and mounting geometry; its current voltage-to-distance approximation is hardware-specific.

`FRONT_WALL_STOP_MM = 70` gates side-wall correction in the control loop. Despite its name, it does **not** independently stop forward motion when a front obstacle is close.

Changing the maze size also requires updating the goal coordinates in the runner. The current 200 mm cell setting and central goals are configured for an 8 × 8 layout.

## Current implementation limitations

These notes describe the supplied source snapshot; they do not invalidate the demonstrated competition run.

- **Repeat-run attribute mismatch:** `main.py` reads `solver.CELL_DISTANCE_MM`, but `CELL_DISTANCE_MM` exists only as a module-level constant in `maze_solver.py`. The repeat-run helper will raise `AttributeError` when it reaches this expression unless the reference or class is updated.
- **Route-planning termination:** repeat-run planning does not require every chosen neighbour to have a strictly decreasing flood-fill distance and has no cycle guard. Since flood-fill distances can include unknown passages while execution is restricted to visited cells, planning can cycle in some map states.
- **Movement completion reporting:** a timed-out distance move prints an error and returns without reporting failure to the navigation caller, which can still advance its logical cell position.
- **Blocking motion and stop checks:** forward-motion calls can block inside the hardware class; button checks in their outer wrappers cannot interrupt those calls. The continuous-distance routine and the runner's turning wrapper also lack their own timeout.
- **Error cleanup:** the runner's generic exception handler blinks indefinitely before its `finally` cleanup can execute. Explicit motor stop and timer shutdown should happen before entering that error display loop.
- **Turning conventions differ:** `main.py` intentionally reverses the quarter-turn signs used by `MazeSolver.turn_to_heading()`. Verify the intended entry point and physical motor/encoder conventions.
- **Calibration path:** a start-position calibration method exists, but `main.py` does not call it; the direct `MazeSolver.start()` path does.
- **Map updates only add walls:** a false-positive wall reading remains in the map until the map is reset. Saved-map loading is also not validated by the runner before continuing.
- **Timing and accuracy:** the configured timer rate, distance tolerances and turn thresholds are software settings, not independently measured control performance.

</details>

## License

No license selected yet. Add a LICENSE file to specify reuse permissions.

