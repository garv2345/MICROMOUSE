# \# Micromouse — Autonomous Maze-Solving Robot

# 

# A Raspberry Pi Pico robot that senses maze walls, builds a grid map and navigates toward the centre using flood-fill path planning. Two encoder-equipped N20 motors provide differential drive, while three Sharp IR sensors measure the left, front and right wall distances.

# 

# I built and integrated the robot, apart from its 3D-printed chassis. The project involved electronics assembly, sensor integration, motor control, configuration and hardware debugging, with AI-assisted code development.

# 

# \## Demonstration and competition

# 

# The robot successfully navigated a competition maze and won the competition. A working maze-solving video is available; the video link, event name, year and exact placement will be added here when supplied.

# 

# The competition result describes the physical project. The source in this repository includes additional exploration, return and speed-run routines; not every routine in this snapshot has been independently verified on hardware.

# 

# \## What the software implements

# 

# \- An 8 × 8 maze model with reciprocal wall updates and boundary walls.

# \- Multi-goal breadth-first flood fill targeting the central four cells.

# \- Exploration that prefers lower-distance neighbours, then unvisited cells, then the current heading when ties remain.

# \- A recent-position check to help escape repeated exploration/return loops.

# \- Encoder-based distance and heading estimation using RP2040 PIO state machines.

# \- PID-style distance, turning, heading and side-wall correction; integral gains currently default to zero.

# \- Five-sample IR averaging and a close-range blind-zone heuristic.

# \- A nominal 2 ms control timer, corresponding to a requested 500 Hz callback rate.

# \- Flash storage for maze walls and visited cells.

# \- A planned repeat run that combines consecutive straight moves into longer segments.

# \- A push button and external status LED for operation.

# 

# \## Hardware

# 

# | Component | Purpose |

# | --- | --- |

# | Raspberry Pi Pico | MicroPython controller and navigation logic |

# | Three Sharp analog IR sensors | Left, front and right wall sensing |

# | Two N20 geared motors with encoders | Differential drive and wheel feedback |

# | Dual motor driver | Separate PWM and direction control for each motor |

# | External LED | Ready, running and completion indications |

# | Push button | Start, saved-map reset and some movement interruption checks |

# | 3D-printed chassis | Mechanical platform; fabricated separately |

# | Battery and power regulation | Motor and controller supply; specifications to be documented |

# 

# Exact sensor models, motor-driver model, motor gearing and battery specifications are not identified in the source. The configured encoder count is a calibration value, not a verified motor datasheet specification.

# 

# \## GPIO connections

# 

# These are \*\*GPIO numbers, not physical header-pin numbers\*\*, taken from `MazeSolver.\_\_init\_\_()`.

# 

# | Signal | Pico GPIO | Configuration |

# | --- | --- | --- |

# | Right motor PWM | GP2 | 20 kHz PWM |

# | Right motor direction 1 | GP3 | Digital output |

# | Right motor direction 2 | GP4 | Digital output |

# | Left motor direction 2 | GP5 | Digital output |

# | Left motor direction 1 | GP6 | Digital output |

# | Left motor PWM | GP7 | 20 kHz PWM |

# | Right encoder A / B | GP10 / GP11 | PIO inputs with pull-ups |

# | Left encoder A / B | GP13 / GP12 | PIO inputs with pull-ups |

# | Status LED | GP14 | Active-high output |

# | Push button | GP15 | Input with pull-up; pressed = LOW |

# | Left IR analog output | GP26 | ADC input |

# | Front IR analog output | GP27 | ADC input |

# | Right IR analog output | GP28 | ADC input |

# 

# Connect the button between GP15 and ground. Use an appropriate resistor for the external LED. Supply motors through the motor driver, share signal ground, and verify sensor/encoder voltage compatibility with the Pico before connecting them. Driver enable/standby wiring depends on the actual driver and is not specified in the code.

# 

# \## Repository structure

# 

# ```text

# .

# ├── main.py          # Primary runner: button modes, exploration, return and repeat run

# ├── maze.py          # Wall bitmasks, grid model, flood fill and map persistence

# └── maze\_solver.py   # Hardware interfaces, PIO encoders, control loop and motion routines

# ```

# 

# The robot generates two files on its MicroPython filesystem after exploration and return:

# 

# ```text

# maze\_map.txt         # Maze dimensions followed by wall bitmasks

# visited.txt          # Explored cell coordinates, one x,y pair per line

# ```

# 

# \## Navigation workflow

# 

# \### 1. Explore toward the centre

# 

# The robot begins at `(0, 0)`, facing north. At each cell it reads left/front/right wall distances, translates those observations into absolute directions and adds the detected walls to its map.

# 

# Flood fill assigns distance values outward from the goals `(3, 3)`, `(3, 4)`, `(4, 3)` and `(4, 4)`. The robot chooses an accessible neighbouring cell using these values, turns if needed and drives one cell forward.

# 

# Unknown internal edges initially behave as open passages. Exploration therefore plans against a partial map rather than a fully surveyed maze.

# 

# \### 2. Return and save

# 

# After reaching a goal, the runner recomputes distances toward `(0, 0)` and returns while continuing to sense walls. It then saves the wall map and visited cells to flash.

# 

# \### 3. Repeat run

# 

# The runner plans a route through visited cells and groups consecutive moves in the same direction into multi-cell segments. This is intended to reduce per-cell stopping. See the limitations below before using this mode: the current snapshot contains a blocking attribute mismatch and does not guarantee a globally shortest or fastest route.

# 

# \## Setup with Thonny

# 

# 1\. Back up any existing Pico files before replacing them.

# 2\. Use a Raspberry Pi Pico with MicroPython and select the appropriate Pico interpreter in Thonny. The exact firmware version used for this project was not recorded.

# 3\. Save `maze.py`, `maze\_solver.py` and `main.py` to the \*\*root of the Pico filesystem\*\*, keeping these filenames.

# 4\. Keep motor power disconnected while checking connections and initialising the board.

# 5\. Review GPIO assignments, wheel geometry, encoder signs, sensor calibration and motor direction before testing movement.

# 6\. Start with the wheels lifted, then test short movements in a clear area at reduced power.

# 7\. Run `main.py` from Thonny, or reboot the Pico to launch the saved `main.py` automatically.

# 

# The code uses MicroPython modules including `machine` and `rp2`; it is not a desktop Python application and does not require a desktop `pip` installation.

# 

# \*\*Entry point:\*\* use `main.py`. Running `maze\_solver.py` directly starts a separate legacy orchestration path with different calibration and turning behaviour. The two entry points should not be treated as interchangeable.

# 

# \## Button and LED behaviour

# 

# | Situation in `main.py` | Behaviour |

# | --- | --- |

# | Boot without a saved map | LED stays on for two seconds, then remains on while waiting |

# | Boot with a saved map | Double-blink pattern for three seconds, then LED on while waiting |

# | Short press without a saved map | Starts exploration and return |

# | Short press with a saved map | Attempts a repeat run using the stored map |

# | Hold for at least three seconds at the initial prompt | Deletes saved maze/visited files and starts fresh exploration |

# | After exploration and return | Slow blinking while waiting for the repeat-run button press |

# | Completed runner sequence | Repeated fast blinking |

# | Exception | Five quick flashes followed by a pause, repeated |

# 

# For a saved-map boot, place the robot at `(0, 0)` facing north in the same maze. A changed maze requires a fresh map.

# 

# The button is \*\*not a reliable emergency-stop system\*\* in this snapshot: some motion methods block before their caller can check the button. Keep an accessible motor-power disconnect during testing.

# 

# \## Calibration

# 

# Most tuning values are near the top of `maze\_solver.py`.

# 

# | Setting | Current value | Meaning |

# | --- | --- | --- |

# | `WHEEL\_DIAMETER\_MM` | 45.1 | Effective wheel diameter for odometry |

# | `TICKS\_PER\_REV` | 700.0 | Assumed encoder counts per wheel revolution |

# | `TRACK\_WIDTH\_MM` | 93.0 | Effective left/right wheel separation |

# | `CELL\_DISTANCE\_MM` | 200.0 | Cell centre-to-centre travel |

# | `WALL\_THRESHOLD\_MM` | 100 | Mapping threshold for wall presence |

# | `WALL\_DETECT\_RANGE\_MM` | 95 | Side-wall correction detection range |

# | `WALL\_CENTER\_TARGET\_MM` | 63 | Target side-wall clearance |

# | `WALL\_CENTER\_MIN\_MM` / `WALL\_CENTER\_MAX\_MM` | 50 / 75 | Side-wall correction dead band |

# | `MAX\_BASE\_PWM` | 95 | Distance-controller output limit before later offsets/clamping |

# | `TURN\_BASE\_PWM` | 70 | Turn-controller output limit before minimum-drive offset |

# 

# Use measured distances to tune wheel scale, then turns to tune effective track width. Check encoder direction before tuning gains. Calibrate `\_adc\_to\_mm()` against the actual Sharp sensor model and mounting geometry; its current voltage-to-distance approximation is hardware-specific.

# 

# `FRONT\_WALL\_STOP\_MM = 70` gates side-wall correction in the control loop. Despite its name, it does \*\*not\*\* independently stop forward motion when a front obstacle is close.

# 

# Changing the maze size also requires updating the goal coordinates in the runner. The current 200 mm cell setting and central goals are configured for an 8 × 8 layout.

# 

# \## Current implementation limitations

# 

# These notes describe the supplied source snapshot; they do not invalidate the demonstrated competition run.

# 

# \- \*\*Repeat-run attribute mismatch:\*\* `main.py` reads `solver.CELL\_DISTANCE\_MM`, but `CELL\_DISTANCE\_MM` exists only as a module-level constant in `maze\_solver.py`. The repeat-run helper will raise `AttributeError` when it reaches this expression unless the reference or class is updated.

# \- \*\*Route-planning termination:\*\* repeat-run planning does not require every chosen neighbour to have a strictly decreasing flood-fill distance and has no cycle guard. Since flood-fill distances can include unknown passages while execution is restricted to visited cells, planning can cycle in some map states.

# \- \*\*Movement completion reporting:\*\* a timed-out distance move prints an error and returns without reporting failure to the navigation caller, which can still advance its logical cell position.

# \- \*\*Blocking motion and stop checks:\*\* forward-motion calls can block inside the hardware class; button checks in their outer wrappers cannot interrupt those calls. The continuous-distance routine and the runner's turning wrapper also lack their own timeout.

# \- \*\*Error cleanup:\*\* the runner's generic exception handler blinks indefinitely before its `finally` cleanup can execute. Explicit motor stop and timer shutdown should happen before entering that error display loop.

# \- \*\*Turning conventions differ:\*\* `main.py` intentionally reverses the quarter-turn signs used by `MazeSolver.turn\_to\_heading()`. Verify the intended entry point and physical motor/encoder conventions.

# \- \*\*Calibration path:\*\* a start-position calibration method exists, but `main.py` does not call it; the direct `MazeSolver.start()` path does.

# \- \*\*Map updates only add walls:\*\* a false-positive wall reading remains in the map until the map is reset. Saved-map loading is also not validated by the runner before continuing.

# \- \*\*Timing and accuracy:\*\* the configured timer rate, distance tolerances and turn thresholds are software settings, not independently measured control performance.

# 

# \## Project ownership and evidence

# 

# The robot build and integration were completed by the repository owner, with the 3D-printed chassis produced separately and AI assistance used during code development and debugging. Competition details, the demonstration video and close-up hardware photographs can be added to this repository to document the build further.

# 

# \## License

# 

# No license has been selected yet. Add a `LICENSE` file before granting reuse permissions under a specific open-source license.



