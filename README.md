English | [简体中文](README.zh-CN.md)

# Arduino PID Line Follower

Two tested Arduino line-following sketches that adapt steering for straights and sharp corners, with a practical PID tutorial.

## Choose a sketch

| Sketch | Approach |
| --- | --- |
| [Dynamic PID](dynamic_pid/dynamic_pid.ino) | Continuous gain shaping: gentle near center, stronger at the edges. |
| [Two-Mode PID](two_mode_pid/two_mode_pid.ino) | Switches between CRUISE and TURN tunes with debouncing and minimum dwell time. |

Both include sensor calibration, filtered derivative, integral limiting, and corner slow-down.

## Run it

Tested hardware: Arduino Mega 2560, two TT gear motors, an Adafruit Motor Shield, and seven photoresistors.

1. Open one linked `.ino` sketch in Arduino IDE.
2. Install **Adafruit Motor Shield V2 Library** and its dependencies through Library Manager.
3. Select your board and port. Match the hardware constants to your wiring:
   sensors left to right on A8–A14, motors on shield ports 1 and 2 by default.
4. Upload, then calibrate the entire sensor array over white first, black second.
   The LED blinks while positioning and stays solid while sampling; hold still.
5. The robot drives after calibration. Start at low speed and check motor directions.

Other boards need compatible pins and hardware; another driver requires motor-code changes.

## Limits and tuning

- Calibration assumes black reads higher than white; reversed polarity is not detected.
- Wheel commands are PWM, not measured speed, and clamp to 0–`MAX_WHEEL`: no reverse or pivot turns.
- Lost-line handling holds the last position indefinitely; **there is no lost-line stop timeout**.
- The 200 Hz loop is a target, not a real-time guarantee; gains need tuning on your robot and track.
- Change one parameter at a time: start slow, tune `KP`, then `KD`; usually leave `KI` at 0.

## Tutorial

The [full tutorial](docs/tutorial.md) preserves the PID explanation, wiring and calibration details,
both algorithms, all parameter tables, tuning order, formal methods, and sensor side-weighting trade-offs.

- [Hardware and calibration](docs/tutorial.md#setup)
- [Parameters and tuning](docs/tutorial.md#parameters-and-how-to-tune-them)

## License

MIT — see [LICENSE](LICENSE).
