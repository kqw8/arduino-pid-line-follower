English | [简体中文](README.zh-CN.md)

# Arduino PID Line Follower — Two Approaches

A tutorial for anyone learning to build a PID line-following robot, a classic entry point into
self-driving control. It covers two control approaches that go beyond a basic PID, along with how to
tune them. I built and tested both on a real robot, and both perform well.

## Contents

1. [Background: PID in brief](#background-pid-in-brief)
2. [The limitation of a single PID](#the-limitation-of-a-single-pid)
3. [Setup](#setup)
4. [Going beyond a basic PID](#going-beyond-a-basic-pid)
   - [Shared improvements (both approaches)](#shared-improvements-both-approaches)
   - [Approach 1: Dynamic PID](#approach-1-dynamic-pid)
   - [Approach 2: Two-Mode PID](#approach-2-two-mode-pid)
5. [Parameters and how to tune them](#parameters-and-how-to-tune-them)
6. [Going further: formal tuning methods](#going-further-formal-tuning-methods)
7. [An alternative idea: sensor side-weighting](#an-alternative-idea-sensor-side-weighting)

## Background: PID in brief

The robot carries a row of light sensors across its front. From their readings it estimates where the
black line lies relative to the center of the array. I call that estimate the **position**, and the
**error** is how far the position is from center: zero when the line is centered, positive when it has
drifted to the right, negative when it has drifted to the left.

Error appears because the track curves, the wheels are never perfectly matched, and the robot carries
momentum, so the line is continually moving off-center beneath it. Error is simply a measure of how
far off the robot is at any instant.

A PID controller converts that error into a steering command by speeding up one wheel relative to the
other. It combines three views of the error:

- **P (proportional):** a correction proportional to the current error. The larger the error, the
  harder it steers. This provides most of the response, but on its own it overshoots the center and
  oscillates.
- **I (integral):** the accumulated error over time. It removes a small, persistent offset that P
  leaves behind. Line followers rarely need much of it.
- **D (derivative):** the rate of change of the error. It acts as damping: as the robot returns
  toward the line, D reduces the correction early so it does not overshoot. If P behaves like a spring
  pulling the robot to the line, D is the shock absorber.

Combined:

```
correction = KP*error + KI*(sum of error) + KD*(rate of change of error)
left wheel  = speed + correction
right wheel = speed - correction
```

The gains `KP`, `KI`, `KD` are found by tuning.

## The limitation of a single PID

A real track mixes straights, sweeping curves, and tight corners. With a single fixed set of gains,
you eventually meet the same trade-off no matter how you adjust them:

- Strong enough for the corners, and the straights oscillate.
- Smooth on the straights, and the corners understeer and cut the line.

A compromise setting may require a lower speed. Fixed gains can work well on some tracks, but adapting
the response helps balance gentle steering on straights with stronger steering in tight corners. The
two approaches below address that trade-off in different ways.

## Setup

My robot is an Arduino Mega 2560 driving two TT gear motors through an Adafruit Motor Shield,
with a reflectance array of seven photoresistors across the front. The hardware is otherwise
unremarkable, so this guide concentrates on the algorithms and tuning rather than the build.

The code consists of two standalone C++ `.ino` sketches:

- [Dynamic PID](dynamic_pid/dynamic_pid.ino)
- [Two-Mode PID](two_mode_pid/two_mode_pid.ino)

Open one sketch in the Arduino IDE, install the **Adafruit Motor Shield V2 Library** and its required
dependencies through Library Manager, then select your board and port. The sketches include `Wire.h`
and `Adafruit_MotorShield.h`. Adjust the hardware constants before uploading: the defaults use analog
pins A8–A14, shield ports 1 and 2, and seven sensors ordered left to right. Other boards need compatible
pins and hardware; another motor driver requires changes to the motor code.

At startup, the LED blinks while you position the entire sensor array over white, then stays solid
while sampling. Repeat over black for the second phase. Calibration assumes black produces a higher
analog reading than white; the code forces a minimum difference of 20 counts rather than detecting
reversed sensor polarity. Each phase averages 200 readings per sensor, with 2000 ms for positioning.
The robot begins driving after calibration. Start at low speed and check motor directions first.

Wheel commands are PWM values, not measured wheel speeds. Both loops clamp wheel commands to
0–`MAX_WHEEL`, so they do not command reverse motion or pivot turns. If the line is lost, the robot
holds the last position indefinitely; there is no lost-line stop timeout.

## Going beyond a basic PID

Both sketches improve on a basic PID in the same set of ways, and then diverge on a single point: the
steering strategy. This section covers all three parts — first the shared base common to both, then
each approach's distinct strategy.

### Shared improvements (both approaches)

Both sketches share the same base. The only real difference between them is the steering strategy (the
two approaches below). Everything here is common to both.

#### Signal quality

**1. Sensor calibration and normalization.** Every reflectance sensor is slightly different in
sensitivity and response to ambient light, and the readings change when the lighting changes. Using
raw `analogRead` values would skew the result if, for example, one sensor reads 200 on white while
another reads 60. Calibration measures each sensor's own white and black values at startup, then
`map()` rescales every sensor to a common range of 0 (its white) to 1000 (its black). After that,
every sensor reports on the same scale regardless of its quirks or the lighting.

**2. Deadband noise rejection.** Even after calibration, a sensor over white does not read exactly
zero; it fluctuates by a few counts. Fed into the weighted average and multiplied by the edge weights,
that noise accumulates and makes the position jump. The deadband treats anything below
`SENSOR_DEADBAND` as white or noise and sets it to zero, so only sensors actually over the line
contribute. Without it, noise alone makes the robot oscillate on a straight.

**3. Weighted-average position.** The simplest reading is "which sensor is darkest," which gives a
coarse integer position (-3, -2, ... 3) that jumps a whole step as the line moves between sensors. A
weighted average, `position = Σ(value × weight) / Σ(value)`, blends partial readings from neighboring
sensors into a smooth fractional value such as 1.5. A continuous position produces continuous
steering rather than stepwise corrections, and it allows the outer sensors to be weighted more
heavily so the robot reacts sooner once the line reaches an edge.

#### The PID itself

**4. Low-pass filtered derivative.** D is the rate of change of error, `(error − last error) / dt`.
Error is noisy, the difference of two noisy values is noisier still, and dividing by a small `dt`
amplifies it again, so raw D is too noisy to use and makes the steering jitter. A low-pass filter (an
exponential moving average, `dFiltered = α·new + (1−α)·old`) keeps only a fraction of each new value,
smoothing the spikes while preserving the real trend. D is the damping that prevents the robot from
overshooting the line, and filtering is what makes it usable in practice.

**5. Integral anti-windup.** I is the running sum of error. If the robot stays off the line for a
while, such as during a long corner or a brief loss of the line, that sum keeps growing. Once the
robot is back on the line, the accumulated value keeps pushing and causes a delayed overshoot.
Clamping the integral to ±`INTEGRAL_LIMIT` lets it correct small offsets without growing without
bound.

#### Behavior and robustness

**6. Automatic slow-down for corners.** A fixed-speed robot must pick one speed that works
everywhere: too fast and it cannot hold the corners, too slow and it wastes the straights. Instead the
speed follows the error: `speed = base − slowFactor·|error|`. On a straight the speed is at maximum;
the more the robot leans into a turn, the more it slows. This is also physically reasonable, since a
sharp turn demands more grip (centripetal force grows with the square of speed) and slowing down
provides more time to react.

**7. Hold the last position when the line is lost.** If every sensor reads white, for example when
overshooting a sharp corner, naive code computes a position of zero, assumes it is centered, and
drives straight off the track. Instead, when the total signal is too low to trust, the code returns
the last valid position. Since the line was last seen toward one side, the robot keeps steering that
way and usually reacquires it.

**8. Rate-gated control loop (target: 200 Hz).** Arduino's `loop()` runs as fast as it can. The sketches
skip updates until `LOOP_US` has elapsed and use a fixed `DT` for the derivative and integral. At
200 Hz, this assumes a 5 ms update interval. This is a target, not a real-time guarantee: sensor reads
and motor communication can delay updates. If execution exceeds the interval, actual timing differs
from `DT`, affecting the gains' effective behavior.

#### Practical conveniences

**9. Runtime calibration.** Track color and room lighting vary, so hard-coded sensor values only
work on the setup they were measured on. The robot recalibrates at every power-up, averaging several
hundred reads to reduce noise. With no display, the onboard LED signals the two phases: it blinks
while you move the array into position and stays solid while it samples (hold still), white first and
then black.

**10. Motor-direction flags.** When motors are wired and mounted, one wheel often spins the wrong way
for a given command. A boolean per motor (`LEFT_MOTOR_REVERSED`, `RIGHT_MOTOR_REVERSED`) reverses its
direction in software, which fixes the problem in one line instead of rewiring. If a wheel spins the
wrong way the robot turns in place; if the whole robot drives backward, reverse both.

### Approach 1: Dynamic PID

Both approaches aim for the same thing: keep the correction small and gentle on the straights, but
turn hard into the sharp corners. Approach 1 does this continuously.

Instead of a fixed `KP`, the steering gain scales with the error through a curve:

```
shape(pos) = pos * ( STEER_SOFTNESS + (1 - STEER_SOFTNESS) * (pos / MAX_POSITION)^2 )
```

Near the center the multiplier is about `STEER_SOFTNESS` (small), so corrections are gentle and the
robot does not oscillate on a straight. Toward the edge the multiplier approaches 1.0, so the robot
steers hard into a corner. Combined with the automatic slow-down, this gives one continuous loop that
is fast and calm on straights and strong and slow in corners, with no modes and no abrupt changes.

### Approach 2: Two-Mode PID

The same goal, reached differently. Here the steering remains an ordinary linear PID, but there are
two complete preset tunes:

- **CRUISE:** faster, with gentle steering, for straights and easy curves.
- **TURN:** slower, with stronger steering and harder braking, for sharp corners.

A small state machine switches between them based on the error. It enters TURN when the error stays
high for a few loops, and returns to CRUISE when the error stays low for a while. Requiring the
condition to hold for several loops debounces the switch so noise does not flip it, and a minimum
dwell time in TURN (`MIN_TURN_MS`) keeps the robot in that mode through a long corner section, such as
a back-and-forth square, so it does not switch back mid-corner and lose the line.

In short, Approach 1 adapts with one continuous curve, and Approach 2 adapts by switching between two
fixed tunes. Comparing the two is the point of this repository.

## Parameters and how to tune them

### Shared parameters (both sketches)

| Parameter | What it does |
| --- | --- |
| `SENSOR_PINS`, `NUM_SENSORS` | your analog sensor pins, left to right |
| `MOTOR_LEFT_PORT`, `MOTOR_RIGHT_PORT` | Adafruit shield ports for each wheel |
| `LEFT_MOTOR_REVERSED`, `RIGHT_MOTOR_REVERSED` | reverse a wheel that spins the wrong way |
| `SENSOR_WEIGHTS` | position weight of each sensor; widen the ends for sharper edge response |
| `MAX_POSITION` | position clamp and dynamic steering normalization (default: 3.0) |
| `SENSOR_DEADBAND` | readings below this are treated as white/noise |
| `LINE_LOST_LEVEL` | total signal below this means the line is lost |
| `D_SMOOTHING` | derivative low-pass, 0..1 (lower is smoother, with more lag) |
| `INTEGRAL_LIMIT` | anti-windup clamp on the integral |
| `MAX_WHEEL` | maximum PWM sent to a wheel |
| `LOOP_HZ` | control loop rate (200 Hz) |
| `CALIB_READS`, `CALIB_POSITION_MS` | calibration samples, and time to position the array |

### Approach 1 parameters

| Parameter | What it does |
| --- | --- |
| `TOP_SPEED` | speed on the straights |
| `MIN_SPEED` | speed floor in turns |
| `SLOW_FACTOR` | how strongly it slows as the error grows |
| `KP` | steering strength |
| `KI` | integral gain (usually left at 0) |
| `KD` | damping; increase if it oscillates |
| `STEER_SOFTNESS` | below 1, soft near center and sharp at the edges; 1 is plain linear |

### Approach 2 parameters

| Parameter | What it does |
| --- | --- |
| `CRUISE` / `TURN` tunes | `{ baseSpeed, minSpeed, KP, KI, KD, slowFactor }` for each mode |
| `ENTER_TURN_ERROR` | error above this (held) switches into TURN |
| `EXIT_TURN_ERROR` | error below this (held) switches back to CRUISE |
| `ENTER_TURN_COUNT` | loops of high error required to confirm a corner |
| `EXIT_TURN_COUNT` | loops of low error required to confirm the corner is over |
| `MIN_TURN_MS` | minimum time to remain in TURN once entered |

### Tuning order (both)

Change one parameter at a time:

1. Start slow. Establish clean tracking first, then raise the speed.
2. `KP`: too low and it cuts corners and drifts wide; too high and it oscillates.
3. `KD`: increase it to remove oscillation; if it becomes twitchy, smooth it with `D_SMOOTHING`.
4. `KI`: leave at 0 unless the robot consistently rides to one side and never centers.
5. Approach 1: `SLOW_FACTOR` sets how hard it brakes into corners, and `STEER_SOFTNESS` (around 0.3
   to 0.5) shapes the steering. Approach 2: tune CRUISE on a straight and TURN on a sharp corner, then
   set the error thresholds and `MIN_TURN_MS` so it enters TURN at the hard sections and holds it.

## Going further: formal tuning methods

Everything above is tuned by hand. If you want to study the underlying control theory, there are
formal methods that compute the gains instead.

- The rigorous approach is **frequency-domain (Bode plot) loop-shaping design**: you build a
  mathematical model of the system, examine its frequency response, and design the controller to meet
  target stability margins. This is essentially a full semester of a controls course.
  ([introduction: PID tuning with Bode plots, LibreTexts](https://eng.libretexts.org/Bookshelves/Industrial_and_Systems_Engineering/Chemical_Process_Dynamics_and_Controls_(Woolf)/09%3A_Proportional-Integral-Derivative_(PID)_Control/9.04%3A_PID_tuning_via_Frequency_Responses_with_Bode_Plots))
- A lighter recipe is the **Ziegler–Nichols method**: increase P until the robot oscillates steadily,
  measure that oscillation, and read the gains from a table.
  ([introduction: Ziegler–Nichols method, Wikipedia](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method))

These are included so you know the formal options exist and where to read about them. For a small
robot like this, they are generally more than you need. They require a reasonable model of the system,
and the hardware (inexpensive sensors, a flexible chassis, motors that do not quite match) introduces
enough error that the results are often inaccurate and difficult to apply. In most cases, tuning
against the actual track will get you there faster.

## An alternative idea: sensor side-weighting

There is another way to get a stronger response in corners: rather than shaping the gain, increase the
weights of the outer sensors in the position calculation. Once the robot drifts far enough that an
edge sensor catches the line (meaning it is already well into a corner), the weighted average produces
a much larger position value, and an ordinary linear PID then produces a much larger correction.

The underlying goal is the same as the Dynamic PID in Approach 1: a bigger correction in corners than
on straights. The difference is where the nonlinearity lives. Approach 1 keeps the position
measurement faithful (a true −3 to +3) and bends the gain in software. Side-weighting bends the
measurement itself by reshaping the weights, then runs a plain linear PID on top. That is part of its
appeal: it is arguably the simplest way to get soft-center, sharp-edge behavior, since the controller
stays a basic linear PID and all of the nonlinearity sits in one array of weights.

The trade-off is that amplifying the outer weights amplifies everything at the edges, not only the
signal. A heavily weighted sensor dominates the weighted average, so a small reading error on an outer
sensor pulls the computed position a long way and produces a large, unwanted steering correction. Such
errors are easy to come by: a noisy or slightly miscalibrated outer sensor, ambient light leaking in,
or a chassis that flexes or tilts so the array shifts relative to the line. With modest weights the
same reading error barely moves the position; with large weights it can dominate.

Both supplied sketches clamp position to ±`MAX_POSITION` (3.0 by default). Increasing outer weights
alone therefore reaches that limit sooner rather than producing an unlimited position value. Revisit
the clamp, gains, and (for Dynamic PID) shaping normalization together when experimenting.

So the method is only as reliable as your edge sensors and your mechanical stability. If the sensors
are accurate and consistent and the chassis is rigid, it is a clean and minimal way to achieve strong
corner response. If the hardware is rougher, the gain-shaping in Approach 1 is the safer choice,
because it leaves the raw measurement intact and adds the nonlinearity where sensor noise is not
amplified. I did not pursue side-weighting here for that reason, but it is a sound idea and worth
experimenting with if your hardware supports it.

## License

MIT, see [LICENSE](LICENSE).
