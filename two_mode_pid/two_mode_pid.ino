/* Approach 2 — Two-Mode PID. Two fixed tunes (CRUISE for straights, TURN for sharp corners) and an
   automatic switch between them based on how far off the line the robot is.

   This sketch shares its entire base (hardware, sensors, calibration, loop skeleton, motor code)
   with dynamic_pid. The ONLY real difference is the marked "strategy" part below. See README.md. */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ===== HARDWARE (edit for your robot) =====
const int  NUM_SENSORS = 7;
const int  SENSOR_PINS[NUM_SENSORS] = { A8, A9, A10, A11, A12, A13, A14 };  // analog pins, LEFT -> RIGHT
const int  MOTOR_LEFT_PORT  = 1;            // Adafruit shield port for the left wheel
const int  MOTOR_RIGHT_PORT = 2;            //                    ... the right wheel
const bool LEFT_MOTOR_REVERSED  = false;    // set true if that wheel spins the wrong way
const bool RIGHT_MOTOR_REVERSED = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor  = AFMS.getMotor(MOTOR_LEFT_PORT);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(MOTOR_RIGHT_PORT);

// ===== TUNING (the strategy: two modes + an automatic switch) =====
// one tune per mode: { baseSpeed, minSpeed, KP, KI, KD, slowFactor }   (speeds are PWM 0..255)
struct Tune { int baseSpeed; int minSpeed; float KP; float KI; float KD; float slowFactor; };
Tune CRUISE = { 115, 85,  9.0, 0.0, 0.010,  5.0 };   // straights & gentle curves: faster, gentle steering
Tune TURN   = {  88, 45, 11.5, 0.0, 0.000, 12.0 };   // sharp corners: slower, stronger, brakes hard

const float        ENTER_TURN_ERROR = 2.9;     // error this high (held) -> switch to TURN
const float        EXIT_TURN_ERROR  = 0.6;     // error this low  (held) -> back to CRUISE
const int          ENTER_TURN_COUNT = 4;       // loops of high error needed to confirm entering a corner
const int          EXIT_TURN_COUNT  = 30;      // loops of low error needed to confirm leaving it
const unsigned long MIN_TURN_MS     = 4500;    // stay in TURN at least this long (rides through a long corner)

// ===== shared base (identical in both sketches) =====
const int           LOOP_HZ          = 200;
const unsigned long LOOP_US          = 1000000UL / LOOP_HZ;
const float         DT               = 1.0 / LOOP_HZ;
const int           SENSOR_DEADBAND  = 100;          // ignore normalized readings below this (noise)
const int           LINE_LOST_LEVEL  = 750;          // total signal below this = line not seen
const float         MAX_POSITION     = 3.0;          // |position| range
const int           MAX_WHEEL        = 255;          // max PWM sent to a wheel
const float         INTEGRAL_LIMIT   = 20.0;         // anti-windup clamp on the integral
const float         D_SMOOTHING      = 0.12;         // derivative low-pass, 0..1 (lower = smoother)
const float         SENSOR_WEIGHTS[NUM_SENSORS] = { -3, -2, -1, 0, 1, 2, 3 };  // widen ends to react harder at edges

// ===== calibration (identical in both; tweak freely) =====
const int           CALIB_READS       = 200;         // reads averaged per surface
const unsigned long CALIB_POSITION_MS = 2000;        // time to position the array before each sample
const int           LED               = LED_BUILTIN; // onboard LED signals the two phases

int   whiteValue[NUM_SENSORS], blackValue[NUM_SENSORS];
float position = 0.0, lastPosition = 0.0, prevPosition = 0.0, dFiltered = 0.0, integral = 0.0;
unsigned long previousTime = 0;
bool  inTurn = false;                                // extra state for the mode switch
unsigned long turnStart = 0;
int   enterCount = 0, exitCount = 0;

void setup() {
  calibrate();              // calibrate before starting the motors, so readings stay clean
  AFMS.begin();
  stopMotors();
  previousTime = micros();
}

void loop() {
  unsigned long now = micros();
  if (now - previousTime < LOOP_US) return;
  previousTime = now;

  position = readPosition();

  float dRaw = (position - prevPosition) / DT;
  dFiltered  = D_SMOOTHING * dRaw + (1.0 - D_SMOOTHING) * dFiltered;
  prevPosition = position;
  integral = constrain(integral + position * DT, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // ---------- STRATEGY (the only part that differs from dynamic_pid) ----------
  // pick a mode, then a plain linear PID using that mode's gains and speed
  updateMode();
  Tune t = inTurn ? TURN : CRUISE;
  float correction = t.KP * position + t.KI * integral + t.KD * dFiltered;
  float speed      = t.baseSpeed - t.slowFactor * fabs(position);
  speed = constrain(speed, (float)t.minSpeed, (float)t.baseSpeed);
  // ---------------------------------------------------------------------------

  int left  = constrain((int)round(speed + correction), 0, MAX_WHEEL);
  int right = constrain((int)round(speed - correction), 0, MAX_WHEEL);
  setMotors(left, right);
}

// strategy helper: switch CRUISE <-> TURN with debouncing + a minimum dwell time in TURN
void updateMode() {
  float a = fabs(position);
  if (!inTurn) {
    enterCount = (a > ENTER_TURN_ERROR) ? enterCount + 1 : 0;
    if (enterCount >= ENTER_TURN_COUNT) { inTurn = true; turnStart = millis(); enterCount = 0; }
  } else {
    if (millis() - turnStart < MIN_TURN_MS) { exitCount = 0; return; }
    exitCount = (a < EXIT_TURN_ERROR) ? exitCount + 1 : 0;
    if (exitCount >= EXIT_TURN_COUNT) { inTurn = false; exitCount = 0; }
  }
}

// ===== everything below is identical in both sketches =====

// weighted average of all sensors; holds the last value while the line is lost
float readPosition() {
  long weightedSum = 0, total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = constrain(map(analogRead(SENSOR_PINS[i]), whiteValue[i], blackValue[i], 0, 1000), 0, 1000);
    if (v < SENSOR_DEADBAND) v = 0;
    weightedSum += (long)(v * SENSOR_WEIGHTS[i]);
    total += v;
  }
  if (total < LINE_LOST_LEVEL) return lastPosition;
  float pos = constrain((float)weightedSum / total, -MAX_POSITION, MAX_POSITION);
  lastPosition = pos;
  return pos;
}

void setMotors(int left, int right) {
  driveMotor(leftMotor,  left,  LEFT_MOTOR_REVERSED);
  driveMotor(rightMotor, right, RIGHT_MOTOR_REVERSED);
}
void driveMotor(Adafruit_DCMotor *motor, int speed, bool reversed) {
  uint8_t fwd = reversed ? FORWARD : BACKWARD;
  uint8_t rev = reversed ? BACKWARD : FORWARD;
  if (speed > 0)      { motor->run(fwd); motor->setSpeed(constrain(speed, 0, 255)); }
  else if (speed < 0) { motor->run(rev); motor->setSpeed(constrain(-speed, 0, 255)); }
  else                { motor->run(RELEASE); motor->setSpeed(0); }
}
void stopMotors() {
  leftMotor->setSpeed(0);  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE); rightMotor->run(RELEASE);
}

// LED blinks = move into position; LED solid = sampling, hold still. White first, then black.
void calibrate() {
  pinMode(LED, OUTPUT);
  blinkFor(CALIB_POSITION_MS);  digitalWrite(LED, HIGH); sampleSurface(whiteValue); digitalWrite(LED, LOW);
  blinkFor(CALIB_POSITION_MS);  digitalWrite(LED, HIGH); sampleSurface(blackValue); digitalWrite(LED, LOW);
  for (int i = 0; i < NUM_SENSORS; i++)
    if (blackValue[i] < whiteValue[i] + 20) blackValue[i] = whiteValue[i] + 20;
}
void sampleSurface(int out[]) {
  long sums[NUM_SENSORS] = { 0 };
  for (int n = 0; n < CALIB_READS; n++)
    for (int i = 0; i < NUM_SENSORS; i++) { sums[i] += analogRead(SENSOR_PINS[i]); delay(1); }
  for (int i = 0; i < NUM_SENSORS; i++) out[i] = sums[i] / CALIB_READS;
}
void blinkFor(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { digitalWrite(LED, HIGH); delay(150); digitalWrite(LED, LOW); delay(150); }
}
