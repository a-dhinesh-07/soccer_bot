// =======================================================
// SmartElex 10D Smart Motor Driver
// FlySky RC Control – Differential Drive (OPTIMIZED)
// =======================================================

// ---------------- PIN DEFINITIONS ----------------
#define DIR1 4
#define DIR2 5
#define PWM1 9
#define PWM2 10

#define THROTTLE_PIN 2
#define STEERING_PIN 3

// ---------------- DASH / DRIFT CONFIG ----------------
#define DASH_HOLD_TIME 1000UL  // 1 second
#define DASH_DURATION 200UL
#define DRIFT_DURATION 150UL

// ---------------- RC SIGNAL CONSTANTS ----------------
#define MIN_PULSE 1000
#define MAX_PULSE 2000
#define CENTER_PULSE 1500
#define DEADZONE 50

#define RC_TIMEOUT 500000UL
#define PRINT_INTERVAL 10000UL

// ---------------- GLOBAL VARIABLES ----------------

// RC pulse capture (ISR)
volatile unsigned long throttleStart = 0;
volatile unsigned long steeringStart = 0;
volatile unsigned long throttleWidth = CENTER_PULSE;
volatile unsigned long steeringWidth = CENTER_PULSE;
volatile unsigned long lastThrottleSignal = 0;
volatile unsigned long lastSteeringSignal = 0;

// Dash / Drift state
bool dashActive = false;
bool driftActive = false;

unsigned long dashHoldStart = 0;
unsigned long dashStartTime = 0;
unsigned long driftStartTime = 0;

// Motor state (cached to avoid redundant writes)
int prevLeftSpeed = -1;
int prevRightSpeed = -1;
bool prevLeftDir = false;
bool prevRightDir = false;

// Debug timing
unsigned long lastPrintTime = 0;

// =======================================================
// INTERRUPTS
// =======================================================

void throttleInterrupt() {
  if (digitalRead(THROTTLE_PIN)) {
    throttleStart = micros();
  } else if (throttleStart) {
    throttleWidth = micros() - throttleStart;
    if (throttleWidth < MIN_PULSE) throttleWidth = MIN_PULSE;
    if (throttleWidth > MAX_PULSE) throttleWidth = MAX_PULSE;
    lastThrottleSignal = micros();
  }
}

void steeringInterrupt() {
  if (digitalRead(STEERING_PIN)) {
    steeringStart = micros();
  } else if (steeringStart) {
    steeringWidth = micros() - steeringStart;
    if (steeringWidth < MIN_PULSE) steeringWidth = MIN_PULSE;
    if (steeringWidth > MAX_PULSE) steeringWidth = MAX_PULSE;
    lastSteeringSignal = micros();
  }
}

// =======================================================
// SPECIAL ACTIONS
// =======================================================

void executeDash() {
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);

  if (millis() - dashStartTime >= DASH_DURATION) {
    dashActive = false;
  }
}

void executeDriftBrake() {
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM1, 180);
  analogWrite(PWM2, 180);

  if (millis() - driftStartTime >= DRIFT_DURATION) {
    driftActive = false;
  }
}

// =======================================================
// SETUP
// =======================================================

void setup() {
  Serial.begin(9600);

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN), steeringInterrupt, CHANGE);

  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);

  Serial.println("SmartElex 10D Optimized Drive Ready");
}

// =======================================================
// LOOP
// =======================================================

void loop() {

  // ---------- ATOMIC COPY ----------
  noInterrupts();
  unsigned long tWidth = throttleWidth;
  unsigned long sWidth = steeringWidth;
  unsigned long lastT = lastThrottleSignal;
  unsigned long lastS = lastSteeringSignal;
  interrupts();

  // ---------- FAILSAFE ----------
  unsigned long nowMicros = micros();
  if ((nowMicros - lastT > RC_TIMEOUT) || (nowMicros - lastS > RC_TIMEOUT)) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    return;
  }

  // ---------- DEADZONE ----------
  if (abs((int)tWidth - CENTER_PULSE) < DEADZONE) tWidth = CENTER_PULSE;
  if (abs((int)sWidth - CENTER_PULSE) < DEADZONE) sWidth = CENTER_PULSE;

  // ---------- FAST RC → MOTOR MAPPING (NO map()) ----------
  int throttle = ((int)tWidth - CENTER_PULSE) * 255 / 500;
  int steering = ((int)sWidth - CENTER_PULSE) * 255 / 500;

  // ---------- GESTURES ----------
  bool dashGesture = (throttle > 230) && (abs(steering) < 30);
  bool driftGesture = (abs(throttle) < 30) && (abs(steering) > 230);

  // ---------- DASH HOLD LOGIC ----------
  if (dashGesture && !dashActive) {
    if (!dashHoldStart) {
      dashHoldStart = millis();
    } else if (millis() - dashHoldStart >= DASH_HOLD_TIME) {
      dashActive = true;
      dashStartTime = millis();
      dashHoldStart = 0;
    }
  } else {
    dashHoldStart = 0;
  }

  // ---------- DRIFT TRIGGER ----------
  if (driftGesture && !driftActive && !dashActive) {
    driftActive = true;
    driftStartTime = millis();
  }

  // ---------- PRIORITY EXECUTION ----------
  if (dashActive) {
    executeDash();
  } else if (driftActive) {
    executeDriftBrake();
  } else {

    int leftMotor = throttle + steering;
    int rightMotor = throttle - steering;

    if (leftMotor > 255) leftMotor = 255;
    if (leftMotor < -255) leftMotor = -255;
    if (rightMotor > 255) rightMotor = 255;
    if (rightMotor < -255) rightMotor = -255;

    bool leftDir = (leftMotor >= 0);
    bool rightDir = (rightMotor >= 0);

    int leftSpeed = leftDir ? leftMotor : -leftMotor;
    int rightSpeed = rightDir ? rightMotor : -rightMotor;

    // ---------- WRITE ONLY IF CHANGED ----------
    if (leftSpeed != prevLeftSpeed || leftDir != prevLeftDir) {
      digitalWrite(DIR1, leftDir ? HIGH : LOW);
      analogWrite(PWM1, leftSpeed);
      prevLeftSpeed = leftSpeed;
      prevLeftDir = leftDir;
    }

    if (rightSpeed != prevRightSpeed || rightDir != prevRightDir) {
      digitalWrite(DIR2, rightDir ? HIGH : LOW);
      analogWrite(PWM2, rightSpeed);
      prevRightSpeed = rightSpeed;
      prevRightDir = rightDir;
    }
  }

  // ---------- DEBUG (10s) ----------
  unsigned long nowMillis = millis();
  if (nowMillis - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = nowMillis;
    Serial.print("T:");
    Serial.print(tWidth);
    Serial.print(" S:");
    Serial.print(sWidth);
    Serial.print(" D:");
    Serial.print(dashActive);
    Serial.print(" R:");
    Serial.println(driftActive);
  }
}
