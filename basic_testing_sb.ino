int L_DIR = 2, L_PWM = 3;
int R_DIR = 4, R_PWM = 5;

int ch1Pin = 8, ch2Pin = 9;
int ch1, ch2;

void setup() {
  pinMode(L_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
}

void driveMotor(int dirPin, int pwmPin, int speed) {
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -speed);
  }
}

void loop() {
  ch1 = pulseIn(ch1Pin, HIGH, 25000);
  ch2 = pulseIn(ch2Pin, HIGH, 25000);

  if (ch1 == 0 || ch2 == 0) return;

  ch1 = constrain(ch1, 1000, 2000);
  ch2 = constrain(ch2, 1000, 2000);

  int throttle = map(ch2, 1000, 2000, -255, 255);
  int steering = map(ch1, 1000, 2000, -255, 255);

  int leftSpeed  = constrain(throttle + steering, -255, 255);
  int rightSpeed = constrain(throttle - steering, -255, 255);

  driveMotor(L_DIR, L_PWM, leftSpeed);
  driveMotor(R_DIR, R_PWM, rightSpeed);
}
