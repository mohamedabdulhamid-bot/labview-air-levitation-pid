const int trigPinBall = 2;
const int echoPinBall = 3;

const int trigPinHand = 10;
const int echoPinHand = 11;

const int motorPin1 = 6;
const int motorPin2 = 7;
const int enA = 9;

// PID gains
double Kp = 6;
double Ki = 0.0796;
double Kd = 1.1996;

// Ball distance
float durationBall, distanceBall;

// Hand distance
float durationHand, distanceHand;

// PID variables
double dt, integral, previous, output, lasttime = 0;
double lowerPWM = 180, upperPWM = 255;
double rangeCM = 40; // Tube length
double rangePWM = 70;

// Manual setpoint from LabVIEW
double manualSetpoint = -1; // -1 = no manual input

// Default setpoint if Hand sensor invalid
double defaultSetpoint = 22; // middle of tube

void setup() {
  pinMode(trigPinBall, OUTPUT);
  pinMode(echoPinBall, INPUT);

  pinMode(trigPinHand, OUTPUT);
  pinMode(echoPinHand, INPUT);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enA, OUTPUT);

  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  Serial.begin(9600);

  delay(500); // give sensors a moment to stabilize
}

void loop() {
  // --- Read Ball distance ---
  digitalWrite(trigPinBall, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinBall, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBall, LOW);
  durationBall = pulseIn(echoPinBall, HIGH);
  distanceBall = 44 - (durationBall * 0.0343 / 2); // invert: 0 at bottom

  // --- Read Hand distance ---
  digitalWrite(trigPinHand, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinHand, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinHand, LOW);
  durationHand = pulseIn(echoPinHand, HIGH);
  distanceHand = durationHand * 0.0343 / 2; // 0 at sensor

  // --- Clamp Hand readings to valid range ---
  if (distanceHand < 0) distanceHand = 0;
  if (distanceHand > 44) distanceHand = 44;

  // --- Check for Manual Setpoint from LabVIEW ---
  if (Serial.available()) {
    double val = Serial.parseFloat();
    if (val >= 0 && val <= 44) {
      manualSetpoint = val;  // use LabVIEW setpoint
    } else {
      manualSetpoint = -1;   // disable manual
    }
  }

  // --- Setpoint selection ---
  double setpoint;
  if (manualSetpoint >= 0) {
    setpoint = manualSetpoint; // Manual from LabVIEW
  } else {
    setpoint = distanceHand;   // Auto from Hand sensor
  }

  // --- PID calculation ---
  double now = millis();
  dt = (now - lasttime) / 1000.0;
  lasttime = now;

  double errorCM = setpoint - distanceBall;
  double currentPWM = map(analogRead(enA), 0, 255, 0, 1023);
  double desiredPWM = (errorCM / rangeCM * 100 * rangePWM) + lowerPWM;
  output = pid(desiredPWM - currentPWM);

  // --- Write PWM ---
  analogWrite(enA, output);

  // --- Serial output for LabVIEW / Debug ---
  Serial.print(distanceBall, 2); Serial.print(",");
  Serial.print(distanceHand, 2); Serial.print(",");
  Serial.print(setpoint, 2); Serial.print(",");
  Serial.println(output, 2);

  delay(30);
}

double pid(double error) {
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;

  double pidOutput = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

  if (pidOutput > upperPWM) pidOutput = upperPWM;
  else if (pidOutput < lowerPWM) pidOutput = lowerPWM;

  return pidOutput;
}
