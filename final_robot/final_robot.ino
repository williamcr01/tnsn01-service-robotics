#include <QTRSensors.h>

//-------------------- Motor Settings --------------------
int motor1pin1 = 10;
int motor1pin2 = 11;

int motor2pin1 = 9;
int motor2pin2 = 6;

int left_speed = 0;
int right_speed = 0;

int base_speed = 80;

//-------------------- QTR sensor settings --------------------
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

//-------------------- PID settings --------------------
double K_p = 0.1;
double K_i = 0.0;
double K_d = 0.0;
double T_s = 25.0 / 1000.0;  // 25 ms
double last_error = 0;
double integrator = 0;
double integrator_max = 1000;
double error;

void setup() {
  //Serial
  Serial.begin(9600);

  // Motor setup
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);

  // QTR sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A1, A2, A3, A4, A5 });
}

void loop() {
  qtr.read(sensorValues);

  uint16_t pos = qtr.readLineBlack(sensorValues);
  error = pos - 2000.0; // 1000 if three sensors, 2000 if five sensors
  integrator += error * T_s;
  integrator = constrain(integrator, -integrator_max, integrator_max);
  double motor_speed = K_p * error + K_i * integrator + K_d * (error - last_error) / T_s;
  last_error = error;

  //right_speed = constrain(base_speed + motor_speed, -100, 100); // destroys differential if abs(base_speed + motor_speed) > 100
  //left_speed = constrain(base_speed - motor_speed, -100, 100); // destroys differential if abs(base_speed - motor_speed) > 100

  // preserves differential, but a bit slower
  right_speed = base_speed + motor_speed;
  left_speed = base_speed - motor_speed;

  int max_speed = max(abs(right_speed), abs(left_speed));
  if (max_speed > 100) {
    right_speed = right_speed * 100 / max_speed;
    left_speed = left_speed * 100 / max_speed;
  }

  runMotors(left_speed, right_speed);

  delay(T_s * 1000);
}

void runMotors(int leftSpeed, int rightSpeed) {  //Function to run the motors, the Speeds can be set between -100 and 100
  if (rightSpeed >= 0 and rightSpeed != 0) {
    int Speed = map(rightSpeed, 0, 100, 0, 255);
    analogWrite(motor1pin1, Speed);
    digitalWrite(motor1pin2, LOW);
  }
  if (rightSpeed <= 0 and rightSpeed != 0) {
    int Speed = map(rightSpeed, -100, 0, 255, 0);
    digitalWrite(motor1pin1, LOW);
    analogWrite(motor1pin2, Speed);
  }
  if (rightSpeed == 0) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
  }
  if (leftSpeed >= 0 and leftSpeed != 0) {
    int Speed = map(leftSpeed, 0, 100, 0, 255);
    analogWrite(motor2pin1, Speed);
    digitalWrite(motor2pin2, LOW);
  }
  if (leftSpeed <= 0 and leftSpeed != 0) {
    int Speed = map(leftSpeed, -100, 0, 255, 0);
    digitalWrite(motor2pin1, LOW);
    analogWrite(motor2pin2, Speed);
  }
  if (leftSpeed == 0) {
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  }
}