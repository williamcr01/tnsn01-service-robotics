#include <QTRSensors.h>

//-------------------- Button Settings --------------------
int button_pin = 0;
bool start = false;
bool last_button_state = HIGH;

//-------------------- Motor Settings --------------------
int motor1pin1 = 10;
int motor1pin2 = 11;

int motor2pin1 = 9;
int motor2pin2 = 6;

int left_speed = 0;
int right_speed = 0;

int base_speed = 80;
int turn_speed = 40;
int turn_delay = 850;

//-------------------- QTR sensor settings --------------------
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

//-------------------- PID settings --------------------
double K_p = 0.02;
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

  // Button setup
  pinMode(button_pin, INPUT_PULLUP);

  // Motor setup
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);

  // QTR sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, SensorCount);

  while (!start) {
    bool button_state = digitalRead(button_pin);
    //if (button_state == LOW && last_button_state == HIGH) {
    if (button_state == LOW) {
      Serial.println("pressed");
      start = true;
      Serial.print("start: ");
      Serial.println(start);
      delay(50);
    }
    last_button_state = button_state;
  }
}

void loop() {
  Serial.println(start);
  if (start == true) {
    Serial.println("In motor");
    qtr.read(sensorValues);

    uint16_t pos = qtr.readLineBlack(sensorValues);
    error = pos - 2500.0;  // 1000 if three sensors, 2000 if five sensors
    integrator += error * T_s;
    integrator = constrain(integrator, -integrator_max, integrator_max);
    double motor_speed = K_p * error + K_i * integrator + K_d * (error - last_error) / T_s;
    last_error = error;

    //right_speed = constrain(base_speed + motor_speed, -100, 100); // destroys differential if abs(base_speed + motor_speed) > 100
    //left_speed = constrain(base_speed - motor_speed, -100, 100); // destroys differential if abs(base_speed - motor_speed) > 100

    // preserves differential, but a bit slower
    right_speed = base_speed + motor_speed;
    left_speed = base_speed - motor_speed;

    //int[] aboveThreshold = getSensorThreshold(sensorValues, SensorCount);
    bool leftEdge = sensorValues[0] > 800;
    bool rightEdge = sensorValues[5] > 800;
    if (leftEdge) {
      //right turn
      performRightTurn();
    } else if (rightEdge) {
      //left turn
      performLeftTurn();
    }

    int max_speed = max(abs(right_speed), abs(left_speed));
    if (max_speed > 100) {
      right_speed = right_speed * 100 / max_speed;
      left_speed = left_speed * 100 / max_speed;
    }

    runMotors(left_speed, right_speed);

    delay(T_s * 1000);
  } else {
    runMotors(0, 0);
  }
}

void performLeftTurn() {
  runMotors(0, 0);
  delay(50);

  runMotors(-turn_speed, turn_speed);

  delay(turn_delay);

  runMotors(0, 0);
  delay(50);
  runMotors(base_speed, base_speed);
  delay(400);
}

void performRightTurn() {
  runMotors(0, 0);
  delay(50);

  runMotors(turn_speed, -turn_speed);

  delay(turn_delay);

  runMotors(0, 0);
  delay(50);
  runMotors(base_speed, base_speed);
  delay(400);
}

int getNbrOfBlackLines(uint16_t xs[], uint8_t sensorCount) {
  int blackCount = 0;
  for (uint8_t i = 0; i < sensorCount; i++) {
    if (xs[i] > 800) {
      blackCount++;
    }
  }
  return blackCount;
}

// int[] getSensorThreshold(uint16_t xs[], uint8_t sensorCount){
//   int[] aboveThreshold = [0, 0, 0, 0, 0, 0];
//   for (uint8_t i = 0; i < sensorCount; i++) {
//     if (xs[i] > 800) {
//       aboveThreshold[i] = 1;
//     }
//   }
//   return aboveThreshold;
// }

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