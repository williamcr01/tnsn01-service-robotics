#include <QTRSensors.h>
#include <Servo.h>
#include <NewPing.h>

//-------------------- Sonar Settings --------------------
int sonar_pin = 1;
int max_dis = 200;

NewPing sonar(sonar_pin, sonar_pin, max_dis);

unsigned int uS;
float distance_cm;

//-------------------- Button Settings --------------------
int button_pin = 0;
bool start = false;
bool last_button_state = HIGH;

//-------------------- Motor Settings --------------------
int motor2pin1 = 5;
int motor2pin2 = 11;

int motor1pin1 = 3;
int motor1pin2 = 6;

int left_speed = 0;
int right_speed = 0;

int base_speed = 70;
int turn_speed = 40;
int turn_delay = 900;

//-------------------- Servo Settings --------------------
Servo servo1;
Servo servo2;

const int SERVO1_PIN = 7;
const int SERVO2_PIN = 8;

// Set starting angles
int servo_start1 = 35;
int servo_start2 = 120;

int pressure_button = 12;

//-------------------- QTR sensor settings --------------------
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int turnThreshold = 2;
int leftEdgeCount = 0;
int rightEdgeCount = 0;
int black_line = 650;

//-------------------- Control settings --------------------
double K_p = 0.05;
double K_i = 0.0;
double K_d = 0.008;
double last_error = 0;
double integrator = 0;
double integrator_max = 1000;
double error;

unsigned long last_loop_time = 0;
const unsigned long loop_interval = 35;

double T_s = loop_interval / 1000.0;  // 25 ms

//-------------------- Encoder settings --------------------

// Encoder pins
const uint8_t ENC1_A_PIN = 2;  // Hardware interrupt capable
const uint8_t ENC1_B_PIN = 9;
const uint8_t ENC2_A_PIN = 10;  // Not interrupt capable on Leonardo
const uint8_t ENC2_B_PIN = 4;

// Encoder tick counts (volatile because modified in ISR)
volatile long enc1_ticks = 0;
volatile long enc2_ticks = 0;

// For polling encoder 2
uint8_t enc2_last_state = 0;

// Island crossing PI controller
double island_Kp = 0.085;  // Tune these
double island_Ki = 0.05;
double island_integrator = 0;
double island_integrator_max = 70;

//-------------------- Maze settings --------------------

struct Neighbor {
  uint8_t x;
  uint8_t y;
};
struct Node {
  uint8_t x;
  uint8_t y;
  uint8_t degree;
  Neighbor neighbors[4];
};
static const uint8_t MAZE_WIDTH = 7;
static const uint8_t MAZE_HEIGHT = 7;
static const uint8_t NODE_COUNT = 49;
static const int8_t START[2] = { 3, 6 };

static const Node MAZE_GRAPH[NODE_COUNT] = {
  { 0, 0, 2, { { 0, 1 }, { 1, 0 }, { 255, 255 }, { 255, 255 } } },
  { 0, 1, 2, { { 0, 0 }, { 0, 2 }, { 255, 255 }, { 255, 255 } } },  // Island
  { 0, 2, 2, { { 0, 1 }, { 0, 3 }, { 255, 255 }, { 255, 255 } } },
  { 0, 3, 2, { { 0, 2 }, { 0, 4 }, { 255, 255 }, { 255, 255 } } },
  { 0, 4, 2, { { 0, 3 }, { 0, 5 }, { 255, 255 }, { 255, 255 } } },
  { 0, 5, 2, { { 0, 4 }, { 1, 5 }, { 255, 255 }, { 255, 255 } } },
  { 0, 6, 1, { { 1, 6 }, { 255, 255 }, { 255, 255 }, { 255, 255 } } },
  { 1, 0, 2, { { 0, 0 }, { 1, 1 }, { 255, 255 }, { 255, 255 } } },
  { 1, 1, 2, { { 1, 0 }, { 2, 1 }, { 255, 255 }, { 255, 255 } } },
  { 1, 2, 2, { { 1, 3 }, { 2, 2 }, { 255, 255 }, { 255, 255 } } },
  { 1, 3, 2, { { 1, 2 }, { 1, 4 }, { 255, 255 }, { 255, 255 } } },
  { 1, 4, 2, { { 1, 3 }, { 1, 5 }, { 255, 255 }, { 255, 255 } } },
  { 1, 5, 4, { { 0, 5 }, { 1, 4 }, { 1, 6 }, { 2, 5 } } },  // intersection deg=4
  { 1, 6, 2, { { 0, 6 }, { 1, 5 }, { 255, 255 }, { 255, 255 } } },
  { 2, 0, 2, { { 2, 1 }, { 3, 0 }, { 255, 255 }, { 255, 255 } } },
  { 2, 1, 2, { { 1, 1 }, { 2, 0 }, { 255, 255 }, { 255, 255 } } },
  { 2, 2, 2, { { 1, 2 }, { 2, 3 }, { 255, 255 }, { 255, 255 } } },
  { 2, 3, 2, { { 2, 2 }, { 3, 3 }, { 255, 255 }, { 255, 255 } } },
  { 2, 4, 1, { { 3, 4 }, { 255, 255 }, { 255, 255 }, { 255, 255 } } },
  { 2, 5, 3, { { 1, 5 }, { 2, 6 }, { 3, 5 }, { 255, 255 } } },  // intersection deg=3
  { 2, 6, 2, { { 2, 5 }, { 3, 6 }, { 255, 255 }, { 255, 255 } } },
  { 3, 0, 3, { { 2, 0 }, { 3, 1 }, { 4, 0 }, { 255, 255 } } },  // intersection deg=3
  { 3, 1, 2, { { 3, 0 }, { 3, 2 }, { 255, 255 }, { 255, 255 } } },
  { 3, 2, 2, { { 3, 1 }, { 4, 2 }, { 255, 255 }, { 255, 255 } } },
  { 3, 3, 2, { { 2, 3 }, { 4, 3 }, { 255, 255 }, { 255, 255 } } },
  { 3, 4, 3, { { 2, 4 }, { 3, 5 }, { 4, 4 }, { 255, 255 } } },  // intersection deg=3
  { 3, 5, 2, { { 2, 5 }, { 3, 4 }, { 255, 255 }, { 255, 255 } } },
  { 3, 6, 1, { { 2, 6 }, { 255, 255 }, { 255, 255 }, { 255, 255 } } },  // Starting node
  { 4, 0, 1, { { 3, 0 }, { 255, 255 }, { 255, 255 }, { 255, 255 } } },
  { 4, 1, 2, { { 4, 2 }, { 5, 1 }, { 255, 255 }, { 255, 255 } } },
  { 4, 2, 3, { { 3, 2 }, { 4, 1 }, { 4, 3 }, { 255, 255 } } },  // intersection deg=3 // Island
  { 4, 3, 3, { { 3, 3 }, { 4, 2 }, { 4, 4 }, { 255, 255 } } },  // intersection deg=3
  { 4, 4, 3, { { 3, 4 }, { 4, 3 }, { 5, 4 }, { 255, 255 } } },  // intersection deg=3
  { 4, 5, 2, { { 4, 6 }, { 5, 5 }, { 255, 255 }, { 255, 255 } } },
  { 4, 6, 2, { { 4, 5 }, { 5, 6 }, { 255, 255 }, { 255, 255 } } },
  { 5, 0, 1, { { 6, 0 }, { 255, 255 }, { 255, 255 }, { 255, 255 } } },
  { 5, 1, 2, { { 4, 1 }, { 6, 1 }, { 255, 255 }, { 255, 255 } } },
  { 5, 2, 2, { { 5, 3 }, { 6, 2 }, { 255, 255 }, { 255, 255 } } },
  { 5, 3, 2, { { 5, 2 }, { 6, 3 }, { 255, 255 }, { 255, 255 } } },
  { 5, 4, 2, { { 4, 4 }, { 5, 5 }, { 255, 255 }, { 255, 255 } } },
  { 5, 5, 2, { { 4, 5 }, { 5, 4 }, { 255, 255 }, { 255, 255 } } },
  { 5, 6, 2, { { 4, 6 }, { 6, 6 }, { 255, 255 }, { 255, 255 } } },
  { 6, 0, 2, { { 5, 0 }, { 6, 1 }, { 255, 255 }, { 255, 255 } } },
  { 6, 1, 2, { { 5, 1 }, { 6, 0 }, { 255, 255 }, { 255, 255 } } },
  { 6, 2, 1, { { 5, 2 }, { 255, 255 }, { 255, 255 }, { 255, 255 } } },
  { 6, 3, 2, { { 5, 3 }, { 6, 4 }, { 255, 255 }, { 255, 255 } } },
  { 6, 4, 2, { { 6, 3 }, { 6, 5 }, { 255, 255 }, { 255, 255 } } },
  { 6, 5, 2, { { 6, 4 }, { 6, 6 }, { 255, 255 }, { 255, 255 } } },
  { 6, 6, 2, { { 5, 6 }, { 6, 5 }, { 255, 255 }, { 255, 255 } } },
};

// Neighbor entries with {255, 255} are unused (padding to fixed 4 slots).

//-------------------- State --------------------
// flag for first loop timing
bool first;

int picked_up = 0;

bool is_intersection = false;
bool is_island = false;
char dirs[] = { 'R', 'R', 'L', 'S' };
//char dirs[] = { 'R', 'R', 'S', 'S', 'L', 'L' };
int curr_cmd = 0;

void setup() {
  // Sonar
  // None needed

  // Encoder
  setupEncoders();

  //Serial
  Serial.begin(9600);

  // Button setup
  pinMode(button_pin, INPUT_PULLUP);

  // Motor setup
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  // Servo setup
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  pinMode(pressure_button, INPUT_PULLUP);

  // Move both to their original positions
  servo1.write(servo_start1);
  servo2.write(servo_start2);

  // QTR sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, SensorCount);

  while (!start) {
    bool button_state = digitalRead(button_pin);
    //if (button_state == LOW && last_button_state == HIGH) {
    if (button_state == LOW) {
      Serial.println("pressed");
      start = !start;
      Serial.print("start: ");
      Serial.println(start);
      delay(50);
    }
    last_button_state = button_state;
  }
  calibrateSensors();
  //   Serial.println("Testing motors...");
  //   runMotors(80, 80);
  //   delay(2000);
  //   runMotors(0, 0);
  //   Serial.println("Test done");

  first = true;
}

void loop() {
  //Serial.println(start);
  if (start == true) {
    unsigned long now = millis();
    if ((now - last_loop_time >= loop_interval) || first == true) {  // keep loop timing consistent
      pollEncoder2();
      first == false;
      last_loop_time = now;
      //Serial.println("In motor");
      qtr.readCalibrated(sensorValues);

      uS = sonar.ping();

      if (uS == 0) {
        Serial.println("Out of range");
      } else {
        distance_cm = uS / 58.0;  // floating point
        // Serial.print("Distance: ");
        // Serial.print(distance_cm, 2);  // print with 2 decimals
        // Serial.println(" cm");
      }

      uint16_t pos = calculateLinePosition();
      error = pos - 1500.0;  // 1000 if three sensors, 2000 if five sensors
      //Serial.println(error);
      integrator += error * T_s;
      integrator = constrain(integrator, -integrator_max, integrator_max);
      double motor_speed = K_p * error + K_i * integrator + K_d * (error - last_error) / T_s;
      last_error = error;

      // Serial.print("Sensors: ");
      // for (int i = 0; i < SensorCount; i++) {
      //   Serial.print(sensorValues[i]);
      //   Serial.print(" ");
      // }
      // Serial.println();
      // Serial.print("Position: ");
      // Serial.println(pos);
      // Serial.print("Error: ");
      // Serial.println(error);

      //right_speed = constrain(base_speed + motor_speed, -100, 100); // destroys differential if abs(base_speed + motor_speed) > 100
      //left_speed = constrain(base_speed - motor_speed, -100, 100); // destroys differential if abs(base_speed - motor_speed) > 100

      // preserves differential, but a bit slower
      right_speed = base_speed + motor_speed;
      left_speed = base_speed - motor_speed;

      // Clamp to minimum of 0 (no reverse) and scale to preserve the differential
      int min_speed = min(right_speed, left_speed);
      if (min_speed < 0) {
        // Shift both up so the slower wheel is at 0
        right_speed -= min_speed;
        left_speed -= min_speed;
      }

      // Scale down if either exceeds 100
      int max_speed = max(right_speed, left_speed);
      if (max_speed > 100) {
        right_speed = right_speed * 100 / max_speed;
        left_speed = left_speed * 100 / max_speed;
      }

      //int[] aboveThreshold = getSensorThreshold(sensorValues, SensorCount);
      // bool leftEdge = sensorValues[0] > 800;
      // bool rightEdge = sensorValues[5] > 800;
      // if (leftEdge) {
      //   //right turn
      //   performRightTurn();
      // } else if (rightEdge) {
      //   //left turn
      //   performLeftTurn();
      // }

      if (distance_cm < 15 && distance_cm > 0) {
        //start = false;
        //turnAround();
        left_speed -= 30;
        right_speed -= 30;
        if (sensorValues[2] < black_line && sensorValues[3] < black_line) {
          turnAround();
        }
      }

      if (digitalRead(pressure_button) == LOW) {
        performPickup();

        // Wait until button is released before allowing next trigger
        while (digitalRead(pressure_button) == LOW) {
          delay(10);
        }
        delay(200);  // Small debounce delay
      }

      qtr.readCalibrated(sensorValues);

      if (sensorValues[0] >= black_line) {
        leftEdgeCount++;
        Serial.println("Left edge");
      } else {
        leftEdgeCount = 0;
      }
      if (sensorValues[5] >= black_line) {
        rightEdgeCount++;
        Serial.println("Right edge");
      } else {
        rightEdgeCount = 0;
      }
      uint16_t sensorSum = 0;
      for (int i = 0; i < SensorCount; i++) {
        sensorSum += sensorValues[i];
      }

      // If sum is very low, all sensors see white -> We are in an Island
      // Threshold depends on calibration, usually < 200-400
      if (sensorSum < 500) {
        runMotors(40, 40);
        delay(25);
        is_island = true;
      }

      if (leftEdgeCount >= turnThreshold && rightEdgeCount >= turnThreshold) {
        // Intersection
        //start = false;
        is_intersection = true;
      } else if (leftEdgeCount >= turnThreshold) {
        performLeftTurn();
        leftEdgeCount = 0;
        rightEdgeCount = 0;
      } else if (rightEdgeCount >= turnThreshold) {
        performRightTurn();
        leftEdgeCount = 0;
        rightEdgeCount = 0;
      }
      if (is_intersection || is_island) {
        Serial.println("Intersection/island");
        char cmd = dirs[curr_cmd];
        if (is_island) {
          cmd = 'I';
          Serial.println("Island");
        }

        switch (cmd) {
          case 'L':
            Serial.println("Intersection Left");
            performLeftTurnIntersect();
            break;
          case 'R':
            Serial.println("Intersection Right");
            performRightTurnIntersect();
            break;
          case 'S':
            Serial.println("Intersection Straight");
            // Drive straight through intersection
            runMotors(base_speed - 20, base_speed - 20);
            delay(400);  // Clear the intersection lines
            break;
          //case 'I':
          // BLIND DRIVE for Island
          // Serial.println("Island command");
          // runMotors(0, 0);
          // delay(500);  // Time it takes to cross the gap
          // //delay(3000);
          // // Then wait for line re-acquisition
          // qtr.readCalibrated(sensorValues);
          // uint16_t sensorSum = 0;
          // for (int i = 0; i < SensorCount; i++) {
          //   sensorSum += sensorValues[i];
          // }

          // // If sum is very low, all sensors see white -> We are in an Island
          // // Threshold depends on calibration, usually < 200-400
          // bool flag = true;
          // while (flag) {
          //   qtr.readCalibrated(sensorValues);
          //   runMotorsSame(30);
          //   //sensorSum = 0;
          //   for (int i = 0; i < SensorCount; i++) {
          //     //sensorSum += sensorValues[i];
          //     if (sensorValues[i] > black_line) { flag = false; }
          //   }
          //   delay(loop_interval);
          // }
          // runMotors(0, 0);
          // break;
          case 'I':
            Serial.println("Island - encoder closed-loop");
            runMotors(0, 0);
            delay(250);

            // Reset encoder counts and integrator
            noInterrupts();
            enc1_ticks = 0;
            enc2_ticks = 0;
            interrupts();
            island_integrator = 0;

            bool found_line = false;
            unsigned long island_start = millis();
            unsigned long max_island_time = 10000;
            unsigned long last_control_time = millis();

            int base_island_speed = 40;

            double last_tick_error = 0;

            while (!found_line && (millis() - island_start < max_island_time)) {
              // Poll encoder 2 since it's not on interrupt
              pollEncoder2();

              qtr.readCalibrated(sensorValues);

              // Check if any sensor found the line
              for (int i = 0; i < SensorCount; i++) {
                if (sensorValues[i] > black_line) {
                  found_line = true;
                  break;
                }
              }

              if (!found_line) {
                unsigned long now = millis();

                // Run control loop at fixed interval
                if (now - last_control_time >= 20) {
                  last_control_time = now;

                  // Read encoder counts atomically
                  noInterrupts();
                  long e1 = enc1_ticks;
                  long e2 = enc2_ticks;
                  interrupts();

                  // Error = difference in wheel travel
                  // Positive error means enc1 (right?) is ahead
                  static double last_tick_error = 0;
                  double tick_error = (double)(e1 - e2);
                  double tick_error_delta = tick_error - last_tick_error;
                  last_tick_error = tick_error;

                  island_integrator += tick_error_delta;
                  island_integrator = constrain(island_integrator, -island_integrator_max, island_integrator_max);

                  double correction = island_Kp * tick_error_delta + island_Ki * island_integrator;

                  // PI control
                  //island_integrator += tick_error;
                  //island_integrator = constrain(island_integrator, -island_integrator_max, island_integrator_max);

                  //double correction = island_Kp * tick_error + island_Ki * island_integrator;
                  //double correction = island_Kp * tick_error;

                  // Apply correction: if right wheel ahead, slow it down
                  int left_cmd = base_island_speed - (int)correction;
                  int right_cmd = base_island_speed + (int)correction;

                  // Clamp speeds
                  left_cmd = constrain(left_cmd, 0, 60);
                  right_cmd = constrain(right_cmd, 0, 60);

                  runMotors(left_cmd, right_cmd);

                  // Debug output
                  Serial.print("E1:");
                  Serial.print(e1);
                  Serial.print(" E2:");
                  Serial.print(e2);
                  Serial.print(" Err:");
                  Serial.print(tick_error_delta);
                  Serial.print(" Cor:");
                  Serial.println(correction);
                }
              }
            }

            runMotors(0, 0);
            delay(50);

            if (!found_line) {
              Serial.println("Warning: Island timeout!");
            } else {
              Serial.println("Line found!");
            }
            break;
        }
        curr_cmd++;
        is_intersection = false;
        is_island = false;
        // start = false;
      }

      runMotorsNew(left_speed, right_speed);

      //delay(T_s * 1000); // not consistent, doesnt keep code execution time in mind
    }
  } else {
    runMotors(0, 0);
  }
}

void setupEncoders() {
  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);

  // Attach interrupt for encoder 1 (pin 2 = interrupt 1 on Leonardo)
  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), enc1_isr, CHANGE);

  // Store initial state for encoder 2 polling
  enc2_last_state = digitalRead(ENC2_A_PIN);
}

void enc1_isr() {
  // Simple counting - just increment (direction doesn't matter for speed matching)
  enc1_ticks++;
}

void pollEncoder2() {
  // Call this frequently in your loop
  uint8_t state = digitalRead(ENC2_A_PIN);
  if (state != enc2_last_state) {
    enc2_ticks++;
    enc2_last_state = state;
  }
}

// Weights: sensor 1 = 0, sensor 2 = 1000, sensor 3 = 2000, sensor 4 = 3000
double calculateLinePosition() {
  uint32_t weighted_sum = 0;
  uint32_t total = 0;

  for (int i = 1; i <= 4; i++) {
    weighted_sum += (uint32_t)sensorValues[i] * (i - 1) * 1000;
    total += sensorValues[i];
  }

  if (total == 0) return 1500.0;  // Center value, line lost

  return (double)weighted_sum / total;  // Returns 0-3000, center is 1500
}

void performPickup() {
  runMotors(0, 0);
  delay(400);
  // ---- 1) Servo 1: slow move +80° ----
  int startPos = servo_start1;
  int endPos = servo_start1 + 80;

  for (int pos = startPos; pos <= endPos; pos++) {
    servo1.write(pos);
    delay(10);  // slow movement
  }

  delay(2000);

  // ---- 2) Servo 2: +30° ----
  servo2.write(servo_start2 + 30);
  delay(1000);

  // ---- 3) Servo 1: fast return ----
  servo1.write(servo_start1);
  delay(2000);

  // ---- 4) Servo 2: -80° ----
  servo2.write(servo_start2 - 80);
  delay(3000);

  // ---- 5) Return both ----
  servo1.write(servo_start1);
  servo2.write(servo_start2);
  delay(2000);
}

// void performLeftTurn() {
//   Serial.println("Turn left");
//   runMotors(base_speed, base_speed);
//   delay(200);
//   runMotors(0, 0);
//   delay(50);

//   runMotors(turn_speed, -turn_speed);

//   delay(turn_delay);

//   runMotors(0, 0);
//   delay(50);
//   runMotors(base_speed, base_speed);
//   delay(100);
// }

// void performRightTurn() {
//   Serial.println("Turn right");
//   runMotors(base_speed, base_speed);
//   delay(200);
//   runMotors(0, 0);
//   delay(50);

//   runMotors(-turn_speed, turn_speed);

//   delay(turn_delay);

//   runMotors(0, 0);
//   delay(50);
//   runMotors(base_speed, base_speed);
//   delay(100);
// }

void turnAround() {
  Serial.println("Turn");
  runMotors(0, 0);
  delay(50);

  runMotors(50, 50);
  delay(200);

  runMotors(turn_speed, -turn_speed);

  unsigned long startTime = millis();

  // Now keep turning until center sensors find the line again
  bool lineFound = false;
  int middleCount = 0;
  startTime = millis();
  while (!lineFound && (millis() - startTime < 6000)) {  // Timeout after 2 seconds
    qtr.readCalibrated(sensorValues);

    // Check if any of the center sensors see the line
    if (sensorValues[2] > black_line || sensorValues[3] > black_line) {
      middleCount++;
    }
    if (middleCount >= 3) {
      lineFound = true;
    }
    delay(25);
  }

  // Stop
  runMotors(0, 0);
  delay(50);
}

void performLeftTurn() {
  Serial.println("Turn left");

  // Stop briefly
  runMotors(0, 0);
  delay(50);

  runMotors(40, 40);
  delay(300);
  qtr.readCalibrated(sensorValues);
  Serial.println(sensorValues[2]);
  Serial.println(sensorValues[3]);
  //Serial.println(sensorValues[0]);
  if ((sensorValues[2] > black_line || sensorValues[3] > black_line)) {
    runMotors(0, 0);
    delay(400);
    is_intersection = true;
    leftEdgeCount = 0;
    rightEdgeCount = 0;
    runMotors(-40, -40);
    delay(300);
    return;
  }


  // Start turning
  runMotors(turn_speed, -turn_speed);

  // First, turn until we lose the line on the center sensors
  // (we need to clear the current line before looking for the new one)
  unsigned long startTime = millis();
  while (millis() - startTime < 300) {  // Turn for at least 300ms to clear current line
    qtr.readCalibrated(sensorValues);
    delay(25);
  }

  // Now keep turning until center sensors find the line again
  bool lineFound = false;
  int middleCount = 0;
  startTime = millis();
  while (!lineFound && (millis() - startTime < 3000)) {  // Timeout after 2 seconds
    qtr.readCalibrated(sensorValues);

    // Check if any of the center sensors see the line
    if (sensorValues[2] > black_line || sensorValues[3] > black_line) {
      middleCount++;
    }
    if (middleCount >= 3) {
      lineFound = true;
    }
    delay(25);
  }

  // Stop
  runMotors(0, 0);
  delay(50);

  if (!lineFound) {
    Serial.println("Warning: line not found after turn!");
  }
}

void performRightTurn() {
  Serial.println("Turn right");

  // Stop briefly
  runMotors(0, 0);
  delay(50);

  runMotors(40, 40);
  delay(300);
  qtr.readCalibrated(sensorValues);
  Serial.println(sensorValues[2]);
  Serial.println(sensorValues[3]);
  //Serial.println(sensorValues[5]);
  if ((sensorValues[2] > black_line || sensorValues[3] > black_line)) {
    runMotors(0, 0);
    delay(400);
    is_intersection = true;
    leftEdgeCount = 0;
    rightEdgeCount = 0;
    runMotors(-40, -40);
    delay(300);
    return;
  }

  // Start turning
  runMotors(-turn_speed, turn_speed);

  // First, turn until we clear the current line
  unsigned long startTime = millis();
  while (millis() - startTime < 300) {
    qtr.readCalibrated(sensorValues);
    delay(25);
  }

  // Now keep turning until center sensors find the line again
  bool lineFound = false;
  int middleCount = 0;
  startTime = millis();
  while (!lineFound && (millis() - startTime < 3000)) {
    qtr.readCalibrated(sensorValues);

    if (sensorValues[2] > black_line || sensorValues[3] > black_line) {
      middleCount++;
    }
    if (middleCount >= 3) {
      lineFound = true;
    }
    // int pos = calculateLinePosition();
    // if (pos - 1500 > -100 && pos - 1500 < 100) {
    //   lineFound = true;
    // }
    delay(25);
  }

  // Stop
  runMotors(0, 0);
  delay(50);

  if (!lineFound) {
    Serial.println("Warning: line not found after turn!");
  }
}

void performLeftTurnIntersect() {
  Serial.println("Turn left after intersect");

  // Stop briefly
  runMotors(0, 0);
  delay(50);

  runMotors(40, 40);
  delay(300);
  // qtr.readCalibrated(sensorValues);
  // if (sensorValues[2] > 750 || sensorValues[3] > 750) {
  //   runMotors(0, 0);
  //   delay(400);
  //   is_intersection = true;
  //   leftEdgeCount = 0;
  //   rightEdgeCount = 0;
  //   runMotors(-40, -40);
  //   delay(300);
  //   return;
  // }


  // Start turning
  runMotors(turn_speed, -turn_speed);

  // First, turn until we lose the line on the center sensors
  // (we need to clear the current line before looking for the new one)
  unsigned long startTime = millis();
  while (millis() - startTime < 300) {  // Turn for at least 300ms to clear current line
    qtr.readCalibrated(sensorValues);
    delay(25);
  }

  // Now keep turning until center sensors find the line again
  bool lineFound = false;
  int middleCount = 0;
  startTime = millis();
  while (!lineFound && (millis() - startTime < 3000)) {  // Timeout after 2 seconds
    qtr.readCalibrated(sensorValues);

    // Check if any of the center sensors see the line
    if (sensorValues[2] > 800 || sensorValues[3] > 800) {
      middleCount++;
    }
    if (middleCount >= 3) {
      lineFound = true;
    }
    delay(25);
  }

  // Stop
  runMotors(0, 0);
  leftEdgeCount = 0;
  rightEdgeCount = 0;
  delay(50);

  if (!lineFound) {
    Serial.println("Warning: line not found after turn!");
  }
}

void performRightTurnIntersect() {
  Serial.println("Turn right after intersetct");

  // Stop briefly
  runMotors(0, 0);
  delay(50);

  runMotors(40, 40);
  delay(300);
  // qtr.readCalibrated(sensorValues);
  // if (sensorValues[2] > 750 || sensorValues[3] > 750) {
  //   runMotors(0, 0);
  //   delay(400);
  //   is_intersection = true;
  //   leftEdgeCount = 0;
  //   rightEdgeCount = 0;
  //   runMotors(-40, -40);
  //   delay(300);
  //   return;
  // }

  // Start turning
  runMotors(-turn_speed, turn_speed);

  // First, turn until we clear the current line
  unsigned long startTime = millis();
  while (millis() - startTime < 300) {
    qtr.readCalibrated(sensorValues);
    delay(25);
  }

  // Now keep turning until center sensors find the line again
  bool lineFound = false;
  int middleCount = 0;
  startTime = millis();
  while (!lineFound && (millis() - startTime < 3000)) {
    qtr.readCalibrated(sensorValues);

    if (sensorValues[2] > 800 || sensorValues[3] > 800) {
      middleCount++;
    }
    if (middleCount >= 3) {
      lineFound = true;
    }
    // int pos = calculateLinePosition();
    // if (pos - 1500 > -100 && pos - 1500 < 100) {
    //   lineFound = true;
    // }
    delay(25);
  }

  // Stop
  runMotors(0, 0);
  leftEdgeCount = 0;
  rightEdgeCount = 0;
  delay(50);

  if (!lineFound) {
    Serial.println("Warning: line not found after turn!");
  }
}

// int getNbrOfBlackLines(uint16_t xs[], uint8_t sensorCount) {
//   int blackCount = 0;
//   for (uint8_t i = 0; i < sensorCount; i++) {
//     if (xs[i] > 800) {
//       blackCount++;
//     }
//   }
//   return blackCount;
// }

// int[] getSensorThreshold(uint16_t xs[], uint8_t sensorCount){
//   int[] aboveThreshold = [0, 0, 0, 0, 0, 0];
//   for (uint8_t i = 0; i < sensorCount; i++) {
//     if (xs[i] > 800) {
//       aboveThreshold[i] = 1;
//     }
//   }
//   return aboveThreshold;
// }

void runMotorsSame(int speed) {
  // If robot pulls LEFT, add negative trim (e.g., -5) to make left motor faster
  // If robot pulls RIGHT, add positive trim (e.g., 5) to make left motor slower
  int trim = -6;

  // Apply trim to one motor to balance them
  runMotorsNew(speed + trim, speed);
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

void runMotorsNew(int leftSpeed, int rightSpeed) {
  int minPWM = 0;  // Adjust this! Find the lowest value where motors still spin.

  if (rightSpeed > 0) {
    int Speed = map(rightSpeed, 0, 100, minPWM, 255);
    analogWrite(motor1pin1, Speed);
    digitalWrite(motor1pin2, LOW);
  } else if (rightSpeed < 0) {
    int Speed = map(abs(rightSpeed), 0, 100, minPWM, 255);
    digitalWrite(motor1pin1, LOW);
    analogWrite(motor1pin2, Speed);
  } else {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
  }

  if (leftSpeed > 0) {
    int Speed = map(leftSpeed, 0, 100, minPWM, 255);
    analogWrite(motor2pin1, Speed);
    digitalWrite(motor2pin2, LOW);
  } else if (leftSpeed < 0) {
    int Speed = map(abs(leftSpeed), 0, 100, minPWM, 255);
    digitalWrite(motor2pin1, LOW);
    analogWrite(motor2pin2, Speed);
  } else {
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  }
}

void calibrateSensors() {
  Serial.println("Calibrating... spinning over line");
  int delayTime = 20;
  int spinSpeed = 30;

  for (int i = 0; i < 20; i++) {
    qtr.calibrate();
    delay(delayTime);
  }

  // Spin one direction
  runMotors(-spinSpeed, spinSpeed);
  for (int i = 0; i < 20; i++) {
    qtr.calibrate();
    delay(delayTime);
  }

  // Spin back the other direction
  runMotors(spinSpeed, -spinSpeed);
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(delayTime);
  }

  // Return to roughly center
  runMotors(-spinSpeed, spinSpeed);
  bool centerFlag = false;
  while (centerFlag == false) {
    qtr.readCalibrated(sensorValues);
    // Serial.println(sensorValues[2]);
    if (sensorValues[2] > black_line && sensorValues[3] > black_line) {
      centerFlag = true;
    }
    qtr.calibrate();
    delay(delayTime);
  }

  // Stop
  runMotors(0, 0);

  Serial.println("Calibration done!");

  // Print calibration values
  Serial.print("Min: ");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Max: ");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(" ");
  }
  Serial.println();
}