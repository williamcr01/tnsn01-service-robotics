#include <Servo.h>

Servo servo1;
Servo servo2;

const int SERVO1_PIN = 7;
const int SERVO2_PIN = 8;

// Set your starting angles
int start1 = 40;
int start2 = 60;

void setup() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Move both to their original positions
  servo1.write(start1);
  servo2.write(start2);
  delay(5000);
}

void loop() {

  // 1) Servo 1: +90°
  servo1.write(start1 + 50);
  delay(2000);

  // 2) Servo 2: +30°
  servo2.write(start2 + 30);
  delay(2000);

  // 3) Servo 1: -90° (back to start)
  servo1.write(start1);
  delay(3000);

  // 4) Servo 2: -60°
  servo2.write(start2 - 50);
  delay(2000);

  // 5) Return both to original position
  servo1.write(start1);
  servo2.write(start2);
  delay(2000);
}