#include <Servo.h>

Servo servo1;
Servo servo2;

const int SERVO1_PIN = 7;
const int SERVO2_PIN = 8;
const int BUTTON_PIN = 2; // Connect button between Pin 2 and GND

// Set your starting angles
int start1 = 40;
int start2 = 60;

void setup() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Use internal pull-up resistor. 
  // Input is HIGH when open, LOW when pressed.
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Move both to their original positions initially
  servo1.write(start1);
  servo2.write(start2);
  delay(1000); // Brief pause at start
}

void loop() {
  // Check if button is pressed (LOW signal)
  if (digitalRead(BUTTON_PIN) == LOW) {
    runServoSequence();
  }
}

// This function holds the specific movement logic you provided
void runServoSequence() {
  // 1) Servo 1: +50° (Total 90°)
  servo1.write(start1 + 50);
  delay(2000);

  // 2) Servo 2: +30° (Total 90°)
  servo2.write(start2 + 30);
  delay(2000);

  // 3) Servo 1: Return to start (40°)
  servo1.write(start1);
  delay(3000);

  // 4) Servo 2: -50° (Total 10°)
  servo2.write(start2 - 50);
  delay(2000);

  // 5) Return both to original position
  servo1.write(start1);
  servo2.write(start2);
  delay(2000);
}
