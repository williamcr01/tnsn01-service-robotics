int servoPin = 8;

void setup() {
  pinMode(servoPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Enter angle (0–180):");
}

// Send one servo pulse
void writePulse(int pulse) {
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(servoPin, LOW);
  delay(20 - pulse / 1000);   // maintain ~20ms frame
}

// Move to angle and hold it
void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);

  // Convert angle (0–180°) to pulse (1000–2000 µs)
  int pulse = map(angle, 0, 180, 1000, 2000);

  // Send ~1 second of pulses so servo reaches position
  for (int i = 0; i < 50; i++) {
    writePulse(pulse);
  }
}

void loop() {
  // If the user typed something:
  if (Serial.available()) {
    int angle = Serial.parseInt();   // Read number like "15"
    
    if (angle >= 0 && angle <= 180) {
      Serial.print("Moving to ");
      Serial.print(angle);
      Serial.println(" degrees");
      setServoAngle(angle);
    } else {
      Serial.println("Invalid angle. Enter 0–180.");
    }
  }
}
