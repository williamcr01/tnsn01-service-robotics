int servoPin = 8;

// You can tweak these if the servo isn't using the whole range:
const int MIN_PULSE = 500;  // us -> one end
const int MAX_PULSE = 2500;  // us -> other end

void setup() {
  pinMode(servoPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Enter angle from -90 to 90:");
}

// Send one servo pulse (frame ~20 ms)
void writePulse(int pulse) {
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(servoPin, LOW);
  delay(20 - pulse / 1000);   // keep total around 20 ms
}

// angleCentered: -90..+90 (what YOU type)
void setServoAngleCentered(int angleCentered) {
  // Clamp to [-90, 90]
  if (angleCentered < -90) angleCentered = -90;
  if (angleCentered >  90) angleCentered =  90;

  // Convert -90..+90 to 0..180
  int angle0to180 = angleCentered + 90;

  // Map 0..180 -> MIN_PULSE..MAX_PULSE
  int pulse = map(angle0to180, 0, 180, MIN_PULSE, MAX_PULSE);

  // Send pulses for ~1 second so it reaches and holds
  for (int i = 0; i < 50; i++) {
    writePulse(pulse);
  }

  Serial.print("angleCentered = ");
  Serial.print(angleCentered);
  Serial.print(" -> pulse = ");
  Serial.println(pulse);
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();   // e.g. -90, 0, 45, 90

    if (angle >= -90 && angle <= 90) {
      Serial.print("Moving to ");
      Serial.print(angle);
      Serial.println(" degrees (center-based)");
      setServoAngleCentered(angle);
    } else {
      Serial.println("Invalid angle. Enter -90 to 90.");
    }
  }
}
