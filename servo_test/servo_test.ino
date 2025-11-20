int servoPin = 8;

void setup() {
  pinMode(servoPin, OUTPUT);
}

void writeServo(int us) {
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(us);
  digitalWrite(servoPin, LOW);
  delay(20);
}

void loop() {
  writeServo(1500); // neutral
}
