#include <NewPing.h>

#define SONAR_PIN 1
#define MAX_DISTANCE 200

NewPing sonar(SONAR_PIN, SONAR_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);
}

void loop() {
  delay(50);

  unsigned int uS = sonar.ping();

  if (uS == 0) {
    Serial.println("Out of range");
  } else {
    float distance_cm = uS / 58.0;  // floating point
    Serial.print("Distance: ");
    Serial.print(distance_cm, 2);  // print with 2 decimals
    Serial.println(" cm");
  }
}

// #include <NewPing.h>

// #define SONAR_PIN 12
// #define MAX_DISTANCE 200

// NewPing sonar(SONAR_PIN, SONAR_PIN, MAX_DISTANCE);

// void setup() {
//   Serial.begin(115200);
// }

// void loop() {
//   delay(50);
//   Serial.print("Ping: ");
//   Serial.print(sonar.ping_cm());
//   Serial.println("cm");
// }
