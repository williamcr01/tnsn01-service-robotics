// Motor test code

// ===== Motor pins =====
int motor2pin1 = 11;  // PWM forward
int motor2pin2 = 10;  // PWM reverse

int motor1pin1 = 6;   // PWM forward
int motor1pin2 = 9;   // PWM reverse

// Set speed between -100 and 100
int motorSpeed = 60;  // change this value to set speed

// ===== Encoder pins =====
// Motor 1 (external interrupt on A)
const uint8_t ENC1_A_PIN = 3;   // A (INT)
const uint8_t ENC1_B_PIN = 2;   // B

// Motor 2 (polled in loop, edge-detect on A)
const uint8_t ENC2_A_PIN = 4;   // A (polled)
const uint8_t ENC2_B_PIN = 5;   // B (polled)

// ===== Encoder/RPM state =====
volatile long enc1_count = 0;   // signed counts Motor 1
volatile long enc2_count = 0;   // signed counts Motor 2

const unsigned long RPM_SAMPLE_MS = 250; // sampling window for RPM
const float PULSES_PER_REV = 20.0f;      // A-channel rising edges per shaft rev

unsigned long lastSample = 0;
long last_enc1 = 0;
long last_enc2 = 0;

float rpm1 = 0.0f;
float rpm2 = 0.0f;

// For Motor 2 polling (edge detect A)
uint8_t enc2_a_prev = LOW;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Motor pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  // Encoder pins
  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);

  // Motor 1: external interrupt on A rising edge
  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), enc1A_ISR, RISING);

  // Init prev state for Motor 2 A
  enc2_a_prev = digitalRead(ENC2_A_PIN);

  // Start motors
  runMotors(motorSpeed);

  Serial.println("RPM monitoring started...");
}

void loop() {
  // ---- Motor 2 encoder polling (edge-detect on A, read B for direction) ----
  uint8_t a = digitalRead(ENC2_A_PIN);
  if (enc2_a_prev == LOW && a == HIGH) { // Rising edge on A
    bool b = digitalRead(ENC2_B_PIN);
    enc2_count += b ? -1 : +1;           // flip signs if reversed
  }
  enc2_a_prev = a;

  // ---- Periodic RPM computation ----
  unsigned long now = millis();
  if (now - lastSample >= RPM_SAMPLE_MS) {
    noInterrupts();
    long c1 = enc1_count;
    long c2 = enc2_count;
    interrupts();

    long d1 = c1 - last_enc1;
    long d2 = c2 - last_enc2;

    last_enc1 = c1;
    last_enc2 = c2;
    lastSample = now;

    float window_sec = RPM_SAMPLE_MS / 1000.0f;
    rpm1 = (d1 / PULSES_PER_REV) * (60.0f / window_sec);
    rpm2 = (d2 / PULSES_PER_REV) * (60.0f / window_sec);

    // ---- Serial print ----
    Serial.print("motor 1: ");
    Serial.print(rpm1, 1);
    Serial.print(" rpm; motor 2: ");
    Serial.print(rpm2, 1);
    Serial.println(" rpm");
  }
}

// ===== Motor drive (both motors same speed) =====
void runMotors(int speedValue) {
  if (speedValue > 0) {
    int pwm = map(speedValue, 0, 100, 0, 255);
    // Forward
    analogWrite(motor1pin1, pwm);
    digitalWrite(motor1pin2, LOW);
    analogWrite(motor2pin1, pwm);
    digitalWrite(motor2pin2, LOW);
  } else if (speedValue < 0) {
    int pwm = map(speedValue, -100, 0, 255, 0);
    // Reverse
    digitalWrite(motor1pin1, LOW);
    analogWrite(motor1pin2, pwm);
    digitalWrite(motor2pin1, LOW);
    analogWrite(motor2pin2, pwm);
  } else {
    // Stop
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  }
}

// ===== ISR for Motor 1 =====
// On each A rising edge, read B to get direction
// if B is HIGH -> -1, if LOW -> +1 (swap signs if reversed)
void enc1A_ISR() {
  bool b = digitalRead(ENC1_B_PIN);
  enc1_count += b ? -1 : +1;
}
