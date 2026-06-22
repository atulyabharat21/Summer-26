#include <Arduino.h>
#include "CytronMotorDriver.h"

#define CURRENT_PIN  A0
#define SENSITIVITY  0.185f
#define CPR          1428L
#define PRINT_MS     200UL

CytronMD motor(PWM_DIR, 9, 8);
inline void driveMotor(int spd) { motor.setSpeed(-spd); }

volatile long    counts    = 0;
volatile uint8_t prevState = 0;
static const int8_t ENC_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};
void doEncoder() {
  uint8_t a = digitalRead(3);
  uint8_t b = digitalRead(2);
  uint8_t cur = (a << 1) | b;
  counts += ENC_TABLE[(prevState << 2) | cur];
  prevState = cur;
}

float offsetV       = 0.0f;
float filteredI     = 0.0f;
void updateCurrent() {
  float v = analogRead(CURRENT_PIN) * (5.0f / 16383.0f);
  filteredI += 0.10f * ((v - offsetV) / SENSITIVITY - filteredI);
}

float kp            = 6.0f;
float kd            = 0.5f;
float kff = 15.0f;
float amplitudeRads = 3.0f * PI;
float periodSecs    = 20.0f;
float lastError     = 0.0f;
float prevAngle     = 0.0f;
bool  isRunning     = false;
unsigned long startTime = 0;
unsigned long lastTime  = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(14);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  uint8_t a = digitalRead(3);
  uint8_t b = digitalRead(2);
  prevState = (a << 1) | b;
  attachInterrupt(digitalPinToInterrupt(3), doEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), doEncoder, CHANGE);

  Serial.println(F("Calibrating..."));
  long sum = 0;
  for (int i = 0; i < 500; i++) { sum += analogRead(CURRENT_PIN); delay(2); }
  offsetV = (sum / 500.0f) * (5.0f / 16383.0f);
  Serial.print(F("OffsetV="));
  Serial.println(offsetV, 4);
  driveMotor(-7);
  Serial.println(F("Send START"));
}

void loop() {
  static uint32_t lastPrint = 0;
  unsigned long now = millis();

  if (!isRunning) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim(); s.toUpperCase();
      if (s == F("START")) {
        isRunning  = true;
        startTime  = now;
        lastTime   = now;
        lastPrint  = now;
        lastError  = 0.0f;
        prevAngle  = 0.0f;
        noInterrupts(); counts = 0; interrupts();
        driveMotor(0);
        Serial.println(F("t_ms,target_rad,actual_rad,targetW,rpm,pwm,I_filt"));
      }
    }
    return;
  }

  if (now - lastTime >= 3UL) {
    float dt = (now - lastTime) * 0.001f;
    lastTime = now;

    noInterrupts();
    long cnt = counts;
    interrupts();

    updateCurrent();

    float actualRad = (cnt / (float)CPR) * TWO_PI;
    float t         = (now - startTime) * 0.001f;
    float targetRad = amplitudeRads * sinf(TWO_PI / periodSecs * t);
    float targetW = amplitudeRads * ( TWO_PI /  periodSecs) * cosf(TWO_PI / periodSecs * t);
    float error     = targetRad - actualRad;
    float deriv     = (error - lastError) / dt;
    lastError       = error;

    // float vOut = kp * error + kd * deriv;
    float vOut = kp * error + kd * deriv;
    float rpm  = (actualRad - prevAngle) / dt * (60.0f / TWO_PI);
    prevAngle  = actualRad;

    int pwm = (int)constrain(vOut / 12.0f * 255.0f, -255.0f, 255.0f);
    driveMotor(pwm);

    if (now - lastPrint >= PRINT_MS) {
      lastPrint = now;
      Serial.print(now - startTime); Serial.print(',');
      Serial.print(targetRad, 4);    Serial.print(',');
      Serial.print(actualRad, 4);    Serial.print(',');
      Serial.print(targetW,4);      Serial.print(',');  
      Serial.print(rpm, 2);          Serial.print(',');
      Serial.print(pwm);             Serial.print(',');
      Serial.println(filteredI, 4);
    }

    if (t >= periodSecs * 2.0f) {
      driveMotor(0);
      Serial.println(F("DONE"));
      isRunning = false;
    }
  }
}