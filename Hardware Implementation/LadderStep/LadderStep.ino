#include <Arduino.h>
#include "CytronMotorDriver.h"

#define ENC_A        3
#define ENC_B        2
#define CURRENT_PIN  A0
#define SENSITIVITY  0.185f
#define CPR          1428L
#define DT_MS        10UL
#define DT_S         0.01f
#define HOLD_MS      5000UL
#define PRINT_MS     200UL
#define STEP_RPM     10
#define NUM_STEPS    10
#define MAX_RPM      100

CytronMD motor(PWM_DIR, 9, 8);
inline void driveMotor(int spd) { motor.setSpeed(-spd); }

volatile long    encCount  = 0;
volatile uint8_t prevState = 0;
static const int8_t ENC_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};
void encoderISR() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t cur = (a << 1) | b;
  encCount += ENC_TABLE[(prevState << 2) | cur];
  prevState = cur;
}

float offsetV    = 0.0f;
float filteredI  = 0.0f;
void updateCurrentFilter() {
  float v = analogRead(CURRENT_PIN) * (5.0f / 16383.0f);
  filteredI += 0.10f * ((v - offsetV) / SENSITIVITY - filteredI);
}

float Kp         = 2.0f;
float Ki         = 6.0f;
float kd = 0.0005f;
float integrator = 0.0f;
float prevError = 0.0f;

float runPI(float sp, float meas, float dt) {
  float err   = sp - meas;
    float derror = (err - prevError) / dt;
  float iNext = integrator + Ki * err * dt + kd * derror;

  float out   = Kp * err + iNext;
  if      (out >  255.0f) { 
    iNext = integrator; 
    out =  255.0f; 
    }
  else if (out < -255.0f) { 
    iNext = integrator; 
    out = -255.0f; 
    }
  integrator = iNext;
  return out;
}

int   profile[NUM_STEPS * 2 + 1];
int   profileLen = 0;

void buildProfile() {
  profileLen = 0;
  for (int i = 1; i <= NUM_STEPS; i++)
    profile[profileLen++] = i * STEP_RPM;
  for (int i = NUM_STEPS - 1; i >= 0; i--)
    profile[profileLen++] = i * STEP_RPM;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(14);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  prevState = (a << 1) | b;
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);
  driveMotor(0);

  Serial.println(F("Calibrating... keep motor stationary."));
  long sum = 0;
  for (int i = 0; i < 500; i++) { 
    sum += analogRead(CURRENT_PIN); 
    delay(2); 
    }
  
  offsetV = (sum / 500.0f) * (5.0f / 16383.0f);
  Serial.print(F("OffsetV="));
  Serial.println(offsetV, 4);
  driveMotor(6);
  buildProfile();
  Serial.println(F("Send START"));
}

void loop() {
  static bool     running   = false;
  static uint32_t lastCtrl  = 0;
  static uint32_t lastPrint = 0;
  static uint32_t stepStart = 0;
  static uint32_t prevCtrl  = 0;
  static int      stepIdx   = 0;
  static float    rpmMeas   = 0.0f;
  static long     prevCnt   = 0;

  if (!running) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      s.toUpperCase();
      if (s == F("START")) {
        running    = true;
        stepIdx    = 0;
        integrator = 0.0f;
        rpmMeas    = 0.0f;
        uint32_t now = millis();
        lastCtrl   = now;
        lastPrint  = now;
        stepStart  = now;
        prevCtrl   = now;
        noInterrupts(); encCount = 0; prevCnt = 0; interrupts();
        Serial.println(F("t_ms,sp_rpm,meas_rpm,pwm,I_filt"));
      }
    }
    return;
  }

  uint32_t now = millis();

  if (now - stepStart >= HOLD_MS) {
    stepStart  = now;
    stepIdx++;
    integrator *= 0.5f;
    if (stepIdx >= profileLen) {
      driveMotor(0);
      Serial.println(F("DONE"));
      running = false;
      return;
    }
  }

  if (running == false) {driveMotor(7);}

  if (now - lastCtrl >= DT_MS) {
    float    dt  = (now - prevCtrl) * 0.001f;
    prevCtrl     = now;
    lastCtrl     = now;

    noInterrupts();
    long cnt = encCount;
    interrupts();

    long  delta = cnt - prevCnt;
    prevCnt     = cnt;
    rpmMeas     = (dt > 0.0f) ? ((float)delta / CPR) * (60.0f / dt) : 0.0f;

    float out = runPI((float)profile[stepIdx], rpmMeas, dt);
    driveMotor((int)out);
    updateCurrentFilter();
  }

  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    float sp  = (float)profile[stepIdx];
    Serial.print(now);               Serial.print(',');
    Serial.print((int)sp);           Serial.print(',');
    Serial.print(rpmMeas, 1);        Serial.print(',');
    Serial.print((int)(Kp*(sp-rpmMeas)+integrator)); Serial.print(',');
    Serial.println(filteredI, 3);
  }

} 