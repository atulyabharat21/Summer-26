//Implementation of all the approved things:
//raw current data - done
//Run the motor with a pwm frequency of 10KHz using 12 bit Data TRF -done 
//PWM by user in percentage and must be converted - done
//operate the motor at fixed pwm signal in both direction - done
//implementation of kalman filter - requires tuning
//Start and Stop - done
//Implement alternative to delay - done / working in micros() 
//encoder implementation - done
//encoder data rate - done
//Position and Speed tracking - done


#include <pwm.h>
#include <string.h>
#include "FspTimer.h"

#define EKF_N 3 //number of state vars
#define EKF_M 1 //number of measurements
#include "tinyekf.h"


//Pins
#define CURRENT_SENSOR A0
#define sensitivity 0.075f //0.075
#define PWM_M 9
#define DIR_M 10
#define ENCODER_RES_PIN 12
#define PWM_BITS        12
#define CHA 2
#define CHB 3

// Timing variables
const uint32_t sampleTime_us = 200;   // 5 kHz inner tier (current + PWM + encoder, synchronized)
const uint32_t OUTER_DIV     = 50;    // 100 Hz outer tier (RPM)
const uint32_t LOG_DIV       = 250;   // 20 Hz log rate -- easy to read in Serial Monitor, adjust freely
const float    SAMPLE_HZ     = 1.0e6f / (float)sampleTime_us; // 5000.0f

//global variables
bool     isRunning = false;
float frequency = 1e4;
uint32_t duty = 2500;
PwmOut   pinA(PWM_M);
float    offSet = 0.0f;

// Control / Commanding
volatile int32_t lastCommandedPwm = 0; 

int pwmVal(float fraction) {
  return (int)(fraction * ((1 << PWM_BITS) - 1));
}

void DriveMotor(int32_t speed) {
  if (speed >  4095) speed =  4095;
  if (speed < -4095) speed = -4095;
  lastCommandedPwm = speed;

  float dutyPercent = (fabsf((float)speed) / (float)((1 << PWM_BITS) - 1)) * 100.0f; // Converts to fraction

  if (speed >= 0) {
    digitalWrite(DIR_M, LOW);
    pinA.pulse_perc(dutyPercent);
  } else {
    digitalWrite(DIR_M, HIGH);
    pinA.pulse_perc(dutyPercent);
  }
}

// Encoder Data Acquisition

#define CPR 1425 // 7*50.9*4 -> Already Quadrature out

volatile long    encCount  = 0;
volatile uint8_t prevState = 0;
float RPM = 0.0f;     
float angles = 0.0f;   // rads (TODO)
float prevAngle = 0.0f;

static const int8_t ENC_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

// void readEncoder() {
//   // Directly obtains the digital pin data and the data is stored at the end
//   uint8_t a = (R_PORT1->PIDR >> 4) & 1;
//   uint8_t b = (R_PORT1->PIDR >> 3) & 1;
//   uint8_t cur = (a << 1) | b;
//   encCount += ENC_TABLE[(prevState << 2) | cur];
//   prevState = cur;
// }

void readEncoder() {
  // Directly obtains the digital pin data and the data is stored at the end
  uint8_t a = digitalRead(CHA);
  uint8_t b = digitalRead(CHB);
  uint8_t cur = (a << 1) | b;
  encCount += ENC_TABLE[(prevState << 2) | cur];
  prevState = cur;
}


uint32_t lastOuterMicros = 0;
long     lastOuterCount  = 0;

void updateRPM(uint32_t nowMicros) {
  
  long encSnap = encCount;
  if (lastOuterMicros != 0) {
    uint32_t dt_us = nowMicros - lastOuterMicros;
    if (dt_us > 0) {
      long  dCount = encSnap - lastOuterCount;
      float rawRPM = ((float)dCount / (float)CPR) * (60.0f * 1e6f / (float)dt_us);
      RPM = (rawRPM) * 2 * PI / 60;
    }
  }

  lastOuterMicros = nowMicros;
  lastOuterCount  = encSnap;
}

// Angles is to be implemented

inline float anglesRads(volatile long encCount){

  // in rads
  float current_rotation_counts = encCount % CPR; // this resets at every 2*PI
  float angle_radians = current_rotation_counts * (2 * 3.14159265 / CPR);

  return angle_radians;
}

inline float readCurrentAmps() {
  int raw = analogRead(CURRENT_SENSOR);
  float voltage = (raw / 4095.0f) * 5.0f;
  return (voltage - offSet) / sensitivity;
}

// Kalman filter implementation - TinyEKF
class CSKF{
  private:
    float _err_measure; // Measurement Noise Variance (R)
    float _err_estimate;// Initial Estimate Error (P0)
    float _q;           // Process Noise Covariance (Q)
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;

  public:
    CSKF(float mea_e, float est_e, float q) {
      _err_measure = mea_e;  // R 
      _err_estimate = est_e; // P0
      _q = q;                // Q
      _current_estimate = 0; // Initialize at 0 Amps
      _last_estimate = 0;
    }

    float updateCSKF(float mea){
      //Predict 
      _err_estimate = _err_estimate + _q;

      //Update 
      _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
      _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
      _err_estimate =  (1.0 - _kalman_gain) * _err_estimate;
      _last_estimate = _current_estimate;

      return _current_estimate;
    }

    void setMeasurementError(float mea_e) { _err_measure = mea_e; }
    void setEstimateError(float est_e) { _err_estimate = est_e; }
    void setProcessNoise(float q) { _q = q; }
};

//Initialisation of the KF
CSKF currentFilter(0.0045, 0.0045, 0.00048); // R, P0, Q

// Sample Record
struct Sample {
  uint32_t t_us;
  float    current_A;
  float    filtered_A;
  int32_t  pwmCmd;  
  float     encCount;
  float    rpm;
};

volatile Sample  latestSample;
volatile bool    logReady = false;

// GPT hardware timer (inner tier, 5 kHz)
FspTimer sampleTimer;
volatile uint32_t innerTick = 0;

void sampleISR(timer_callback_args_t __attribute__((unused)) *args) {
  if (!isRunning) return;

  uint32_t nowMicros = micros();

  // Synchronized read
  float   current = readCurrentAmps();
  float filteredI = currentFilter.updateCSKF(current);
  int32_t pwmCmd  = lastCommandedPwm;
  float    enc     = anglesRads(encCount);

  innerTick++;

  // Outer tier: RPM update every OUTER_DIV inner ticks (100 Hz)
  if ((innerTick % OUTER_DIV) == 0) {
    updateRPM(nowMicros);
  }

  // Log tier: stash a snapshot every LOG_DIV inner ticks for loop() to print
  if ((innerTick % LOG_DIV) == 0) {
    latestSample.t_us      = nowMicros;
    latestSample.current_A = current;
    latestSample.pwmCmd    = pwmCmd;
    latestSample.encCount  = enc;
    latestSample.rpm       = RPM;
    latestSample.filtered_A = filteredI;
    logReady = true;
  }
}

bool beginSampleTimer(float rate_hz) {
  uint8_t timer_type = GPT_TIMER;
  int8_t  tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0) {
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0) return false;

  FspTimer::force_use_of_pwm_reserved_timer();

  if (!sampleTimer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate_hz, 0.0f, sampleISR)) {
    return false;
  }
  if (!sampleTimer.setup_overflow_irq()) return false;
  if (!sampleTimer.open())  return false;
  if (!sampleTimer.start()) return false;
  return true;
}

void startSystem() {
  noInterrupts();
  lastOuterMicros = 0; 
  interrupts();

  isRunning = true;
  DriveMotor(duty);
  Serial.println("t_us,current_A,pwmCmd,encCount,rpm,filtered_A");
}

void stopSystem() {
  isRunning = false;
  DriveMotor(0); 
}

// Serial Buffer : avoids choking
#define CMD_BUF_LEN 16
char    cmdBuf[CMD_BUF_LEN];
uint8_t cmdIdx = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);

  //motor operation
  pinMode(CHA, INPUT_PULLUP);
  pinMode(CHB, INPUT_PULLUP);
  uint8_t a = digitalRead(CHA), b = digitalRead(CHB);
  prevState = (a << 1) | b;
  attachInterrupt(digitalPinToInterrupt(CHA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHB), readEncoder, CHANGE);

  pinMode(CURRENT_SENSOR, INPUT);
  analogRead(CURRENT_SENSOR); 
  pinMode(DIR_M, OUTPUT);

  // Current sensor offset voltage
  float sum = 0;
  for (int i = 0; i < 500; i++){ sum += analogRead(CURRENT_SENSOR);}

  offSet = (sum / 500.0f) / 4095.0f * 5.0f;

  Serial.print("Sensor Offset = ");
  Serial.println(offSet, 4);

  pinA.begin(frequency,0.5f);

  if (!beginSampleTimer(SAMPLE_HZ)) {
    Serial.println("ERROR: could not start GPT sample timer!");
  } else {
    Serial.print("Sample timer running at ");
    Serial.print(SAMPLE_HZ);
    Serial.println(" Hz (GPT hardware timer)");
  }

  Serial.println("Send START to start, STOP to stop.");
}

void loop() {

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdIdx] = '\0';
      cmdIdx = 0;
      if (cmdBuf[0] == '\0') continue;

      for (uint8_t i = 0; cmdBuf[i]; i++) {
        if (cmdBuf[i] >= 'a' && cmdBuf[i] <= 'z') cmdBuf[i] -= 32;
      }

      if (strcmp(cmdBuf, "START") == 0) {
        if (!isRunning) startSystem();
      } else if (strcmp(cmdBuf, "STOP") == 0) {
        if (isRunning) stopSystem();
      } else {
        Serial.print("Unknown command: ");
        Serial.println(cmdBuf);
      }
    } else if (cmdIdx < CMD_BUF_LEN - 1) {
      cmdBuf[cmdIdx++] = c;
    }
  }

  if (!isRunning) return;

  // Serial Logs
  if (logReady) {
    logReady = false;
    Serial.print(latestSample.t_us);
    Serial.print(',');
    Serial.print(latestSample.current_A, 4);
    Serial.print(',');
    Serial.print(latestSample.pwmCmd);
    Serial.print(',');
    Serial.print(latestSample.encCount);
    Serial.print(',');
    Serial.print(latestSample.rpm, 2);
    Serial.print(',');
    Serial.println(latestSample.filtered_A, 4);
  }
}
