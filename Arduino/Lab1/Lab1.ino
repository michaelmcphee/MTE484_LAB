#include <Arduino.h>
#include "geeWhiz.h"

// Pins
int MOT_PIN = A0;   // motor gear angle sensor
int BAL_PIN = A1;   // ball position sensor (unused)

// Gear angle scaling (Lab 1c)
// rad = M_THETA * sensorReading + OFFSET_THETA
const float M_THETA      = -0.000362004f;
const float OFFSET_THETA = 1.49809278f;

// Motor stiction offsets (Lab 1d)
const float STICTION_CW  = 0.15f;
const float STICTION_CCW = 0.18f;

// Controller gain (Lab 1e)
float Kp = -20.0f;  

// Square wave reference
const float REF_LOW  = -0.1;               // rad
const float REF_HIGH =  0.1;               // rad
const unsigned long HALF_PERIOD_MS = 1000;  // 2 s period, 50% duty cycle

// Limit on theta_ref (rad)
const float THETA_REF_LIMIT = 0.7f;

// Low-pass filter on the raw gear angle reading
// y[k] = ALPHA*x[k] + (1-ALPHA)*y[k-1]
const float FILTER_ALPHA = 0.25f;
float filteredRaw = 0.0f;
bool filterPrimed = false;

// Log every 5th tick (5 ms at Ts = 1 ms)
const int LOG_DECIMATION = 5;
int logCounter = 0;

unsigned long lastSwitch = 0;
bool refHigh = false;

void setup() {
  analogReadResolution(14);
  pinMode(A5, OUTPUT);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  set_control_interval_ms(1);
  setMotorVoltage(0.0f);

  Serial.println("t_us,theta_ref,theta,V");
}

void loop() {
  // all work happens in the control ISR
}

// Filtered gear angle (rad)
float readGearAngleRadFiltered() {
  int raw = analogRead(MOT_PIN);

  if (!filterPrimed) {
    filteredRaw = raw;   // start from the first reading
    filterPrimed = true;
  } else {
    filteredRaw = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filteredRaw;
  }

  return M_THETA * filteredRaw + OFFSET_THETA;
}

// Add motor stiction offset
float applyStiction(float v) {
  if (v > 0.0f) return v + STICTION_CW;
  if (v < 0.0f) return v - STICTION_CCW;
  return 0.0f;
}

// Clamp to +/- limit
float saturate(float val, float limit) {
  if (val > limit)  return limit;
  if (val < -limit) return -limit;
  return val;
}

// Control loop, runs every 1 ms
void interval_control_code(void) {
  digitalWrite(A5, HIGH);

  // Square wave reference
  unsigned long now = millis();
  if (now - lastSwitch >= HALF_PERIOD_MS) {
    refHigh = !refHigh;
    lastSwitch = now;
  }
  float theta_ref = refHigh ? REF_HIGH : REF_LOW;

  // Saturate the reference
  float theta_ref_sat = saturate(theta_ref, THETA_REF_LIMIT);

  // Read gear angle
  float theta = readGearAngleRadFiltered();

  // Proportional control
  float e = theta_ref_sat - theta;
  float V = Kp * e;

  // Stiction compensation
  V = applyStiction(V);

  // Clamp to +/- 6 V
  if (V > 6.0f)  V = 6.0f;
  if (V < -6.0f) V = -6.0f;

  setMotorVoltage(V);

  // Log data for MATLAB
  logCounter++;
  if (logCounter >= LOG_DECIMATION) {
    logCounter = 0;
    Serial.print(micros());
    Serial.print(",");
    Serial.print(theta_ref_sat, 4);
    Serial.print(",");
    Serial.print(theta, 4);
    Serial.print(",");
    Serial.println(V, 4);
  }

  digitalWrite(A5, LOW);
}