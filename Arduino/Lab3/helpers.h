#ifndef HELPERS_H
#define HELPERS_H

#include <Arduino.h>
#include "config.h"

static float saturate(float val, float limit) {
  if (val >  limit) return  limit;
  if (val < -limit) return -limit;
  return val;
}

// Square wave reference in meters.
static float getReference() {
  static unsigned long lastSwitch = 0;
  static bool refHigh = false;
  static bool started = false;

  unsigned long now = millis();
  if (!started) {
    lastSwitch = now;
    started = true;
  }
  if (now - lastSwitch >= HALF_PERIOD_MS) {
    refHigh = !refHigh;
    lastSwitch = now;
  }
  return refHigh ? Y_HIGH : Y_LOW;
}

// Filtered gear angle in rad.
static float readGearAngle() {
  static float filteredRaw = 0.0f;
  static bool primed = false;

  int raw = analogRead(MOT_PIN);
  if (!primed) {
    filteredRaw = raw;
    primed = true;
  } else {
    filteredRaw = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filteredRaw;
  }
  return M_THETA * filteredRaw + OFFSET_THETA;
}

// Filtered ball position in meters.
static float readBallPosition() {
  static float filteredRaw = 0.0f;
  static bool primed = false;

  int raw = analogRead(BALL_PIN);
  if (!primed) {
    filteredRaw = raw;
    primed = true;
  } else {
    filteredRaw = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * filteredRaw;
  }
  return M_Y * filteredRaw + OFFSET_Y;
}

// One step of D2(z). Returns the saturated gear angle reference.
// The unsaturated value is fed back in the recursion and logged.
static float runOuterController(float y_ref, float y, float &theta_ref_unsat) {
  static float e_prev = 0.0f;
  static float u_prev = 0.0f;

  float e = y_ref - y;
  float u = (n1o * e + n0o * e_prev - m0o * u_prev) / m1o;

  e_prev = e;
  u_prev = u;

  theta_ref_unsat = u;
  return saturate(u, THETA_REF_LIMIT);
}

// One step of D1(z). Returns the motor voltage.
static float runController(float theta_ref, float theta) {
  static float e_prev = 0.0f;
  static float u_prev = 0.0f;

  // Safety: 0 V outside +/- THETA_LIMIT
  if (theta > THETA_LIMIT || theta < -THETA_LIMIT) {
    return 0.0f;
  }

  float e = theta_ref - theta;
  float u = (n1i * e + n0i * e_prev - m0i * u_prev) / m1i;

  e_prev = e;
  u_prev = u;

  // Stiction is added after, so it stays out of the recursion.
  float V = u;
  if (u > 0.0f) V += STICTION_CW;
  if (u < 0.0f) V -= STICTION_CCW;
  return V;
}

// Columns: time (us), y_ref, y, theta_ref (unsat), theta_ref (sat), theta, V
static void logData(float y_ref, float y, float theta_ref_unsat,
                    float theta_ref_sat, float theta, float V) {
  static int counter = 0;
  if (++counter < LOG_DECIMATION) return;
  counter = 0;

  Serial.print(micros());           Serial.print(",");
  Serial.print(y_ref, 4);           Serial.print(",");
  Serial.print(y, 4);               Serial.print(",");
  Serial.print(theta_ref_unsat, 4); Serial.print(",");
  Serial.print(theta_ref_sat, 4);   Serial.print(",");
  Serial.print(theta, 4);           Serial.print(",");
  Serial.println(V, 4);
}

#endif