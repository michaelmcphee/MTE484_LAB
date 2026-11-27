#ifndef HELPERS_H
#define HELPERS_H

#include <Arduino.h>
#include "config.h"

static float saturate(float val, float limit) {
  if (val >  limit) return  limit;
  if (val < -limit) return -limit;
  return val;
}

// Square wave reference (m)
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

// Filtered gear angle (rad)
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

// Filtered ball position (m)
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

// Constant beam angle offset toward the setpoint, off inside the band
static float stictionOffset(float y_ref, double e) {
  float band = (y_ref < 0.175f) ? 0.004f : 0.007f;   // m
  if (fabs(e) <= band) return 0.0f;
  return (e > 0.0) ? -STICTION_BEAM : STICTION_BEAM;
}

// Outer controller D2(z) plus ball stiction offset.
// Returns the saturated gear angle reference.
// theta_ref_unsat is the value before the saturator, offset included.
static float runOuterController(float y_ref, float y, float &theta_ref_unsat) {
  static double e1 = 0.0, e2 = 0.0, e3 = 0.0;   // e[k-1], e[k-2], e[k-3]
  static double u1 = 0.0, u2 = 0.0, u3 = 0.0;   // u[k-1], u[k-2], u[k-3]

  double e = (double)y_ref - (double)y;
  double u = (n3o * e + n2o * e1 + n1o * e2 + n0o * e3
              - m2o * u1 - m1o * u2 - m0o * u3) / m3o;

  e3 = e2;  e2 = e1;  e1 = e;
  u3 = u2;  u2 = u1;  u1 = u;     // offset is not stored here

  float as = stictionOffset(y_ref, e);

  float theta_total = (float)u + as;
  theta_ref_unsat = theta_total;
  return saturate(theta_total, THETA_REF_LIMIT);
}

// Inner controller D1(z), returns motor voltage
static float runController(float theta_ref, float theta) {
  static float e_prev = 0.0f;
  static float u_prev = 0.0f;

  // Motor off outside +/- THETA_LIMIT
  if (theta > THETA_LIMIT || theta < -THETA_LIMIT) {
    return 0.0f;
  }

  float e = theta_ref - theta;
  float u = (n1i * e + n0i * e_prev - m0i * u_prev) / m1i;

  e_prev = e;
  u_prev = u;

  // Add stiction offset (not fed back into the controller)
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