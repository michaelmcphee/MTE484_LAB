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

// Ramp-until-free ball anti-stiction. Call exactly once per control tick.
// Returns an offset (rad) to add to theta_ref. Negative moves the ball up.
static float antiStictionOffset(float y, double e) {
  enum { IDLE, RAMP, COOL };
  static int   state = IDLE;
  static float as = 0.0f;
  static float anchor = 0.0f;
  static int   cool = 0;
  static float yold[AS_SLOW_TICKS];
  static int   yi = 0;
  static bool  primed = false;

  if (!primed) {
    for (int i = 0; i < AS_SLOW_TICKS; i++) yold[i] = y;
    primed = true;
  }
  bool slow = fabsf(y - yold[yi]) < AS_SLOW;   // yold[yi] is the sample from 200 ms ago
  yold[yi] = y;
  yi = (yi + 1) % AS_SLOW_TICKS;

  float ae = fabsf((float)e);

  switch (state) {
    case IDLE:
      as = 0.0f;
      if (ae > AS_TRIGGER && slow) {
        state = RAMP;
        anchor = y;
      }
      break;
    case RAMP:
      as += ((e > 0.0) ? -1.0f : 1.0f) * AS_RATE * T_S;
      as = saturate(as, AS_MAX);
      if (fabsf(y - anchor) > AS_MOVE || ae < AS_RELEASE) {
        as = 0.0f;
        state = COOL;
        cool = AS_COOL_TICKS;
      }
      break;
    case COOL:
      as = 0.0f;
      if (--cool <= 0) state = IDLE;
      break;
  }
  return as;
}

// One step of D2(z), third order, plus ramp anti-stiction.
// Returns the saturated gear angle reference.
// theta_ref_unsat includes the anti-stiction offset, before the saturator.
static float runOuterController(float y_ref, float y, float &theta_ref_unsat) {
  static double e1 = 0.0, e2 = 0.0, e3 = 0.0;   // e[k-1], e[k-2], e[k-3]
  static double u1 = 0.0, u2 = 0.0, u3 = 0.0;   // u[k-1], u[k-2], u[k-3]

  double e = (double)y_ref - (double)y;
  double u = (n3o * e + n2o * e1 + n1o * e2 + n0o * e3
              - m2o * u1 - m1o * u2 - m0o * u3) / m3o;

  e3 = e2;  e2 = e1;  e1 = e;
  u3 = u2;  u2 = u1;  u1 = u;     // anti-stiction stays out of the recursion

  float as = antiStictionOffset(y, e);   // called every tick

  float theta_total = (float)u + as;
  theta_ref_unsat = theta_total;
  return saturate(theta_total, THETA_REF_LIMIT);
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