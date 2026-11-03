#ifndef HELPERS_H
#define HELPERS_H

#include <Arduino.h>
#include "config.h"

static float saturate(float val, float limit) {
  if (val >  limit) return  limit;
  if (val < -limit) return -limit;
  return val;
}

// Square wave reference in rad.
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
  return refHigh ? REF_HIGH : REF_LOW;
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

// Filtered ball position in metres.
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


// One step of D(z). Returns the motor voltage.
static float runController(float theta_ref, float theta) {
  static float e_prev = 0.0f;
  static float u_prev = 0.0f;

    // Safety: 0 V outside +/- THETA_LIMIT
  if (theta > THETA_LIMIT || theta < -THETA_LIMIT) {
    return 0.0f;
  }

  float e = theta_ref - theta;
  float u = (N1 * e + N0 * e_prev - M0 * u_prev) / M1;

  e_prev = e;
  u_prev = u;

  // Stiction is added after, so it stays out of the recursion.
  float V = u;
  if (u > 0.0f) V += STICTION_CW;
  if (u < 0.0f) V -= STICTION_CCW;
  return V;
}


static void logData(float theta_ref, float theta, float y, float V) {
  static int counter = 0;
  if (++counter < LOG_DECIMATION) return;
  counter = 0;

  Serial.print(micros());  Serial.print(",");
  Serial.print(theta_ref, 4);  Serial.print(",");
  Serial.print(theta, 4);  Serial.print(",");
  Serial.print(y, 4); Serial.print(",");
  Serial.println(V, 4);
}

#endif