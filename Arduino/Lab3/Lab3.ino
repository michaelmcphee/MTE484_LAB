#include <Arduino.h>
#include "geeWhiz.h"
#include "config.h"
#include "helpers.h"

void setup() {
  analogReadResolution(14);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  setMotorVoltage(0.0f);

  // // wait for MATLAB to send one byte
  // while (Serial.available() == 0) {}
  // Serial.read();

  set_control_interval_ms(CONTROL_INTERVAL_MS);
}

void loop() {
  // all work happens in the control ISR
}

void interval_control_code(void) {

  float y_ref = getReference();          // ball position reference (m)
  float theta = readGearAngle();         // gear angle (rad)
  float y     = readBallPosition();      // ball position (m)

  // Outer loop: ball position error -> gear angle reference
  float theta_ref_unsat;
  float theta_ref_sat = runOuterController(y_ref, y, theta_ref_unsat);

  // Inner loop: gear angle reference -> motor voltage
  float V = runController(theta_ref_sat, theta);

  setMotorVoltage(V);
  logData(y_ref, y, theta_ref_unsat, theta_ref_sat, theta, V);
}