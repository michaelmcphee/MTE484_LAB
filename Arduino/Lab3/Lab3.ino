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

  // // wait for MATLAB start byte
  // while (Serial.available() == 0) {}
  // Serial.read();

  set_control_interval_ms(CONTROL_INTERVAL_MS);
}

void loop() {
  // control runs in the timer interrupt
}

void interval_control_code(void) {

  float y_ref = getReference();          // ball reference (m)
  float theta = readGearAngle();         // gear angle (rad)
  float y     = readBallPosition();      // ball position (m)

  // Outer loop: ball position to gear angle reference
  float theta_ref_unsat;
  float theta_ref_sat = runOuterController(y_ref, y, theta_ref_unsat);

  // Inner loop: gear angle to motor voltage
  float V = runController(theta_ref_sat, theta);

  setMotorVoltage(V);
  logData(y_ref, y, theta_ref_unsat, theta_ref_sat, theta, V);
}