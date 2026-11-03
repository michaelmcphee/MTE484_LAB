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

  // wait for MATLAB to send one byte
  while (Serial.available() == 0) {}
  Serial.read();

  set_control_interval_ms(CONTROL_INTERVAL_MS);
}

void loop() {
  // all work happens in the control ISR
}

void interval_control_code(void) {

  float theta_ref = getReference();
  float theta     = readGearAngle();
  float y         = readBallPosition();
  float V         = runController(theta_ref, theta);

  setMotorVoltage(V);
  logData(theta_ref, theta, y, V);
}
