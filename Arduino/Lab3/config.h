#ifndef CONFIG_H
#define CONFIG_H

// ---- Pins ----
const int MOT_PIN    = A0;   // motor gear angle sensor
const int BALL_PIN   = A1;   // ball position sensor

// ---- Timing ----
const int   CONTROL_INTERVAL_MS = 1;

// ---- Gear angle: rad = M_THETA * reading + OFFSET_THETA (Lab 1c) ----
const double M_THETA      = -3.464482414633648e-04f;
const double OFFSET_THETA =  1.489727438292469f;

// ---- Ball position: meters = M_Y * reading + OFFSET_Y (Lab 2e) ----
const double M_Y      =  2.443047606934512e-05f;
const double OFFSET_Y = -0.016192042846546f;

const float FILTER_ALPHA = 0.25f;

// ---- New Stiction Values (Lab 2d) ----
const float STICTION_CW  = -0.05f;
const float STICTION_CCW =  0.40f;

const float T_S = 0.001f;           // control period (s), must match CONTROL_INTERVAL_MS

// ---- Ball anti-stiction: ramp an offset until the ball moves ----
const float AS_TRIGGER = 0.004f;    // m, start ramping when |error| is above this
const float AS_RELEASE = 0.0024f;   // m, stop ramping when |error| falls below this
const float AS_RATE    = 0.30f;     // rad/s, growth rate of the offset
const float AS_MAX     = 0.45f;     // rad, limit on the offset
const float AS_MOVE    = 0.002f;    // m, ball counts as freed after moving this far
const float AS_SLOW    = 0.0015f;   // m, ball must have moved less than this in 200 ms to start a ramp
const int   AS_COOL_TICKS = 1500;   // ticks (1.5 s) to wait after each nudge
#define AS_SLOW_TICKS 200           // ticks (200 ms) for the slow check

// ---- Reference (ball position, meters) ----
const float Y_LOW  = 0.10f;
const float Y_HIGH = 0.25f;
const unsigned long HALF_PERIOD_MS = 15000;   // 30 s period, 50% duty

// ---- Safety and gear angle saturator ----
const float THETA_LIMIT     = 0.8f;   // motor off beyond +/- 0.8 rad
const float THETA_REF_LIMIT = 0.7f;   // saturator on theta_ref

// ---- Inner controller D1(z) = (n1i z + n0i) / (m1i z + m0i) ----
// Forward Euler, T = 0.001 s. Values from MATLAB (format long).
const double n1i = -3.462970705995644;
const double n0i =  3.317894430084641;

const double m1i =  1.000000000000000;
const double m0i = -0.972000000000000;

// ---- Outer controller D2(z) = (n3o z^3 + n2o z^2 + n1o z + n0o) / (m3o z^3 + m2o z^2 + m1o z + m0o) ----
// Forward Euler, T = 0.001 s. Poles: -0.4 +/- j0.1315, -2.5, -5, -10.
// Values from MATLAB (format long).
const double n3o =  0.000000000000000;
const double n2o = -0.672367610667031;
const double n1o =  1.344343167508085;
const double n0o = -0.671975632055217;

const double m3o =  1.000000000000000;
const double m2o = -2.981700000000000;
const double m1o =  2.963501677285318;
const double m0o = -0.981801677285319;

// ---- Logging: 5 ticks = 5 ms between lines ----
const int LOG_DECIMATION = 5;

#endif