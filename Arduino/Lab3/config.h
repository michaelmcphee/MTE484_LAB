#ifndef CONFIG_H
#define CONFIG_H

// ---- Pins ----
const int MOT_PIN    = A0;   // motor gear angle sensor
const int BALL_PIN   = A1;   // ball position sensor

// ---- Timing ----
const int   CONTROL_INTERVAL_MS = 1;
const float T_S = 0.001f;

// ---- Gear angle: rad = M_THETA * reading + OFFSET_THETA (Lab 1c) ----
const float M_THETA      = -3.464482414633648e-04f;
const float OFFSET_THETA =  1.489727438292469f;

// ---- Ball position: meters = M_Y * reading + OFFSET_Y (Lab 2e) ----
const float M_Y      =  2.443047606934512e-05f;
const float OFFSET_Y = -0.016192042846546f;
const float FILTER_ALPHA = 0.25f;

// ---- New Stiction Values (Lab 2d) ----
const float STICTION_CW  = -0.05f;
const float STICTION_CCW =  0.40f;

// ---- Reference (ball position, meters) ----
const float Y_LOW  = 0.10f;
const float Y_HIGH = 0.25f;
const unsigned long HALF_PERIOD_MS = 15000;   // 30 s period, 50% duty

// ---- Safety and gear angle saturator ----
const float THETA_LIMIT     = 0.8f;   // motor off beyond +/- 0.8 rad
const float THETA_REF_LIMIT = 0.7f;   // saturator on theta_ref

// ---- Inner controller C1(s) = (g1i s + g0i) / (f1i s + f0i) ----
const float f1i = 1.0f;
const float f0i = 28.0f;
const float g1i = -3.462970705995644f;
const float g0i = -1.450762759110030e+02f;

// Inner D1(z) = (n1i z + n0i) / (m1i z + m0i), forward Euler
const float m1i = f1i;
const float m0i = f0i * T_S - f1i;
const float n1i = g1i;
const float n0i = g0i * T_S - g1i;

// ---- Outer controller C2(s) = (g1o s + g0o) / (f1o s + f0o) ----
// C2LD(s) = -7 (s + 0.35) / (s + 2.5)
const float f1o = 1.0f;
const float f0o = 2.5f;
const float g1o = -7.0f;
const float g0o = -2.45f;

// Outer D2(z) = (n1o z + n0o) / (m1o z + m0o), forward Euler
const float m1o = f1o;
const float m0o = f0o * T_S - f1o;
const float n1o = g1o;
const float n0o = g0o * T_S - g1o;

// ---- Logging: 5 ticks = 5 ms between lines ----
const int LOG_DECIMATION = 5;

#endif