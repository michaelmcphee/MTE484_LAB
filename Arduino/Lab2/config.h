#ifndef CONFIG_H
#define CONFIG_H

// ---- Pins ----
const int MOT_PIN    = A0;   // motor gear angle sensor
const int BALL_PIN   = A1;   // ball position sensor

// ---- Timing ----
const int   CONTROL_INTERVAL_MS = 1;
const float T_S = 0.001f;

// ---- Gear angle: rad = M_THETA * reading + OFFSET_THETA (Lab 1c) ----
const float M_THETA      = -0.000362004f;
const float OFFSET_THETA =  1.49809278f;

// ---- Gear angle: rad = M_THETA * reading + OFFSET_THETA (Lab 2e) ----
const float M_Y      =  2.443047606934512e-05f;
const float OFFSET_Y = -0.016192042846546f;
const float FILTER_ALPHA = 0.25f;

// ---- New Stiction Values (Lab 2d) ----
const float STICTION_CW  =  -0.05f;
const float STICTION_CCW =  0.4f;

// ---- Reference ----
const float REF_LOW  = -0.7f;
const float REF_HIGH =  0.7f;
const float THETA_LIMIT = 0.8f;   // safety: motor off beyond +/- 0.8 rad
const unsigned long HALF_PERIOD_MS = 1000;

// ---- Controller: D(z) = (N1 z + N0) / (M1 z + M0), forward Euler ----
const float F1 = 1.0f;
const float F0 = 28.0f;
const float G1 = -3.462970705995644f;
const float G0 = -1.450762759110030e+02f;

const float M1 = F1;
const float M0 = F0 * T_S - F1;
const float N1 = G1;
const float N0 = G0 * T_S - G1;

// ---- Logging: 5 ticks = 5 ms between lines ----
const int LOG_DECIMATION = 5;

#endif