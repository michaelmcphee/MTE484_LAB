#ifndef CONFIG_H
#define CONFIG_H

// ---- Pins ----
const int MOT_PIN    = A0;   // motor gear angle sensor
const int BALL_PIN   = A1;   // ball position sensor

// ---- Timing ----
const int   CONTROL_INTERVAL_MS = 1;
const float T_S = 0.001f;
const double T_D = 0.001;    // same period, double precision (outer controller)

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

// ---- Outer controller C2(s) (Lab 3(c), pole placement with integrator) ----
// C2(s) = (g3o s^3 + g2o s^2 + g1o s + g0o) / (f3o s^3 + f2o s^2 + f1o s + f0o)
// Paste full-precision values from MATLAB (format long) if you change the poles.
// Poles: -0.4 +/- j0.1315, -2.5, -5, -10
const double f3o = 1.0;
const double f2o = 18.3;
const double f1o = 101.67728531855956;
const double f0o = 0.0;                    // integrator
const double g3o = 0.0;
const double g2o = -672.3676106670313;
const double g1o = -392.0538259779795;
const double g0o = -75.21416325716635;

// Outer D2(z) = (n3o z^3 + n2o z^2 + n1o z + n0o) / (m3o z^3 + m2o z^2 + m1o z + m0o)
// Forward Euler, s = (z-1)/T, both polynomials multiplied by T^3.
const double m3o = f3o;
const double m2o = -3.0*f3o + f2o*T_D;
const double m1o =  3.0*f3o - 2.0*f2o*T_D + f1o*T_D*T_D;
const double m0o = -f3o + f2o*T_D - f1o*T_D*T_D + f0o*T_D*T_D*T_D;

const double n3o = g3o;
const double n2o = -3.0*g3o + g2o*T_D;
const double n1o =  3.0*g3o - 2.0*g2o*T_D + g1o*T_D*T_D;
const double n0o = -g3o + g2o*T_D - g1o*T_D*T_D + g0o*T_D*T_D*T_D;

// ---- Logging: 5 ticks = 5 ms between lines ----
const int LOG_DECIMATION = 5;

#endif