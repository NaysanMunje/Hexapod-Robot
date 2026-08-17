#pragma once
#include <Arduino.h>

// Tripod gait + 3-DOF leg IK, matching hexapod_description/view_hexapod.html.
// Units: meters and radians unless a field name says otherwise.
//
// Leg index (same order as JOINTS[] in main.cpp):
//   0=FL  1=ML  2=RL  3=FR  4=MR  5=RR
// Model names LF/LM/LR/RF/RM/RR are the same legs.

struct WalkParams {
  float freq = 0.8f;       // gait cycles per second
  float stride = 0.060f;   // stance travel of each foot, meters
  float lift = 0.030f;     // swing-foot peak height, meters
  float height = 0.095f;   // body height above ground, meters
  float radius = 0.140f;   // mid-stance foot distance from hip, meters
  float splay = 22.0f * DEG_TO_RAD;  // extra yaw on corner legs
  float crab = 0;          // travel heading in body frame, rad (0 = +X)
  float turn = 0;          // yaw rate, rad/s
  bool freezeHips = true;  // hold coxa at 0°; thighs/shins still walk
};

struct LegAngles {
  float coxa;   // rad, model hip (0 = mount default, + = CCW from above)
  float femur;  // rad, 0 = thigh flat, + = knee rises
  float knee;   // rad, 0 = shin straight, + = fold under
  bool ok;      // false if the foot target was clamped to reach
};

void walkReset();
void walkSetParams(const WalkParams &p);
WalkParams walkGetParams();
void walkSetEnabled(bool on);
bool walkEnabled();
float walkPhase();          // 0..1
void walkUpdate(float dt);  // seconds; no-op unless enabled
LegAngles walkLegAngles(uint8_t leg);
