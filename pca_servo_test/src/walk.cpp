// Tripod / ripple / wave gait + inverse kinematics for the 18-DOF hexapod.
// Geometry and signs match hexapod_description/view_hexapod.html and hips.json.
//
// Robot frame: +X forward, +Y left, +Z up. Every leg uses the same joint signs;
// left/right servo mirrors are applied later by calibration direction flags.
//
// IK: foot target relative to the hip -> coxa yaw, femur pitch, knee bend.
// The three printed shin segments collapse to one effective link (length + offset).

#include "walk.h"
#include <math.h>

static const float L_COXA = 0.053f;
static const float L_FEMUR = 0.07736f;
static const float L_S1 = 0.022358f, L_S2 = 0.071518f, L_S3 = 0.03114f;
// Interior CAD angles 167.2° / 159.9° -> pitch bends of -12.8° / -20.1°.
static const float BEND1 = -12.8f * DEG_TO_RAD;
static const float BEND2 = -20.1f * DEG_TO_RAD;
static const float DUTY_TRIPOD = 0.5f;
static const float DUTY_SEQUENTIAL = 5.f / 6.f;  // ripple / wave: one leg swinging
static const float STRETCH_AMP = 0.035f;
static const float STRETCH_HZ = 0.22f;
static const float STRETCH_ROT_HZ = 0.18f;

static float shinLen = 0, shinPhi = 0;
static bool shinInit = false;

// Fold the polyline shin into one vector: length and angle relative to shin1.
static void ensureShin() {
  if (shinInit) return;
  float ang = 0, x = 0, y = 0;
  const float segs[3][2] = {{L_S1, 0}, {L_S2, BEND1}, {L_S3, BEND2}};
  for (int i = 0; i < 3; i++) {
    ang += segs[i][1];
    x += segs[i][0] * cosf(ang);
    y += segs[i][0] * sinf(ang);
  }
  shinLen = sqrtf(x * x + y * y);
  shinPhi = atan2f(y, x);
  shinInit = true;
}

struct HipDef {
  float x, y;       // hip in body frame, meters
  float yaw0;       // coxa zero = mechanical mount default, rad
  int8_t splaySign; // -1 / 0 / +1 extra yaw for corner clearance
  uint8_t tripod;   // 0 or 1; the two tripods are 180° out of phase
  float ripple;     // phase offset: rear→front, alternating sides
  float wave;       // phase offset: clockwise around body (RF→RM→RR→LR→LM→LF)
};

// FL ML RL FR MR RR — matches JOINTS[] order in main.cpp
// Ripple lift order: LR, RR, LM, RM, LF, RF
// Wave lift order:  RF, RM, RR, LR, LM, LF (clockwise viewed from above)
static const HipDef HIPS[6] = {
  {0.073282f, 0.061608f, 1.161125f, -1, 0, 1.f / 6.f, 5.f / 6.f},
  {0.0f, 0.093430f, 1.570796f, 0, 1, 0.5f, 4.f / 6.f},
  {-0.073282f, 0.061608f, 1.980468f, 1, 0, 5.f / 6.f, 3.f / 6.f},
  {0.073282f, -0.061608f, -1.161125f, 1, 1, 0.f, 0.f},
  {0.0f, -0.093430f, -1.570796f, 0, 0, 2.f / 6.f, 1.f / 6.f},
  {-0.073282f, -0.061608f, -1.980468f, -1, 1, 4.f / 6.f, 2.f / 6.f},
};

static WalkParams params;
static bool enabled = false;
static uint8_t stretchMode = WALK_STRETCH_OFF;
static float phase = 0;
static float stretchPhase = 0;
static LegAngles lastAng[6];

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float wrapPi(float a) { return atan2f(sinf(a), cosf(a)); }

// Foot (dx, dy, dz) relative to hip, body axes -> model joint angles.
static LegAngles legIK(float dx, float dy, float dz, float yaw0) {
  ensureShin();
  LegAngles a;
  a.coxa = wrapPi(atan2f(dy, dx) - yaw0);
  float r = sqrtf(dx * dx + dy * dy) - L_COXA;
  float z = dz;
  float dMin = fabsf(L_FEMUR - shinLen) + 1e-4f;
  float dMax = L_FEMUR + shinLen - 1e-4f;
  float d = sqrtf(r * r + z * z);
  a.ok = true;
  if (d > dMax || d < dMin) {
    a.ok = false;
    float s = clampf(d, dMin, dMax) / d;
    r *= s;
    z *= s;
    d = sqrtf(r * r + z * z);
  }
  float cosF = (L_FEMUR * L_FEMUR + d * d - shinLen * shinLen) / (2.f * L_FEMUR * d);
  a.femur = atan2f(z, r) + acosf(clampf(cosF, -1.f, 1.f));
  float psi = atan2f(z - L_FEMUR * sinf(a.femur), r - L_FEMUR * cosf(a.femur));
  a.knee = a.femur + shinPhi - psi;
  return a;
}

void walkReset() {
  phase = 0;
  stretchPhase = 0;
}

void walkSetStretchMode(uint8_t mode) {
  if (mode > WALK_STRETCH_ROTATE) mode = WALK_STRETCH_OFF;
  stretchMode = mode;
  stretchPhase = 0;
}

uint8_t walkStretchMode() { return stretchMode; }

void walkSetParams(const WalkParams &p) { params = p; }

WalkParams walkGetParams() { return params; }

void walkSetEnabled(bool on) {
  enabled = on;
  if (!on) {
    phase = 0;
    stretchPhase = 0;
    stretchMode = WALK_STRETCH_OFF;
  }
}

bool walkEnabled() { return enabled; }

float walkPhase() {
  return stretchMode != WALK_STRETCH_OFF ? stretchPhase : phase;
}

LegAngles walkLegAngles(uint8_t leg) {
  if (leg >= 6) return lastAng[0];
  return lastAng[leg];
}

static void stretchOffset(float u, uint8_t mode, float *leanX, float *leanY) {
  *leanX = 0;
  *leanY = 0;
  if (mode == WALK_STRETCH_LINEAR) {
    float u1 = fmodf(u, 1.f);
    if (u1 < 0) u1 += 1.f;
    float half = u1 < 0.5f ? u1 * 2.f : (u1 - 0.5f) * 2.f;
    float mag = sinf(PI * half);
    float sign = u1 < 0.5f ? 1.f : -1.f;
    *leanY = sign * STRETCH_AMP * mag;
  } else if (mode == WALK_STRETCH_ROTATE) {
    float ang = u * 2.f * PI;
    *leanX = STRETCH_AMP * cosf(ang);
    *leanY = STRETCH_AMP * sinf(ang);
  }
}

void walkUpdate(float dt) {
  if (!enabled) return;
  if (dt > 0.05f) dt = 0.05f;
  if (dt < 0) dt = 0;

  if (stretchMode != WALK_STRETCH_OFF) {
    float hz = stretchMode == WALK_STRETCH_ROTATE ? STRETCH_ROT_HZ : STRETCH_HZ;
    stretchPhase = fmodf(stretchPhase + dt * hz, 1.f);
    if (stretchPhase < 0) stretchPhase += 1.f;

    float leanX, leanY;
    stretchOffset(stretchPhase, stretchMode, &leanX, &leanY);

    for (uint8_t i = 0; i < 6; i++) {
      const HipDef &h = HIPS[i];
      float yawStance = h.yaw0 + h.splaySign * params.splay;
      float nx = h.x + params.radius * cosf(yawStance);
      float ny = h.y + params.radius * sinf(yawStance);
      float fx = nx - leanX;
      float fy = ny - leanY;
      float fz = -params.height;
      lastAng[i] = legIK(fx - h.x, fy - h.y, fz, h.yaw0);
    }
    return;
  }

  phase = fmodf(phase + dt * params.freq, 1.f);
  if (phase < 0) phase += 1.f;

  const bool sequential = params.gait == WALK_GAIT_RIPPLE || params.gait == WALK_GAIT_WAVE;
  const float duty = sequential ? DUTY_SEQUENTIAL : DUTY_TRIPOD;
  // No-slip: body advances one stride during each stance interval.
  float tStance = duty / params.freq;
  float speed = params.stride / tStance;
  float vx = speed * cosf(params.crab);
  float vy = speed * sinf(params.crab);

  for (uint8_t i = 0; i < 6; i++) {
    const HipDef &h = HIPS[i];
    float yawStance = h.yaw0 + h.splaySign * params.splay;
    float nx = h.x + params.radius * cosf(yawStance);
    float ny = h.y + params.radius * sinf(yawStance);
    // Foot must cancel body motion at this footprint, including yaw.
    float sx = (vx - params.turn * ny) * tStance;
    float sy = (vy + params.turn * nx) * tStance;

    float p;
    if (params.gait == WALK_GAIT_WAVE) p = fmodf(phase + h.wave, 1.f);
    else if (params.gait == WALK_GAIT_RIPPLE) p = fmodf(phase + h.ripple, 1.f);
    else p = fmodf(phase + h.tripod * 0.5f, 1.f);
    if (p < 0) p += 1.f;
    float fx, fy, fz;
    if (p < duty) {
      float u = p / duty;
      fx = nx + sx * (0.5f - u);
      fy = ny + sy * (0.5f - u);
      fz = -params.height;
    } else {
      float u = (p - duty) / (1.f - duty);
      fx = nx + sx * (u - 0.5f);
      fy = ny + sy * (u - 0.5f);
      fz = -params.height + params.lift * sinf(PI * u);
    }
    lastAng[i] = legIK(fx - h.x, fy - h.y, fz, h.yaw0);
  }
}
