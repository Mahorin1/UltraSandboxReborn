#pragma once
#include <stdbool.h>
#include <box2d/box2d.h>

#define PPM 64.0f /* pixels par mètre (ancien B2SCALE) */
#define PI_F  3.14159265358979f
#define PI2_F (2.0f * PI_F)
#define RAD90 (PI_F * 0.5f)

static inline float deg2rad(float d) { return d * (PI_F / 180.0f); }
static inline float rad2deg(float r) { return r * (180.0f / PI_F); }
static inline b2Vec2 px2m(b2Vec2 v) { return (b2Vec2){ v.x / PPM, v.y / PPM }; }
static inline b2Vec2 m2px(b2Vec2 v) { return (b2Vec2){ v.x * PPM, v.y * PPM }; }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float cap_rad(float r);   /* [0, 2pi)  */
float cap_rad2(float r);  /* [-pi, pi] */
float shortest_angle_dif(float target, float current);
b2Vec2 intersect_two_circles(b2Vec2 a, b2Vec2 b, float r1, float r2, bool first);

typedef struct {
    float kp, ki, kd;
    float limit_i, out_min, out_max;
    float stored_i, last_value, last_error;
    bool value_mode, d_ready;
} Pid;

void  pid_init(Pid* p, float kp, float ki, float kd, float out_min, float out_max);
float pid_update(Pid* p, float cur, float target, float dt, bool angle_mode);
