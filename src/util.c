#include "util.h"
#include <math.h>

float cap_rad(float r) {
    r = fmodf(r, PI2_F);
    return r < 0.0f ? r + PI2_F : r;
}

float cap_rad2(float r) {
    r = fmodf(r, PI2_F);
    r = fmodf(r + PI2_F, PI2_F);
    return r > PI_F ? r - PI2_F : r;
}

float shortest_angle_dif(float a1, float a2) {
    return fmodf(cap_rad(a1 - a2) + 3.0f * PI_F, PI2_F) - PI_F;
}

b2Vec2 intersect_two_circles(b2Vec2 a, b2Vec2 b, float r1, float r2, bool first) {
    float dx = a.x - b.x, dy = a.y - b.y;
    float R = sqrtf(dx * dx + dy * dy);
    if (!(fabsf(r1 - r2) <= R && R <= r1 + r2)) {
        float ang = atan2f(b.y - a.y, b.x - a.x);
        return (b2Vec2){ a.x + cosf(ang) * r1, a.y + sinf(ang) * r1 };
    }
    float R2 = R * R, R4 = R2 * R2;
    float k  = (r1 * r1 - r2 * r2) / (2.0f * R2);
    float d  = r1 * r1 - r2 * r2;
    float c  = sqrtf(2.0f * (r1 * r1 + r2 * r2) / R2 - (d * d) / R4 - 1.0f);
    float fx = (a.x + b.x) * 0.5f + k * (b.x - a.x);
    float gx = c * (b.y - a.y) * 0.5f;
    float fy = (a.y + b.y) * 0.5f + k * (b.y - a.y);
    float gy = c * (a.x - b.x) * 0.5f;
    return first ? (b2Vec2){ fx + gx, fy + gy } : (b2Vec2){ fx - gx, fy - gy };
}

void pid_init(Pid* p, float kp, float ki, float kd, float out_min, float out_max) {
    *p = (Pid){ .kp = kp, .ki = ki, .kd = kd, .limit_i = 1.0f,
                .out_min = out_min, .out_max = out_max, .value_mode = true };
}

float pid_update(Pid* p, float cur, float target, float dt, bool angle_mode) {
    float err = angle_mode ? shortest_angle_dif(target, cur) : target - cur;
    float der = 0.0f;
    if (p->d_ready) {
        if (p->value_mode) {
            float dif = angle_mode ? shortest_angle_dif(cur, p->last_value) : cur - p->last_value;
            der = -(dif / dt);
        } else {
            float dif = angle_mode ? shortest_angle_dif(err, p->last_error) : err - p->last_error;
            der = dif / dt;
        }
    } else p->d_ready = true;
    p->last_value = cur; p->last_error = err;
    p->stored_i = clampf(p->stored_i + err * dt, -p->limit_i, p->limit_i);
    return clampf(p->kp * err + p->ki * p->stored_i + p->kd * der, p->out_min, p->out_max);
}
