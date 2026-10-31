#include "policy.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Reproduction exacte des réglages de ml/env.c : la politique n'a de sens que
   si les observations, le PD et les couples max sont identiques. */

#define POLICY_DT   (1.0f / 80.0f)
#define GROUND_Y_PX 950.0f /* dessus du sol du jeu */
#define PARAM_COUNT (POLICY_OBS * POLICY_H1 + POLICY_H1 + \
                     POLICY_H1 * POLICY_H2 + POLICY_H2 + \
                     POLICY_H2 * POLICY_ACT + POLICY_ACT)

/* pose de repos, amplitude, gains PD et couples max (cf. ml/env.c et ml/rd_old.c) */
static const float REF[RD_JOINTS] = { 0.00f, 0.14f, 0.14f, -0.56f, -0.56f, 0.00f,
                                      0.00f, 0.00f, 0.00f, 0.00f, 0.00f };
static const float AMP[RD_JOINTS] = { 0.40f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f,
                                      1.00f, 1.00f, 1.00f, 0.55f, 0.55f };
static const float KP[RD_JOINTS]  = { 0.30f, 2.00f, 2.00f, 1.00f, 1.00f, 8.00f,
                                      8.00f, 6.00f, 6.00f, 4.00f, 4.00f };
static const float KD[RD_JOINTS]  = { 0.02f, 0.10f, 0.10f, 0.05f, 0.05f, 0.40f,
                                      0.40f, 0.30f, 0.30f, 0.20f, 0.20f };
static const float TAU_MAX[RD_JOINTS] = { 0.10f, 0.80f, 0.80f, 0.40f, 0.40f, 3.00f,
                                          3.00f, 2.50f, 2.50f, 1.20f, 1.20f };

bool policy_load(Policy* p, const char* path) {
    if (!p) return false;
    memset(p, 0, sizeof *p);
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    char magic[6];
    int count = 0;
    bool ok = fread(magic, 1, 6, f) == 6 && memcmp(magic, "USRNN1", 6) == 0 &&
              fread(&count, sizeof count, 1, f) == 1 && count == PARAM_COUNT &&
              fread(p->w1, sizeof(float), sizeof p->w1 / sizeof(float), f) == sizeof p->w1 / sizeof(float) &&
              fread(p->b1, sizeof(float), POLICY_H1, f) == (size_t)POLICY_H1 &&
              fread(p->w2, sizeof(float), sizeof p->w2 / sizeof(float), f) == sizeof p->w2 / sizeof(float) &&
              fread(p->b2, sizeof(float), POLICY_H2, f) == (size_t)POLICY_H2 &&
              fread(p->w3, sizeof(float), sizeof p->w3 / sizeof(float), f) == sizeof p->w3 / sizeof(float) &&
              fread(p->b3, sizeof(float), POLICY_ACT, f) == (size_t)POLICY_ACT;
    fclose(f);
    p->loaded = ok;
    return ok;
}

void policy_forward(const Policy* p, const float obs[POLICY_OBS], float act[POLICY_ACT]) {
    float a[POLICY_H1], b[POLICY_H2];
    for (int i = 0; i < POLICY_H1; i++) {
        float s = p->b1[i];
        for (int j = 0; j < POLICY_OBS; j++) s += p->w1[i * POLICY_OBS + j] * obs[j];
        a[i] = tanhf(s);
    }
    for (int i = 0; i < POLICY_H2; i++) {
        float s = p->b2[i];
        for (int j = 0; j < POLICY_H1; j++) s += p->w2[i * POLICY_H1 + j] * a[j];
        b[i] = tanhf(s);
    }
    for (int i = 0; i < POLICY_ACT; i++) {
        float s = p->b3[i];
        for (int j = 0; j < POLICY_H2; j++) s += p->w3[i * POLICY_H2 + j] * b[j];
        act[i] = tanhf(s);
    }
}

static float joint_angle(const Ragdoll* r, int i) {
    return b2RevoluteJoint_GetCurrentAngle(r->art[i].id);
}

static float joint_vel(const Ragdoll* r, int i) {
    b2BodyId a = b2Joint_GetBodyA(r->art[i].id);
    b2BodyId b = b2Joint_GetBodyB(r->art[i].id);
    return b2Body_GetAngularVelocity(b) - b2Body_GetAngularVelocity(a);
}

/* mêmes 32 entrées que ml/env.c, dans le même ordre */
static void build_obs(const Ragdoll* r, float cmd, float* o) {
    int k = 0;
    for (int i = 0; i < RD_JOINTS; i++) o[k++] = joint_angle(r, i) * 0.5f;
    for (int i = 0; i < RD_JOINTS; i++) o[k++] = joint_vel(r, i) * 0.1f;

    o[k++] = b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_CHEST]));
    o[k++] = b2Body_GetAngularVelocity(r->body[RD_CHEST]) * 0.2f;
    o[k++] = (r->com_vx / PPM) * 0.5f; /* com stocké en px/s côté jeu */
    o[k++] = (r->com_vy / PPM) * 0.5f;

    float chest_px = m2px(b2Body_GetPosition(r->body[RD_CHEST])).y;
    o[k++] = ((GROUND_Y_PX - chest_px) / PPM) / 2.5f;

    o[k++] = r->foot_ground[0] ? 1.0f : -1.0f;
    o[k++] = r->foot_ground[1] ? 1.0f : -1.0f;
    o[k++] = sinf(6.2831853f * r->policy_t / 0.7f);
    o[k++] = cosf(6.2831853f * r->policy_t / 0.7f);
    o[k++] = cmd * 2.0f;
    while (k < POLICY_OBS) o[k++] = 0.0f;
}

static void apply_tau(Ragdoll* r, const float tau[RD_JOINTS]) {
    for (int i = 0; i < RD_JOINTS; i++) {
        float t = clampf(tau[i], -TAU_MAX[i], TAU_MAX[i]);
        b2RevoluteJoint_SetMaxMotorTorque(r->art[i].id, fabsf(t));
        b2RevoluteJoint_SetTargetMotorSpeed(r->art[i].id, t >= 0.0f ? 50.0f : -50.0f);
        b2RevoluteJoint_EnableMotor(r->art[i].id, true);
    }
}

void ragdoll_policy_update(Ragdoll* r, const Policy* p, float dt, float cmd) {
    if (!r || !p || !p->loaded || !r->policy_on) return;

    if (cmd != r->policy_cmd) { /* nouvelle commande = nouvelle tentative */
        r->policy_cmd = cmd;
        r->policy_t = 0.0f;
        r->policy_acc = 0.0f;
    }
    r->policy_acc += dt > 0.0f ? dt : 0.0f;

    int guard = 0;
    while (r->policy_acc >= POLICY_DT && guard++ < 8) {
        r->policy_acc -= POLICY_DT;

        float obs[POLICY_OBS], act[POLICY_ACT], tau[RD_JOINTS];
        ragdoll_update_com(r);
        r->chest_angle = b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_CHEST]));
        build_obs(r, cmd, obs);
        policy_forward(p, obs, act);

        for (int i = 0; i < RD_JOINTS; i++) {
            float tgt = REF[i] + AMP[i] * tanhf(act[i]);
            tau[i] = KP[i] * (tgt - joint_angle(r, i)) - KD[i] * joint_vel(r, i);
        }
        apply_tau(r, tau);
        r->policy_t += POLICY_DT;
    }

    /* les moteurs ne réveillent pas les corps : on les garde actifs */
    for (int i = 0; i < RD_PARTS; i++) b2Body_SetAwake(r->body[i], true);
}
