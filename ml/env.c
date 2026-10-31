#include "env.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define EPISODE_S   6.0f
#define GROUND_Y    0.0f      /* dessus du sol (y vers le bas, comme le jeu) */
#define SPAWN_CHEST (-1.63f)  /* torse au-dessus du sol */
#define PUSH_T      1.5f

struct Env {
    b2WorldId world;
    b2BodyId  ground;
    RdOld*    rd;
    float     dt, gravity;
    int       substeps;
    float     t;
    float     cmd;
    int       group;
    bool      push, pushed;
    unsigned  rng;
    float     com_x, com_y, com_vx, com_vy;
    float     prev_action[ENV_ACT];
};

static float frand(unsigned* s) {
    unsigned x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}

static void update_com(Env* e) {
    float ms = 0.0f, x = 0.0f, y = 0.0f, vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < RD_OLD_PARTS; i++) {
        float m = b2Body_GetMass(e->rd->body[i]);
        if (m <= 0.0f) continue;
        b2Vec2 c = b2Body_GetWorldCenter(e->rd->body[i]);
        b2Vec2 v = b2Body_GetLinearVelocity(e->rd->body[i]);
        ms += m; x += c.x * m; y += c.y * m; vx += v.x * m; vy += v.y * m;
    }
    if (ms > 0.0f) {
        e->com_x = x / ms; e->com_y = y / ms;
        e->com_vx = vx / ms; e->com_vy = vy / ms;
    }
}

Env* env_create(unsigned seed) {
    Env* e = calloc(1, sizeof *e);
    if (!e) return NULL;
    e->rng = seed | 1u;
    e->group = 1;
    e->dt = ENV_DT_DEFAULT;
    e->gravity = 9.81f;
    e->substeps = 8;

    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = (b2Vec2){ 0.0f, e->gravity }; /* y vers le bas */
    wd.enableSleep = false;
    e->world = b2CreateWorld(&wd);

    b2BodyDef gd = b2DefaultBodyDef();
    gd.type = b2_staticBody;
    gd.position = (b2Vec2){ 0.0f, GROUND_Y + 0.5f };
    e->ground = b2CreateBody(e->world, &gd);
    b2ShapeDef gs = b2DefaultShapeDef();
    gs.material.friction = 0.9f;
    gs.enableSensorEvents = true;
    b2Polygon gb = b2MakeBox(80.0f, 0.5f);
    b2CreatePolygonShape(e->ground, &gs, &gb);
    return e;
}

void env_destroy(Env* e) {
    if (!e) return;
    rd_old_destroy(e->rd);
    b2DestroyWorld(e->world);
    free(e);
}

void env_reset(Env* e, unsigned seed, float cmd, bool push) {
    e->rng = (seed | 1u) ^ ((unsigned)(cmd + 2.0f) * 7919u) ^ 0x9e3779b9u;
    rd_old_destroy(e->rd);
    e->group++;
    float jx = (frand(&e->rng) - 0.5f) * 0.06f;
    float jy = (frand(&e->rng) - 0.5f) * 0.03f;
    e->rd = rd_old_create(e->world, (b2Vec2){ jx, SPAWN_CHEST + jy }, 1.0f, e->group);
    e->t = 0.0f;
    e->cmd = cmd;
    e->push = push;
    e->pushed = false;
    memset(e->prev_action, 0, sizeof e->prev_action);
    b2Body_SetLinearVelocity(e->rd->body[RD_CHEST],
                             (b2Vec2){ (frand(&e->rng) - 0.5f) * 0.25f, 0.0f });
    update_com(e);
}

void env_obs(const Env* e, float* o) {
    int k = 0;
    for (int j = 0; j < RD_OLD_JOINTS; j++) o[k++] = rd_old_joint_angle(e->rd, j) * 0.5f;
    for (int j = 0; j < RD_OLD_JOINTS; j++) o[k++] = rd_old_joint_vel(e->rd, j) * 0.1f;
    float ang = b2Rot_GetAngle(b2Body_GetRotation(e->rd->body[RD_CHEST]));
    o[k++] = ang;
    o[k++] = b2Body_GetAngularVelocity(e->rd->body[RD_CHEST]) * 0.2f;
    o[k++] = e->com_vx * 0.5f;
    o[k++] = e->com_vy * 0.5f;
    o[k++] = (GROUND_Y - b2Body_GetPosition(e->rd->body[RD_CHEST]).y) / 2.5f;
    o[k++] = e->rd->contact[0] ? 1.0f : -1.0f;
    o[k++] = e->rd->contact[1] ? 1.0f : -1.0f;
    o[k++] = sinf(6.2831853f * e->t / 0.7f);
    o[k++] = cosf(6.2831853f * e->t / 0.7f);
    o[k++] = e->cmd * 2.0f;
    while (k < ENV_OBS) o[k++] = 0.0f;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float env_step(Env* e, const float* action, bool* done) {
    float act[ENV_ACT];
    float energy = 0.0f, jerk = 0.0f;
    for (int i = 0; i < ENV_ACT; i++) {
        act[i] = clampf(action[i], -1.0f, 1.0f);
        energy += act[i] * act[i];
        float d = act[i] - e->prev_action[i];
        jerk += d * d;
        e->prev_action[i] = act[i];
    }
    /* PD autour d'une pose de repos : la politique décale les cibles
       articulaires, ce qui donne une raideur passive et rend l'équilibre
       apprenable (le couple brut seul est trop instable). */
    static const float ref[ENV_ACT] = { 0.00f, 0.14f, 0.14f, -0.56f, -0.56f, 0.00f,
                                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f };
    static const float amp[ENV_ACT] = { 0.40f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f,
                                        1.00f, 1.00f, 1.00f, 0.55f, 0.55f };
    static const float kp[ENV_ACT]  = { 0.30f, 2.00f, 2.00f, 1.00f, 1.00f, 8.00f,
                                        8.00f, 6.00f, 6.00f, 4.00f, 4.00f };
    static const float kd[ENV_ACT]  = { 0.02f, 0.10f, 0.10f, 0.05f, 0.05f, 0.40f,
                                        0.40f, 0.30f, 0.30f, 0.20f, 0.20f };
    float tau[ENV_ACT];
    for (int i = 0; i < ENV_ACT; i++) {
        float tgt = ref[i] + amp[i] * tanhf(act[i]);
        tau[i] = kp[i] * (tgt - rd_old_joint_angle(e->rd, i)) - kd[i] * rd_old_joint_vel(e->rd, i);
    }
    rd_old_apply_tau(e->rd, tau);

    b2World_Step(e->world, e->dt, e->substeps);
    rd_old_process_sensors(e->rd);
    update_com(e);
    e->t += e->dt;

    if (e->push && !e->pushed && e->t > PUSH_T) {
        float dir = frand(&e->rng) < 0.5f ? -1.0f : 1.0f;
        b2Body_ApplyLinearImpulseToCenter(e->rd->body[RD_CHEST],
                                          (b2Vec2){ dir * 0.12f, 0.0f }, true);
        e->pushed = true;
    }

    float ang = b2Rot_GetAngle(b2Body_GetRotation(e->rd->body[RD_CHEST]));
    float chest_y = GROUND_Y - b2Body_GetPosition(e->rd->body[RD_CHEST]).y;
    bool fallen = chest_y < 1.00f || fabsf(ang) > 0.90f;

    float h = clampf(chest_y / 1.63f, 0.0f, 1.0f);
    /* immobile : on paie la dérive ; en marche : on paie l'écart à la vitesse visée */
    float track = (fabsf(e->cmd) < 0.5f)
                ? -1.5f * fabsf(e->com_vx)
                : 3.0f * clampf(e->cmd * e->com_vx, -2.0f, 2.0f);
    float r = track
            + 0.80f * h
            + 0.40f * cosf(ang)
            + 0.10f
            - 0.10f * fabsf(e->com_vy)
            - 0.002f * energy
            - 0.02f * jerk;
    if (fallen) r -= 10.0f;

    *done = fallen || e->t >= EPISODE_S;
    return r;
}

int env_contact(const Env* e, int leg) { return e->rd->contact[leg]; }

float env_joint_angle(const Env* e, int j) { return rd_old_joint_angle(e->rd, j); }

float env_upright(const Env* e) {
    return cosf(b2Rot_GetAngle(b2Body_GetRotation(e->rd->body[RD_CHEST])));
}

float env_chest_y(const Env* e) {
    return GROUND_Y - b2Body_GetPosition(e->rd->body[RD_CHEST]).y;
}

float env_x(const Env* e) { return e->com_x; }

float env_vx(const Env* e) { return e->com_vx; }

b2WorldId env_world(const Env* e) { return e->world; }

const RdOld* env_rag(const Env* e) { return e->rd; }

void env_push(Env* e, float dir) {
    /* impulsion proportionnelle à la gravité pour rester comparable */
    float k = 0.12f * (e->gravity / 9.81f);
    b2Body_ApplyLinearImpulseToCenter(e->rd->body[RD_CHEST],
                                      (b2Vec2){ dir * k, 0.0f }, true);
}

void env_set_physics(Env* e, float gravity, float dt, int substeps) {
    e->gravity = gravity;
    e->dt = dt > 0.0f ? dt : ENV_DT_DEFAULT;
    e->substeps = substeps > 0 ? substeps : 8;
    b2World_SetGravity(e->world, (b2Vec2){ 0.0f, e->gravity });
}

float env_dt(const Env* e) { return e->dt; }
