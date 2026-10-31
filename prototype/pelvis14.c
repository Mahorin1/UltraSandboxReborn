#include "pelvis14.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Longueurs réelles de la chaîne de joints (hanche 0.72 -> genou 0.60 -> cheville 0.40). */
#define P14_THIGH 0.12f
#define P14_SHIN  0.20f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static b2BodyId make_body(b2WorldId w, float x, float y, float dx, float dy, int group) {
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = (b2Pos){ x, y };
    b2BodyId body = b2CreateBody(w, &bd);
    b2ShapeDef sd = b2DefaultShapeDef();
    sd.density = 1.0f;
    sd.filter.groupIndex = -group;
    b2Polygon box = b2MakeBox(dx * 0.5f, dy * 0.5f);
    b2CreatePolygonShape(body, &sd, &box);
    return body;
}

static b2JointId make_joint(b2WorldId w, b2BodyId a, b2BodyId b, P14Vec2 anchor) {
    b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
    jd.base.bodyIdA = a;
    jd.base.bodyIdB = b;
    b2Pos p = { anchor.x, anchor.y };
    jd.base.localFrameA.p = b2InvTransformPoint(b2Body_GetTransform(a), p);
    jd.base.localFrameB.p = b2InvTransformPoint(b2Body_GetTransform(b), p);
    jd.base.localFrameA.q = b2Rot_identity;
    jd.base.localFrameB.q = b2Rot_identity;
    jd.enableMotor = true;
    jd.maxMotorTorque = 10.0f;
    return b2CreateRevoluteJoint(w, &jd);
}

P14Ragdoll* p14_create(b2WorldId world) {
    P14Ragdoll* r = calloc(1, sizeof *r);
    if (!r) return NULL;
    r->world = world;

    const P14Vec2 pos[P14_PARTS] = {
        { 0.00f, 0.72f }, { 0.00f, 0.84f }, { 0.00f, 1.04f }, { 0.00f, 1.30f },
        { -0.24f, 1.08f }, { -0.24f, 0.88f }, { 0.24f, 1.08f }, { 0.24f, 0.88f },
        { -0.12f, 0.72f }, { -0.12f, 0.50f }, { 0.12f, 0.72f }, { 0.12f, 0.50f },
        { -0.12f, 0.34f }, { 0.12f, 0.34f }
    };
    const P14Vec2 dim[P14_PARTS] = {
        { 0.30f, 0.18f }, { 0.12f, 0.20f }, { 0.34f, 0.30f }, { 0.24f, 0.24f },
        { 0.09f, 0.22f }, { 0.08f, 0.22f }, { 0.09f, 0.22f }, { 0.08f, 0.22f },
        { 0.12f, 0.30f }, { 0.10f, 0.28f }, { 0.12f, 0.30f }, { 0.10f, 0.28f },
        { 0.12f, 0.08f }, { 0.12f, 0.08f }
    };

    for (int i = 0; i < P14_PARTS; i++) {
        r->body[i] = make_body(world, pos[i].x, pos[i].y, dim[i].x, dim[i].y, 1);
        r->dim[i][0] = dim[i].x;
        r->dim[i][1] = dim[i].y;
    }

    const struct { int a, b; P14Vec2 p; } J[P14_JOINTS] = {
        { P14_PELVIS, P14_SPINE, { 0.00f, 0.80f } },
        { P14_SPINE, P14_CHEST, { 0.00f, 0.94f } },
        { P14_CHEST, P14_HEAD, { 0.00f, 1.18f } },
        { P14_CHEST, P14_BACKARM1, { -0.18f, 1.10f } },
        { P14_CHEST, P14_FRONTARM1, { 0.18f, 1.10f } },
        { P14_BACKARM1, P14_BACKARM2, { -0.24f, 0.98f } },
        { P14_FRONTARM1, P14_FRONTARM2, { 0.24f, 0.98f } },
        { P14_PELVIS, P14_BACKLEG1, { -0.12f, 0.72f } },
        { P14_PELVIS, P14_FRONTLEG1, { 0.12f, 0.72f } },
        { P14_BACKLEG1, P14_BACKLEG2, { -0.12f, 0.60f } },
        { P14_FRONTLEG1, P14_FRONTLEG2, { 0.12f, 0.60f } },
        { P14_BACKLEG2, P14_FOOT1, { -0.12f, 0.40f } },
        { P14_FRONTLEG2, P14_FOOT2, { 0.12f, 0.40f } }
    };
    for (int i = 0; i < P14_JOINTS; i++)
        r->joint[i] = make_joint(world, r->body[J[i].a], r->body[J[i].b], J[i].p);

    return r;
}

void p14_destroy(P14Ragdoll* r) {
    if (!r) return;
    for (int i = 0; i < P14_PARTS; i++)
        if (b2Body_IsValid(r->body[i])) b2DestroyBody(r->body[i]);
    free(r);
}

P14LegSolution p14_leg_ik(P14Vec2 hip, P14Vec2 foot, float thigh, float shin, int bend) {
    P14LegSolution out = { 0 };
    float dx = foot.x - hip.x, dy = foot.y - hip.y;
    float dist = sqrtf(dx * dx + dy * dy);
    float max_reach = thigh + shin;
    float min_reach = fabsf(thigh - shin) + 1e-4f;
    out.reachable = dist >= min_reach && dist <= max_reach;
    if (dist < 1e-5f) { dx = 0.0f; dy = 1.0f; dist = 1e-5f; }
    dist = clampf(dist, min_reach, max_reach - 1e-4f);

    float ck = (dist * dist - thigh * thigh - shin * shin) / (2.0f * thigh * shin);
    ck = clampf(ck, -1.0f, 1.0f);
    float knee = (bend < 0 ? -1.0f : 1.0f) * acosf(ck);
    float hip_a = atan2f(dy, dx) - atan2f(shin * sinf(knee), thigh + shin * cosf(knee));
    float knee_x = hip.x + thigh * cosf(hip_a);
    float knee_y = hip.y + thigh * sinf(hip_a);
    out.hip_angle = hip_a;
    out.knee_angle = knee;
    out.knee = (P14Vec2){ knee_x, knee_y };
    out.foot = (P14Vec2){ knee_x + shin * cosf(hip_a + knee), knee_y + shin * sinf(hip_a + knee) };
    return out;
}

static void set_legs(P14Controller* c) {
    c->leg[0] = p14_leg_ik(c->hip[0], c->foot[0], P14_THIGH, P14_SHIN, 1);
    c->leg[1] = p14_leg_ik(c->hip[1], c->foot[1], P14_THIGH, P14_SHIN, -1);
}

void p14_controller_init(P14Controller* c) {
    memset(c, 0, sizeof *c);
    c->gait = P14_STAND;
    c->getup = P14_DONE;
    c->hip[0] = (P14Vec2){ -0.12f, 0.72f };
    c->hip[1] = (P14Vec2){ 0.12f, 0.72f };
    c->foot[0] = (P14Vec2){ -0.12f, 0.42f };
    c->foot[1] = (P14Vec2){ 0.12f, 0.42f };
    set_legs(c);
}

void p14_controller_begin_getup(P14Controller* c) {
    c->getup = P14_ROLL;
    c->getup_t = 0.0f;
    c->gait = P14_GETUP;
    c->hip[0].y = c->hip[1].y = 0.40f;
    c->foot[0].y = c->foot[1].y = 0.42f;
    set_legs(c);
}

void p14_controller_set_contacts(P14Controller* c, bool hands, bool knees) {
    c->hand_contact = hands;
    c->knee_contact = knees;
}

static void update_getup(P14Controller* c, float dt) {
    c->getup_t += dt;
    if (c->getup == P14_ROLL && c->getup_t > 0.25f) c->getup = P14_PUSH;
    else if (c->getup == P14_PUSH && c->getup_t > 0.50f && c->hand_contact) c->getup = P14_KNEEL;
    else if (c->getup == P14_KNEEL && c->getup_t > 0.75f && c->knee_contact) c->getup = P14_EXTEND;
    else if (c->getup == P14_EXTEND && c->getup_t > 1.00f) {
        c->getup = P14_DONE;
        c->gait = P14_STAND;
    }

    float hip_y = 0.40f;
    if (c->getup == P14_PUSH) hip_y = 0.52f;
    else if (c->getup == P14_KNEEL) hip_y = 0.60f;
    else if (c->getup == P14_EXTEND || c->getup == P14_DONE) hip_y = 0.72f;
    c->hip[0] = (P14Vec2){ -0.12f, hip_y };
    c->hip[1] = (P14Vec2){ 0.12f, hip_y };
    c->foot[0] = (P14Vec2){ -0.12f, 0.42f };
    c->foot[1] = (P14Vec2){ 0.12f, 0.42f };
    set_legs(c);
}

void p14_controller_update(P14Controller* c, float dt, P14Gait gait, bool crouch, float lean) {
    P14Gait in = gait;
    /* Front montant seulement : maintenir l'entrée après P14_DONE ne doit pas
       relancer la relève en boucle. */
    bool rising = in == P14_GETUP && c->last_input != P14_GETUP;
    c->last_input = in;

    if (c->getup != P14_DONE) {
        update_getup(c, dt);
        c->balance = clampf(-2.0f * lean, -1.0f, 1.0f);
        return;
    }

    if (in == P14_GETUP) {
        if (rising) {
            p14_controller_begin_getup(c);
            return;
        }
        gait = P14_STAND; /* entrée maintenue une fois la relève terminée */
    }

    c->gait = crouch ? P14_CROUCH : gait;
    c->balance = clampf(-2.0f * lean, -1.0f, 1.0f);

    float hip_y = 0.72f;
    float foot_y = 0.42f;
    float stride = 0.0f, lift = 0.0f, freq = 0.0f;

    if (c->gait == P14_CROUCH) {
        hip_y = 0.55f;
    } else if (c->gait == P14_WALK) {
        /* Amplitude bornée par la longueur réelle de la jambe (0.32 m) : à
           bassin fixe, l'allonge horizontale max vaut ~0.11 m. */
        stride = 0.06f; lift = 0.03f; freq = 2.2f;
    } else if (c->gait == P14_RUN) {
        stride = 0.10f; lift = 0.05f; freq = 5.0f;
    }

    if (stride > 0.0f) {
        c->phase += dt * freq;
        float s = sinf(c->phase);
        c->foot[0].x = -0.12f + s * stride;
        c->foot[1].x = 0.12f - s * stride;
        c->foot[0].y = foot_y + (s > 0.0f ? s * lift : 0.0f);
        c->foot[1].y = foot_y + (s < 0.0f ? -s * lift : 0.0f);
    } else {
        c->phase = 0.0f;
        c->foot[0].x = -0.12f;
        c->foot[1].x = 0.12f;
        c->foot[0].y = foot_y;
        c->foot[1].y = foot_y;
    }

    c->hip[0] = (P14Vec2){ -0.12f, hip_y };
    c->hip[1] = (P14Vec2){ 0.12f, hip_y };
    set_legs(c);
}
