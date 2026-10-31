#include "rd_old.h"
#include <math.h>
#include <stdlib.h>

#define PPM  64.0f
#define PI_F 3.14159265358979f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* différence d'angle la plus courte (de current vers target) */
static float short_dif(float target, float current) {
    float d = fmodf(target - current + PI_F, 2.0f * PI_F);
    if (d < 0.0f) d += 2.0f * PI_F;
    return d - PI_F;
}

static b2BodyId make_body(b2WorldId w, float x_px, float y_px) {
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = (b2Vec2){ x_px / PPM, y_px / PPM };
    return b2CreateBody(w, &bd);
}

static b2JointId make_joint(b2WorldId w, b2BodyId a, b2BodyId b, float x_px, float y_px,
                            float ref_deg) {
    b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
    b2Vec2 p = (b2Vec2){ x_px / PPM, y_px / PPM };
    jd.base.bodyIdA = a;
    jd.base.bodyIdB = b;
    jd.base.localFrameA.p = b2InvTransformPoint(b2Body_GetTransform(a), p);
    jd.base.localFrameB.p = b2InvTransformPoint(b2Body_GetTransform(b), p);
    jd.base.localFrameA.q = b2Rot_identity;
    jd.base.localFrameB.q = b2MakeRot(-ref_deg * PI_F / 180.0f); /* angle = B - A - ref */
    jd.enableMotor = true;
    jd.maxMotorTorque = 10.0f;
    return b2CreateRevoluteJoint(w, &jd);
}

static void lim(RdOld* r, int j, float lo_deg, float hi_deg) {
    b2RevoluteJoint_SetLimits(r->joint[j],
                              clampf(lo_deg * PI_F / 180.0f, -PI_F, PI_F),
                              clampf(hi_deg * PI_F / 180.0f, -PI_F, PI_F));
}

RdOld* rd_old_create(b2WorldId w, b2Vec2 pos_m, float size, int group) {
    RdOld* r = calloc(1, sizeof *r);
    if (!r) return NULL;
    r->world = w;

    const float px = pos_m.x * PPM, py = pos_m.y * PPM;
    const float head_r = 13.25f * size;
    const float chest_w = 11.5f * size, chest_h = 43.5f * size;
    const float barm_w = 6.0f * size, barm_h = 33.0f * size;
    const float farm_w = 4.8f * size, farm_h = 32.0f * size;
    const float bleg_w = 8.85f * size, bleg_h = 39.0f * size;
    const float fleg_w = 6.75f * size, fleg_h = 35.5f * size;
    const float foot_w = 15.0f * size, foot_h = 4.33f * size;

    const float arm_off = (chest_h - barm_h) / 2.0f - 2.0f * size;
    const float leg_off = (chest_h - bleg_h) / 2.0f + bleg_h;

    const b2Vec2 head_p  = { px, py - chest_h / 2.0f - head_r - 6.0f * size };
    const b2Vec2 barm_p  = { px, py - arm_off };
    const b2Vec2 farm_p  = { px, py - arm_off + barm_h };
    const b2Vec2 bleg_p  = { px, py + leg_off };
    const b2Vec2 fleg_p  = { px, py + leg_off + (bleg_h - fleg_h) / 2.0f + fleg_h };
    const b2Vec2 foot_p  = { px + foot_w / 2.0f - fleg_w / 2.0f - size,
                             fleg_p.y + fleg_h / 2.0f + foot_h / 2.0f + 2.5f * size };

    const b2Vec2 head_j  = { px, py - chest_h / 2.0f - 2.0f };
    const b2Vec2 barm_j  = { px, py - chest_h / 2.0f + 2.0f * size };
    const b2Vec2 farm_j  = { px, farm_p.y - farm_h / 2.0f };
    const b2Vec2 bleg_j  = { px, bleg_p.y - bleg_h / 2.0f };
    const b2Vec2 fleg_j  = { px, fleg_p.y - fleg_h / 2.0f };
    const b2Vec2 foot_j  = { px, foot_p.y };

    const b2Vec2 P[RD_OLD_PARTS] = { head_p, { px, py }, barm_p, barm_p, farm_p, farm_p,
                                     bleg_p, bleg_p, fleg_p, fleg_p, foot_p, foot_p };
    const b2Vec2 D[RD_OLD_PARTS] = { { head_r, head_r }, { chest_w, chest_h },
                                     { barm_w, barm_h }, { barm_w, barm_h },
                                     { farm_w, farm_h }, { farm_w, farm_h },
                                     { bleg_w, bleg_h }, { bleg_w, bleg_h },
                                     { fleg_w, fleg_h }, { fleg_w, fleg_h },
                                     { foot_w, foot_h }, { foot_w, foot_h } };

    b2ShapeDef sd = b2DefaultShapeDef();
    sd.density = 1.0f;
    sd.material.friction = 5.0f;
    sd.material.restitution = 0.0f;
    sd.filter.groupIndex = -group; /* pas d'auto-collision */
    sd.enableSensorEvents = true;

    for (int i = 0; i < RD_OLD_PARTS; i++) {
        r->dim[i][0] = D[i].x / PPM;
        r->dim[i][1] = D[i].y / PPM;
        r->body[i] = make_body(w, P[i].x, P[i].y);
        sd.material.friction = (i == RD_FOOT1 || i == RD_FOOT2) ? 20.0f : 5.0f;
        if (i == RD_HEAD) {
            b2Circle c = { b2Vec2_zero, head_r / PPM };
            b2CreateCircleShape(r->body[i], &sd, &c);
        } else {
            b2Polygon box = b2MakeBox(D[i].x / 2.0f / PPM, D[i].y / 2.0f / PPM);
            b2CreatePolygonShape(r->body[i], &sd, &box);
        }
    }

    /* capteurs de pied (cercle r=30 px), même montage que l'ancien code */
    b2ShapeDef ssd = b2DefaultShapeDef();
    ssd.isSensor = true;
    ssd.density = 0.0f;
    ssd.enableSensorEvents = true;
    ssd.filter.groupIndex = -group;
    ssd.userData = r;
    b2Circle sc = { b2Vec2_zero, 30.0f / PPM };
    b2CreateCircleShape(r->body[RD_FOOT1], &ssd, &sc);
    b2CreateCircleShape(r->body[RD_FOOT2], &ssd, &sc);

    const struct { int a, b; b2Vec2 p; float ref; } J[RD_OLD_JOINTS] = {
        { 1, 0, head_j, 0 },  { 1, 2, barm_j, 0 }, { 1, 3, barm_j, 0 },
        { 2, 4, farm_j, 0 },  { 3, 5, farm_j, 0 },
        { 1, 6, bleg_j, 0 },  { 1, 7, bleg_j, 0 },
        { 6, 8, fleg_j, 0 },  { 7, 9, fleg_j, 0 },
        { 8, 10, foot_j, 90 }, { 9, 11, foot_j, 90 },
    };
    for (int i = 0; i < RD_OLD_JOINTS; i++)
        r->joint[i] = make_joint(w, r->body[J[i].a], r->body[J[i].b], J[i].p.x, J[i].p.y, J[i].ref);

    /* Limites symétriques : contrairement au contrôleur d'origine (qui miroite
       les limites selon le sens), une politique unique doit pouvoir aller dans
       les deux sens. Amplitudes issues de l'ancien code, élargies aux deux côtés. */
    lim(r, 0, -70, 70);
    lim(r, 1, -180, 180); lim(r, 2, -180, 180);
    lim(r, 3, -150, 150); lim(r, 4, -150, 150);
    lim(r, 5, -120, 120); lim(r, 6, -120, 120);
    lim(r, 7, -140, 140); lim(r, 8, -140, 140);
    lim(r, 9, -90, 90);   lim(r, 10, -90, 90);

    return r;
}

void rd_old_destroy(RdOld* r) {
    if (!r) return;
    for (int i = 0; i < RD_OLD_PARTS; i++) b2DestroyBody(r->body[i]);
    free(r);
}

void rd_old_set_targets(RdOld* r, const float target[RD_OLD_JOINTS], float speed,
                        const float torque[RD_OLD_JOINTS]) {
    for (int i = 0; i < RD_OLD_JOINTS; i++) {
        float cur = b2RevoluteJoint_GetCurrentAngle(r->joint[i]);
        /* même convention que le code d'origine : dif = angle courant - cible,
           puis vitesse = -speed * dif (le moteur va donc vers la cible). */
        float dif = (i == 0 || i == 9 || i == 10) ? (cur - target[i]) : short_dif(cur, target[i]);
        b2RevoluteJoint_SetMaxMotorTorque(r->joint[i], torque[i]);
        b2RevoluteJoint_SetTargetMotorSpeed(r->joint[i], -speed * dif);
        b2RevoluteJoint_EnableMotor(r->joint[i], true);
    }
}

/* Couples maximaux par articulation, dimensionnés sur la masse réelle du
   pantin (~0.55 kg, 2.5 m) : les jambes portent le corps, les bras non. */
static const float TORQUE_MAX[RD_OLD_JOINTS] = {
    0.10f, 0.80f, 0.80f, 0.40f, 0.40f, 3.00f, 3.00f, 2.50f, 2.50f, 1.20f, 1.20f
};

void rd_old_apply_tau(RdOld* r, const float tau[RD_OLD_JOINTS]) {
    for (int i = 0; i < RD_OLD_JOINTS; i++) {
        float t = clampf(tau[i], -TORQUE_MAX[i], TORQUE_MAX[i]);
        b2RevoluteJoint_SetMaxMotorTorque(r->joint[i], fabsf(t));
        b2RevoluteJoint_SetTargetMotorSpeed(r->joint[i], t >= 0.0f ? 50.0f : -50.0f);
        b2RevoluteJoint_EnableMotor(r->joint[i], true);
    }
}

void rd_old_apply_torques(RdOld* r, const float action[RD_OLD_JOINTS]) {
    float tau[RD_OLD_JOINTS];
    for (int i = 0; i < RD_OLD_JOINTS; i++) tau[i] = action[i] * TORQUE_MAX[i];
    rd_old_apply_tau(r, tau);
}

float rd_old_joint_angle(const RdOld* r, int j) { return b2RevoluteJoint_GetCurrentAngle(r->joint[j]); }

float rd_old_joint_vel(const RdOld* r, int j) {
    b2BodyId a = b2Joint_GetBodyA(r->joint[j]);
    b2BodyId b = b2Joint_GetBodyB(r->joint[j]);
    return b2Body_GetAngularVelocity(b) - b2Body_GetAngularVelocity(a);
}

static void touch(RdOld* r, b2ShapeId sensor, b2ShapeId visitor, int delta) {
    if (!b2Shape_IsValid(sensor)) return;
    if (b2Shape_GetUserData(sensor) != r) return;
    if (b2Shape_IsValid(visitor)) {
        b2BodyId vb = b2Shape_GetBody(visitor);
        for (int i = 0; i < RD_OLD_PARTS; i++)
            if (b2StoreBodyId(vb) == b2StoreBodyId(r->body[i])) return; /* soi-même */
    }
    b2BodyId sb = b2Shape_GetBody(sensor);
    int leg = (b2StoreBodyId(sb) == b2StoreBodyId(r->body[RD_FOOT1])) ? 0
            : (b2StoreBodyId(sb) == b2StoreBodyId(r->body[RD_FOOT2])) ? 1 : -1;
    if (leg < 0) return;
    r->contact[leg] += delta;
    if (r->contact[leg] < 0) r->contact[leg] = 0;
    r->n_contact += delta;
    if (r->n_contact < 0) r->n_contact = 0;
}

void rd_old_process_sensors(RdOld* r) {
    b2SensorEvents ev = b2World_GetSensorEvents(r->world);
    for (int i = 0; i < ev.beginCount; i++)
        touch(r, ev.beginEvents[i].sensorShapeId, ev.beginEvents[i].visitorShapeId, +1);
    for (int i = 0; i < ev.endCount; i++)
        touch(r, ev.endEvents[i].sensorShapeId, ev.endEvents[i].visitorShapeId, -1);
}
