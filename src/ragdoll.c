#include "ragdoll.h"
#include <math.h>
#include <stdlib.h>

static int next_group = 1;

/* ---------- construction ---------- */

static uint32_t rand_color(void) {
    float h = (float)(rand() % 360) / 60.0f;
    int i = (int)h; float f = h - (float)i, q = 1.0f - f, t = f;
    float rgb[6][3] = { {1,t,0}, {q,1,0}, {0,1,t}, {0,q,1}, {t,0,1}, {1,0,q} };
    const float* c = rgb[i % 6];
    return ((uint32_t)(c[0] * 255) << 24) | ((uint32_t)(c[1] * 255) << 16) | ((uint32_t)(c[2] * 255) << 8) | 0xFF;
}

static b2ShapeDef part_shape_def(int group) {
    b2ShapeDef sd = b2DefaultShapeDef();
    sd.density = 1.0f;
    sd.material.friction = 5.0f;
    sd.material.restitution = 0.0f;
    sd.filter.groupIndex = -group;   /* un ragdoll ne se collisionne pas lui-même */
    sd.enableSensorEvents = true;
    return sd;
}

static b2BodyId make_body(b2WorldId w, float x, float y) {
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = px2m((b2Vec2){ x, y });
    return b2CreateBody(w, &bd);
}

static b2JointId make_joint(b2WorldId w, b2BodyId a, b2BodyId b, float x, float y, float ref_deg) {
    b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
    b2Vec2 p = px2m((b2Vec2){ x, y });
    jd.base.bodyIdA = a; jd.base.bodyIdB = b;
    jd.base.localFrameA.p = b2InvTransformPoint(b2Body_GetTransform(a), p);
    jd.base.localFrameB.p = b2InvTransformPoint(b2Body_GetTransform(b), p);
    jd.base.localFrameA.q = b2Rot_identity;
    jd.base.localFrameB.q = b2MakeRot(-deg2rad(ref_deg)); /* angle = B - A - ref */
    jd.enableMotor = true;
    jd.maxMotorTorque = 10.0f;
    return b2CreateRevoluteJoint(w, &jd);
}

static void lim(Ragdoll* r, int j, float lo_deg, float hi_deg) {
    /* Box2D v3 borne les limites à [-180, 180] (la v2 acceptait ±215) */
    b2RevoluteJoint_SetLimits(r->art[j].id, clampf(deg2rad(lo_deg), -PI_F, PI_F), clampf(deg2rad(hi_deg), -PI_F, PI_F));
}

Ragdoll* ragdoll_create(b2WorldId w, b2Vec2 pos, float s) {
    Ragdoll* r = calloc(1, sizeof *r);
    r->group = next_group++;
    r->outline = 2.0f * s;

    const float head_r = 13.25f * s;
    const b2Vec2 chest = { 11.5f * s, 43.5f * s }, barm = { 6.0f * s, 33.0f * s }, farm = { 4.8f * s, 32.0f * s };
    const b2Vec2 bleg = { 8.85f * s, 39.0f * s }, fleg = { 6.75f * s, 35.5f * s }, foot = { 15.0f * s, 4.33f * s };

    r->arm1.dims = r->arm2.dims = (b2Vec2){ barm.y, farm.y };
    r->leg1.dims = r->leg2.dims = (b2Vec2){ bleg.y, fleg.y };
    r->chest_mid_y = chest.y * 0.5f;
    r->leg_dim_y = fleg.y + bleg.y + foot.y;

    float arm_off = (chest.y - barm.y) / 2 - 2 * s;
    float leg_off = (chest.y - bleg.y) / 2 + bleg.y;
    b2Vec2 head_p = { pos.x, pos.y - chest.y / 2 - head_r - 6 * s };
    b2Vec2 barm_p = { pos.x, pos.y - arm_off };
    b2Vec2 farm_p = { pos.x, pos.y - arm_off + barm.y };
    b2Vec2 bleg_p = { pos.x, pos.y + leg_off };
    b2Vec2 fleg_p = { pos.x, pos.y + leg_off + (bleg.y - fleg.y) / 2 + fleg.y };
    b2Vec2 foot_p = { pos.x + foot.x / 2 - fleg.x / 2 - s, fleg_p.y + fleg.y / 2 + foot.y / 2 + 2.5f * s };

    b2Vec2 head_j = { pos.x, pos.y - chest.y / 2 - 2 };
    b2Vec2 barm_j = { pos.x, pos.y - chest.y / 2 + 2 * s };
    b2Vec2 farm_j = { pos.x, farm_p.y - farm.y / 2 };
    b2Vec2 bleg_j = { pos.x, bleg_p.y - bleg.y / 2 };
    b2Vec2 fleg_j = { pos.x, fleg_p.y - fleg.y / 2 };
    b2Vec2 foot_j = { pos.x, foot_p.y };

    const b2Vec2 P[RD_PARTS] = { head_p, pos, barm_p, barm_p, farm_p, farm_p, bleg_p, bleg_p, fleg_p, fleg_p, foot_p, foot_p };
    const b2Vec2 D[RD_PARTS] = { {head_r, head_r}, chest, barm, barm, farm, farm, bleg, bleg, fleg, fleg, foot, foot };

    b2ShapeDef sd = part_shape_def(r->group);
    for (int i = 0; i < RD_PARTS; i++) {
        r->dims[i][0] = D[i].x; r->dims[i][1] = D[i].y;
        r->color[i] = rand_color();
        r->body[i] = make_body(w, P[i].x, P[i].y);
        sd.material.friction = (i >= RD_FOOT1) ? 20.0f : 5.0f;
        if (i == RD_HEAD) {
            b2Circle c = { b2Vec2_zero, head_r / PPM };
            b2CreateCircleShape(r->body[i], &sd, &c);
        } else {
            b2Polygon bx = b2MakeBox(D[i].x / 2 / PPM, D[i].y / 2 / PPM);
            b2CreatePolygonShape(r->body[i], &sd, &bx);
        }
    }

    /* capteurs de sol (cercle r=30px sur chaque pied), reconnus via userData */
    b2ShapeDef ssd = b2DefaultShapeDef();
    ssd.isSensor = true; ssd.density = 0.0f; ssd.enableSensorEvents = true;
    ssd.filter.groupIndex = -r->group; ssd.userData = r;
    b2Circle sc = { b2Vec2_zero, 30.0f / PPM };
    b2CreateCircleShape(r->body[RD_FOOT1], &ssd, &sc);
    b2CreateCircleShape(r->body[RD_FOOT2], &ssd, &sc);

    const struct { int a, b; b2Vec2 p; float ref; } J[RD_JOINTS] = {
        {1,0,head_j,0}, {1,2,barm_j,0}, {1,3,barm_j,0}, {2,4,farm_j,0}, {3,5,farm_j,0},
        {1,6,bleg_j,0}, {1,7,bleg_j,0}, {6,8,fleg_j,0}, {7,9,fleg_j,0}, {8,10,foot_j,90}, {9,11,foot_j,90},
    };
    for (int i = 0; i < RD_JOINTS; i++) {
        r->art[i].id = make_joint(w, r->body[J[i].a], r->body[J[i].b], J[i].p.x, J[i].p.y, J[i].ref);
        r->art[i].speed = 10.0f; r->art[i].short_mode = true;
    }
    r->art[0].short_mode = r->art[9].short_mode = r->art[10].short_mode = false;

    lim(r, 0, -70, 70);
    r->dir = -1; ragdoll_set_dir(r, 1);
    ragdoll_set_active(r, true);
    r->foot_step = 35.0f;
    r->leg1.active = r->leg2.active = true;
    r->leg1.point = m2px(b2Body_GetPosition(r->body[RD_FOOT1]));
    r->leg2.point = m2px(b2Body_GetPosition(r->body[RD_FOOT2]));
    pid_init(&r->balancer, 0.8f, 0.1f, 0.2f, -1.0f, 1.0f);
    return r;
}

void ragdoll_destroy(Ragdoll* r) {
    for (int i = 0; i < RD_PARTS; i++) b2DestroyBody(r->body[i]); /* détruit aussi joints/shapes */
    free(r);
}

/* ---------- états ---------- */

void ragdoll_set_active(Ragdoll* r, bool on) {
    if (on == r->active && r->art[0].active == on && r->wake_timer >= 0) { /* idempotent */ }
    for (int i = 0; i < RD_JOINTS; i++) {
        r->art[i].active = on;
        b2RevoluteJoint_EnableMotor(r->art[i].id, on);
        b2RevoluteJoint_EnableLimit(r->art[i].id, true);
    }
    if (on) { /* bras arrière libres */
        b2RevoluteJoint_EnableLimit(r->art[1].id, false);
        b2RevoluteJoint_EnableLimit(r->art[2].id, false);
    }
    r->active = on;
}

void ragdoll_set_dir(Ragdoll* r, int dir) {
    if (dir == 1 && r->dir == -1) {
        lim(r, 1, -215, 65);  lim(r, 2, -215, 65);
        lim(r, 3, -150, 0);   lim(r, 4, -150, 0);
        lim(r, 5, -140, 60);  lim(r, 6, -140, 60);
        lim(r, 7, 0, 140);    lim(r, 8, 0, 140);
        lim(r, 9, -120, -50); lim(r, 10, -120, -50);
    } else if (dir == -1 && r->dir == 1) {
        lim(r, 1, -65, 215);  lim(r, 2, -65, 215);
        lim(r, 3, 0, 150);    lim(r, 4, 0, 150);
        lim(r, 5, -60, 140);  lim(r, 6, -60, 140);
        lim(r, 7, -140, 0);   lim(r, 8, -140, 0);
        lim(r, 9, 50, 120);   lim(r, 10, 50, 120);
    } else return;
    float a = (dir == 1) ? 0.0f : PI_F;
    for (int i = RD_FOOT1; i <= RD_FOOT2; i++)
        b2Body_SetTransform(r->body[i], b2Body_GetPosition(r->body[i]), b2MakeRot(a));
    r->dir = dir;
}

static void set_j(Ragdoll* r, int i, float deg, float speed, float force) {
    r->art[i].target = deg2rad(deg);
    r->art[i].speed = speed;
    b2RevoluteJoint_SetMaxMotorTorque(r->art[i].id, force);
}

static void idle(Ragdoll* r) {
    const float s = 5.0f, f = 5.0f; float d = (float)r->dir;
    set_j(r, 1, 8 * d, s, f);  set_j(r, 2, 8 * d, s, f);
    set_j(r, 3, -28 * d, s, f); set_j(r, 4, -28 * d, s, f);
    set_j(r, 0, 0, s, f);
    for (int i = 5; i <= 8; i++) set_j(r, i, 0, s, f);
    set_j(r, 9, -90 * d, 1, 1); set_j(r, 10, -90 * d, 1, 1);
}

static void rest(Ragdoll* r) { ragdoll_set_active(r, false); r->wake_up = true; r->wake_timer = 0.0f; }

static b2Vec2 anchor_px(b2JointId j) {
    return m2px(b2Body_GetWorldPoint(b2Joint_GetBodyA(j), b2Joint_GetLocalFrameA(j).p));
}

static b2Vec2 ik(const IkAnim* k, b2Vec2 pos, int dir) {
    b2Vec2 mid = intersect_two_circles(pos, k->point, k->dims.x, k->dims.y, dir == -1);
    float ax = atan2f(mid.y - pos.y, mid.x - pos.x) - RAD90;
    float ay = atan2f(k->point.y - mid.y, k->point.x - mid.x) - RAD90 - ax;
    return (b2Vec2){ ax, ay };
}

/* ---------- boucle ---------- */

void ragdoll_update(Ragdoll* r, float dt, float gs, const RagdollInput* in) {
    for (int i = 0; i < RD_JOINTS; i++) {
        Articulation* a = &r->art[i];
        if (!a->active) continue;
        float cur = b2RevoluteJoint_GetCurrentAngle(a->id);
        float dif = a->short_mode ? shortest_angle_dif(cur, a->target) : cur - a->target;
        b2RevoluteJoint_SetTargetMotorSpeed(a->id, -a->speed * dif);
    }

    if (in->left) ragdoll_set_dir(r, -1);
    else if (in->right) ragdoll_set_dir(r, 1);
    else idle(r);

    if (in->up && !r->wake_up) ragdoll_set_active(r, true);
    if (in->down) ragdoll_set_active(r, false);
    if (in->foot_inc) r->foot_step += 10 * dt;
    if (in->foot_dec) r->foot_step -= 10 * dt;

    if (r->active) {
        const float s = 10.0f, f = 4.0f;
        b2Vec2 p, an;
        if (r->arm1.active) { p = anchor_px(r->art[2].id); an = ik(&r->arm1, p, r->dir);
            set_j(r, 2, rad2deg(an.x), s, f); set_j(r, 4, rad2deg(an.y), s, f); }
        if (r->arm2.active) { p = anchor_px(r->art[1].id); an = ik(&r->arm2, p, r->dir);
            set_j(r, 1, rad2deg(an.x), s, f); set_j(r, 3, rad2deg(an.y), s, f); }
        if (r->leg1.active) { p = anchor_px(r->art[6].id); an = ik(&r->leg1, p, -r->dir);
            set_j(r, 6, rad2deg(an.x), s, f); set_j(r, 8, rad2deg(an.y), s, f); }
        if (r->leg2.active) { p = anchor_px(r->art[5].id); an = ik(&r->leg2, p, -r->dir);
            set_j(r, 5, rad2deg(an.x), s, f); set_j(r, 7, rad2deg(an.y), s, f); }

        if (r->on_ground) {
            b2BodyId chest = r->body[RD_CHEST];
            float ca = b2Rot_GetAngle(b2Body_GetRotation(chest));
            p = m2px(b2Body_GetPosition(chest));
            float a = ca + RAD90;
            p.x += cosf(a) * r->chest_mid_y; p.y += sinf(a) * r->chest_mid_y;
            a = RAD90 - ca;
            p.x += cosf(a) * r->leg_dim_y;   p.y += sinf(a) * r->leg_dim_y;

            a = r->walk_angle;
            float d = r->foot_step;
            b2Vec2 p2 = p;
            p.x  += cosf(a) * d;        p.y  += sinf(a) * d;
            p2.x += cosf(a - PI_F) * d; p2.y += sinf(a - PI_F) * d;
            r->leg1.point = p; r->leg2.point = p2;
            if (in->left || in->right) r->walk_angle += 5.0f * dt * gs * (float)r->dir;

            /* auto-équilibre */
            float input = pid_update(&r->balancer, ca, 0.0f, dt, true);
            b2Body_ApplyTorque(chest, input * 5.8f, true);
            if (fabsf(cap_rad2(ca)) > deg2rad(40.0f)) {
                if (!r->wake_up) rest(r);
            } else if (r->wake_up) {
                r->wake_timer += dt;
                if (r->wake_timer > 4.0f) r->wake_up = false;
            }
            b2Body_ApplyForceToCenter(chest, (b2Vec2){ 0.0f, r->wake_up ? -0.5f : -0.2f }, true);
        } else rest(r);
    } else if (r->wake_up) {
        if (r->on_ground) {
            r->wake_timer += dt;
            if (r->wake_timer > 3.0f) ragdoll_set_active(r, true);
        } else r->wake_timer = 0.0f;
    }
}

/* ---------- capteurs de sol ---------- */

static void touch(b2ShapeId sensor, b2ShapeId visitor, int delta) {
    if (!b2Shape_IsValid(sensor)) return;
    Ragdoll* r = b2Shape_GetUserData(sensor);
    if (!r) return;
    if (b2Shape_IsValid(visitor) && b2Shape_GetFilter(visitor).groupIndex == -r->group) return;
    r->n_contact += delta;
    if (r->n_contact < 0) r->n_contact = 0;
    r->on_ground = r->n_contact > 0;
}

void ragdoll_process_sensor_events(b2WorldId w) {
    b2SensorEvents ev = b2World_GetSensorEvents(w);
    for (int i = 0; i < ev.beginCount; i++) touch(ev.beginEvents[i].sensorShapeId, ev.beginEvents[i].visitorShapeId, +1);
    for (int i = 0; i < ev.endCount; i++)   touch(ev.endEvents[i].sensorShapeId, ev.endEvents[i].visitorShapeId, -1);
}
