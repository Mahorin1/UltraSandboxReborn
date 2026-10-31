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
    /* masse totale : sert à dimensionner les poussées (le monde est en px, gravité 18) */
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
    lim(r, 9, -175, 175); /* chevilles larges : le contrôle actif gère l'orientation */
    lim(r, 10, -175, 175);
    r->mass = 0.0f;
    for (int i = 0; i < RD_PARTS; i++) r->mass += b2Body_GetMass(r->body[i]);
    r->world = w;
    r->getup_joint = (b2JointId){ 0 };
    b2BodyDef gd = b2DefaultBodyDef();
    gd.type = b2_kinematicBody;
    gd.position = px2m(pos);
    r->getup_anchor = b2CreateBody(w, &gd);
    r->ground_y = 947.5f;
    r->dir = -1;
    r->gait = RD_GAIT_STAND;
    r->stance = 0; r->swing = 1;
    r->leg1.active = r->leg2.active = true;
    ragdoll_set_dir(r, 1);
    ragdoll_set_active(r, true);
    {
        b2Vec2 start = m2px(b2Body_GetPosition(r->body[RD_CHEST]));
        b2Vec2 f1 = m2px(b2Body_GetPosition(r->body[RD_FOOT1]));
        b2Vec2 f2 = m2px(b2Body_GetPosition(r->body[RD_FOOT2]));
        r->leg1.point = (b2Vec2){ f1.x, r->ground_y };
        r->leg2.point = (b2Vec2){ f2.x, r->ground_y };
        r->com_x = start.x; r->com_y = start.y;
    }
    pid_init(&r->balancer, 0.8f, 0.1f, 0.2f, -1.0f, 1.0f);
    return r;
}

void ragdoll_destroy(Ragdoll* r) {
    if (b2Joint_IsValid(r->getup_joint)) b2DestroyJoint(r->getup_joint);
    for (int i = 0; i < RD_PARTS; i++) b2DestroyBody(r->body[i]); /* détruit aussi joints/shapes */
    if (b2Body_IsValid(r->getup_anchor)) b2DestroyBody(r->getup_anchor);
    free(r);
}

static void getup_detach(Ragdoll* r) {
    if (b2Joint_IsValid(r->getup_joint)) b2DestroyJoint(r->getup_joint);
    r->getup_joint = (b2JointId){ 0 };
}

/* ---------- états ---------- */

void ragdoll_set_active(Ragdoll* r, bool on) {
    if (!on) getup_detach(r);
    for (int i = 0; i < RD_JOINTS; i++) {
        r->art[i].active = on;
        b2RevoluteJoint_EnableMotor(r->art[i].id, on);
        b2RevoluteJoint_EnableLimit(r->art[i].id, true);
    }
    if (on) { /* bras libres : le contrôleur pilote les épaules */
        b2RevoluteJoint_EnableLimit(r->art[1].id, false);
        b2RevoluteJoint_EnableLimit(r->art[2].id, false);
    }
    if (on && !r->active) {
        r->gait = RD_GAIT_STAND;
        r->stepping = false;
        r->getup_phase = 0;
        r->getup_timer = 0.0f;
        r->fall_timer = 0.0f;
        r->lying = false;
        r->gait_phase = 0.0f;
        r->lean = 0.0f;
        r->wake_timer = 0.0f;
        b2Vec2 c = m2px(b2Body_GetPosition(r->body[RD_CHEST]));
        b2Vec2 f1 = m2px(b2Body_GetPosition(r->body[RD_FOOT1]));
        b2Vec2 f2 = m2px(b2Body_GetPosition(r->body[RD_FOOT2]));
        r->leg1.point = (b2Vec2){ clampf(f1.x, c.x - 18.0f, c.x + 18.0f), r->ground_y };
        r->leg2.point = (b2Vec2){ clampf(f2.x, c.x - 18.0f, c.x + 18.0f), r->ground_y };
    }
    r->active = on;
}

void ragdoll_set_dir(Ragdoll* r, int dir) {
    if (dir == r->dir) return;
    if (dir == 1) {
        lim(r, 1, -215, 65);  lim(r, 2, -215, 65);
        lim(r, 3, -150, 0);   lim(r, 4, -150, 0);
        lim(r, 5, -140, 60);  lim(r, 6, -140, 60);
        lim(r, 7, 0, 140);    lim(r, 8, 0, 140);
    } else {
        lim(r, 1, -65, 215);  lim(r, 2, -65, 215);
        lim(r, 3, 0, 150);    lim(r, 4, 0, 150);
        lim(r, 5, -60, 140);  lim(r, 6, -60, 140);
        lim(r, 7, -140, 0);   lim(r, 8, -140, 0);
    }
    /* les pieds restent à rotation 0 : le contrôleur d'ankle les remet à plat */
    r->dir = dir;
    r->stepping = false;
    r->gait_phase = 0.0f;
}

void ragdoll_get_up(Ragdoll* r) {
    if (!r->active) ragdoll_set_active(r, true);
    getup_detach(r);
    r->gait = RD_GAIT_GETUP;
    r->getup_timer = 0.0f;
    r->getup_phase = 0;
    r->stepping = false;
    r->fall_timer = 0.0f;
    r->lying = true;

    b2Vec2 c = m2px(b2Body_GetPosition(r->body[RD_CHEST]));
    r->getup_x0 = c.x; r->getup_y0 = c.y;
    b2Body_SetTransform(r->getup_anchor, px2m(c), b2Rot_identity);

    b2MotorJointDef jd = b2DefaultMotorJointDef();
    jd.base.bodyIdA = r->getup_anchor;
    jd.base.bodyIdB = r->body[RD_CHEST];
    jd.base.localFrameA.p = b2Vec2_zero;
    jd.base.localFrameA.q = b2Rot_identity;
    jd.base.localFrameB.p = b2Vec2_zero;
    jd.base.localFrameB.q = b2Rot_identity;
    jd.base.collideConnected = false;
    jd.linearHertz = 5.0f;
    jd.linearDampingRatio = 1.0f;
    jd.maxSpringForce = 400.0f * r->mass;
    jd.maxVelocityForce = 0.0f;
    jd.angularHertz = 3.0f;
    jd.angularDampingRatio = 1.0f;
    jd.maxSpringTorque = 12.0f * r->mass;
    jd.maxVelocityTorque = 0.0f;
    r->getup_joint = b2CreateMotorJoint(r->world, &jd);
}

static void set_j(Ragdoll* r, int i, float deg, float speed, float force) {
    r->art[i].target = deg2rad(deg);
    r->art[i].speed = speed;
    b2RevoluteJoint_SetMaxMotorTorque(r->art[i].id, force);
}

static void set_j_rad(Ragdoll* r, int i, float rad, float speed, float force) {
    r->art[i].target = rad;
    r->art[i].speed = speed;
    b2RevoluteJoint_SetMaxMotorTorque(r->art[i].id, force);
}

static b2Vec2 anchor_px(b2JointId j) {
    return m2px(b2Body_GetWorldPoint(b2Joint_GetBodyA(j), b2Joint_GetLocalFrameA(j).p));
}

static b2Vec2 ik(const IkAnim* k, b2Vec2 pos, b2Vec2 target, int dir) {
    b2Vec2 mid = intersect_two_circles(pos, target, k->dims.x, k->dims.y, dir == -1);
    float ax = atan2f(mid.y - pos.y, mid.x - pos.x) - RAD90;
    float ay = atan2f(target.y - mid.y, target.x - mid.x) - RAD90 - ax;
    return (b2Vec2){ ax, ay };
}

/* ---------- contrôleur actif ---------- */

static void update_com(Ragdoll* r) {
    float msum = 0.0f, x = 0.0f, y = 0.0f, vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < RD_PARTS; i++) {
        float m = b2Body_GetMass(r->body[i]);
        if (m <= 0.0f) continue;
        b2Vec2 p = m2px(b2Body_GetWorldCenter(r->body[i]));
        b2Vec2 v = m2px(b2Body_GetLinearVelocity(r->body[i]));
        msum += m; x += p.x * m; y += p.y * m; vx += v.x * m; vy += v.y * m;
    }
    if (msum > 0.0f) {
        r->com_x = x / msum; r->com_y = y / msum;
        r->com_vx = vx / msum; r->com_vy = vy / msum;
    }
}

static b2Vec2 hip_px(const Ragdoll* r, int leg) {
    int j = (leg == 0) ? 5 : 6; /* foot10 : 5-7-9, foot11 : 6-8-10 */
    return anchor_px(r->art[j].id);
}

/* IK jambe + cheville à plat ; la cible est bornée pour éviter la singularité */
static void drive_leg(Ragdoll* r, int leg, b2Vec2 target, float force) {
    int jh = (leg == 0) ? 5 : 6;
    int jk = (leg == 0) ? 7 : 8;
    int jf = 9 + leg;
    b2Vec2 hip = hip_px(r, leg);
    float dx = target.x - hip.x, dy = target.y - hip.y;
    float dist = sqrtf(dx * dx + dy * dy);
    const float max_reach = 78.0f, min_reach = 24.0f;
    if (dist < 1.0f) { target = (b2Vec2){ hip.x, hip.y + 1.0f }; dx = 0.0f; dy = 1.0f; dist = 1.0f; }
    if (dist > max_reach) { target.x = hip.x + dx * (max_reach / dist); target.y = hip.y + dy * (max_reach / dist); }
    else if (dist < min_reach) { target.x = hip.x + dx * (min_reach / dist); target.y = hip.y + dy * (min_reach / dist); }

    IkAnim* k = (leg == 0) ? &r->leg1 : &r->leg2;
    b2Vec2 an = ik(k, hip, target, -r->dir);
    set_j_rad(r, jh, an.x, 14.0f, force);
    set_j_rad(r, jk, an.y, 14.0f, force);

    float cur = b2RevoluteJoint_GetCurrentAngle(r->art[jf].id);
    float frot = b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_FOOT1 + leg]));
    float corr = shortest_angle_dif(0.0f, frot);
    set_j_rad(r, jf, cap_rad2(cur + corr), 10.0f, 1.5f);
}

static void drive_arms(Ragdoll* r, int gait) {
    float d = (float)r->dir;
    float amp = (gait == RD_GAIT_RUN) ? 32.0f : ((gait == RD_GAIT_WALK) ? 18.0f : 0.0f);
    float sw = sinf(r->gait_phase * PI2_F) * amp;
    float bend = (gait == RD_GAIT_RUN) ? 55.0f : ((gait == RD_GAIT_CROUCH) ? 85.0f : 32.0f);
    float base = (gait == RD_GAIT_CROUCH) ? 35.0f : 8.0f;
    set_j(r, 1, (base + sw) * d, 9.0f, 2.0f);
    set_j(r, 2, (base - sw) * d, 9.0f, 2.0f);
    set_j(r, 3, -bend * d, 9.0f, 1.5f);
    set_j(r, 4, -bend * d, 9.0f, 1.5f);
    set_j(r, 0, 0.0f, 6.0f, 1.0f);
}

static float landing_x(const Ragdoll* r, int swing, int gait) {
    float ahead = (gait == RD_GAIT_RUN) ? 20.0f : ((gait == RD_GAIT_CROUCH) ? 10.0f : 18.0f);
    float x = r->com_x + (float)r->dir * ahead;
    b2Vec2 hip = hip_px(r, swing);
    float maxstep = (gait == RD_GAIT_RUN) ? 20.0f : ((gait == RD_GAIT_CROUCH) ? 14.0f : 20.0f);
    return clampf(x, hip.x - maxstep, hip.x + maxstep);
}

static float gait_period_for(const Ragdoll* r, int gait) {
    (void)r;
    if (gait == RD_GAIT_RUN) return 0.50f;
    if (gait == RD_GAIT_CROUCH) return 0.55f;
    return 0.50f;
}

static void start_step(Ragdoll* r, int gait) {
    r->stepping = true;
    r->gait_phase = 0.0f;
    r->stance = 0;
    r->swing = 1;
    r->gait_period = (gait == RD_GAIT_RUN) ? 0.22f : ((gait == RD_GAIT_CROUCH) ? 0.42f : 0.32f);
    r->step_lift = (gait == RD_GAIT_RUN) ? 16.0f : ((gait == RD_GAIT_CROUCH) ? 6.0f : 10.0f);
    b2Vec2* sw = (r->swing == 0) ? &r->leg1.point : &r->leg2.point;
    r->gait_period = gait_period_for(r, gait);
    r->step_lift = (gait == RD_GAIT_RUN) ? 32.0f : ((gait == RD_GAIT_CROUCH) ? 14.0f : 30.0f);
    r->swing_x0 = sw->x;
    r->swing_x1 = landing_x(r, r->swing, gait);
}

static void step_update(Ragdoll* r, float dt, int gait) {
    r->gait_period = gait_period_for(r, gait);
    r->step_lift = (gait == RD_GAIT_RUN) ? 32.0f : ((gait == RD_GAIT_CROUCH) ? 14.0f : 30.0f);
    b2Vec2* sw = (r->swing == 0) ? &r->leg1.point : &r->leg2.point;
    r->gait_phase += dt / r->gait_period;

    if (r->gait_phase >= 1.0f) {
        sw->x = r->swing_x1; sw->y = r->ground_y;
        r->gait_phase -= 1.0f;
        r->stance = r->swing;
        r->swing = 1 - r->stance;
        sw = (r->swing == 0) ? &r->leg1.point : &r->leg2.point;
        r->swing_x0 = sw->x;
        r->swing_x1 = landing_x(r, r->swing, gait);
    }

    float u = clampf(r->gait_phase, 0.0f, 1.0f);
    float sm = u * u * (3.0f - 2.0f * u);
    sw->x = r->swing_x0 + (r->swing_x1 - r->swing_x0) * sm;
    sw->y = r->ground_y - sinf(PI_F * u) * r->step_lift;
}

static void set_stand_targets(Ragdoll* r, float dt, int gait) {
    (void)dt; (void)gait;
    /* les pieds restent plantés : on ne les fait pas glisser sous le torse */
    r->leg1.point.y = r->ground_y;
    r->leg2.point.y = r->ground_y;
}

static void torso_control(Ragdoll* r, float v_cmd, float pos_err, float kp, float kd, float tq_max) {
    float err = shortest_angle_dif(r->lean, r->chest_angle);
    float w = b2Body_GetAngularVelocity(r->body[RD_CHEST]);
    float tq = clampf(kp * err - kd * w, -tq_max, tq_max);
    b2Body_ApplyTorque(r->body[RD_CHEST], tq, true);

    /* stabilisateur core : vitesse désirée + maintien du CoM au-dessus du support */
    float f = clampf(0.10f * (v_cmd - r->com_vx) + 1.5f * pos_err, -20.0f, 20.0f);
    if (fabsf(v_cmd) > 1.0f || fabsf(r->com_vx) > 8.0f || fabsf(pos_err) > 0.5f)
        b2Body_ApplyForceToCenter(r->body[RD_CHEST], (b2Vec2){ f, 0.0f }, true);
}

/* relevé assisté : appui des coudes + guidage borné du torse vers la station debout */
static void update_getup(Ragdoll* r, float dt) {
    r->getup_timer += dt;
    float t = r->getup_timer;
    float upright = fabsf(cap_rad2(r->chest_angle));
    float chest_y_px = m2px(b2Body_GetPosition(r->body[RD_CHEST])).y;
    float d = (float)r->dir;

    set_j(r, 1, 55.0f * d, 10.0f, 3.0f);
    set_j(r, 2, 55.0f * d, 10.0f, 3.0f);
    set_j(r, 3, -100.0f * d, 10.0f, 2.5f);
    set_j(r, 4, -100.0f * d, 10.0f, 2.5f);
    set_j(r, 0, 0.0f, 6.0f, 1.0f);

    /* trajectoire du guidage : remonte le torse et le redresse */
    float u = clampf(t / 0.85f, 0.0f, 1.0f);
    u = u * u * (3.0f - 2.0f * u);
    float target_y = r->ground_y - 96.0f; /* torse debout */
    b2Vec2 target = { r->getup_x0, r->getup_y0 + (target_y - r->getup_y0) * u };
    if (b2Body_IsValid(r->getup_anchor)) {
        b2Body_SetTransform(r->getup_anchor, px2m(target), b2Rot_identity);
        b2Body_SetAwake(r->body[RD_CHEST], true);
    }

    /* pieds sous le bassin, prêts à recevoir le poids */
    b2Vec2 hip0 = hip_px(r, 0), hip1 = hip_px(r, 1);
    float k = 1.0f - expf(-12.0f * dt);
    r->leg1.point.x += (hip0.x + d * 12.0f - 6.0f - r->leg1.point.x) * k;
    r->leg2.point.x += (hip1.x + d * 12.0f + 6.0f - r->leg2.point.x) * k;
    r->leg1.point.y += (r->ground_y - r->leg1.point.y) * k;
    r->leg2.point.y += (r->ground_y - r->leg2.point.y) * k;
    drive_leg(r, 0, r->leg1.point, 3.5f);
    drive_leg(r, 1, r->leg2.point, 3.5f);

    if (t > 1.0f && upright < deg2rad(30.0f) && chest_y_px < (r->ground_y - 78.0f) &&
        (r->foot_ground[0] || r->foot_ground[1])) {
        getup_detach(r);
        b2Vec2 v = b2Body_GetLinearVelocity(r->body[RD_CHEST]);
        v.y = 0.0f;
        b2Body_SetLinearVelocity(r->body[RD_CHEST], v);
        b2Body_SetAngularVelocity(r->body[RD_CHEST], 0.0f);
        r->gait = RD_GAIT_STAND;
        r->lying = false;
        r->stepping = false;
        r->fall_timer = 0.0f;
        r->lean = 0.0f;
        r->getup_timer = 0.0f;
        return;
    }
    if (t > 2.0f) ragdoll_get_up(r); /* nouvelle tentative depuis la pose courante */
}

static void update_crouch_pose(Ragdoll* r) {
    float d = (float)r->dir;
    set_j(r, 5, -45.0f * d, 10.0f, 3.0f);
    set_j(r, 6, -45.0f * d, 10.0f, 3.0f);
    set_j(r, 7,  90.0f * d, 10.0f, 2.5f);
    set_j(r, 8,  90.0f * d, 10.0f, 2.5f);
    for (int leg = 0; leg < 2; leg++) {
        int jf = 9 + leg;
        float cur = b2RevoluteJoint_GetCurrentAngle(r->art[jf].id);
        float frot = b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_FOOT1 + leg]));
        float corr = shortest_angle_dif(0.0f, frot);
        set_j_rad(r, jf, cap_rad2(cur + corr), 10.0f, 1.5f);
    }
    set_j(r, 0, 0.0f, 6.0f, 1.0f);
    set_j(r, 1, 35.0f * d, 9.0f, 2.0f);
    set_j(r, 2, 35.0f * d, 9.0f, 2.0f);
    set_j(r, 3, -60.0f * d, 9.0f, 1.5f);
    set_j(r, 4, -60.0f * d, 9.0f, 1.5f);
}

static void apply_targets(Ragdoll* r) {
    for (int i = 0; i < RD_JOINTS; i++) {
        Articulation* a = &r->art[i];
        if (!a->active) continue;
        float cur = b2RevoluteJoint_GetCurrentAngle(a->id);
        float dif = a->short_mode ? shortest_angle_dif(cur, a->target) : cur - a->target;
        b2RevoluteJoint_SetTargetMotorSpeed(a->id, -a->speed * dif);
    }
}

/* ---------- boucle ---------- */

void ragdoll_update(Ragdoll* r, float dt, float gs, const RagdollInput* in) {
    float adt = dt * (gs > 0.0f ? gs : 1.0f);
    update_com(r);
    r->chest_angle = b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_CHEST]));
    float upright = fabsf(cap_rad2(r->chest_angle));

    if (in->foot_inc) r->foot_step += 10.0f * adt;
    if (in->foot_dec) r->foot_step -= 10.0f * adt;
    if (in->down) {
        if (r->active) ragdoll_set_active(r, false);
        r->wake_up = false;
        return;
    }

    if (in->up) {
        if (!r->active) ragdoll_set_active(r, true);
        if (r->active && r->gait != RD_GAIT_GETUP && upright > deg2rad(45.0f))
            ragdoll_get_up(r);
    }

    if (!r->active) return;

    /* un ragdoll actif ne doit jamais s'endormir : les moteurs ne réveillent pas les membres */
    for (int i = 0; i < RD_PARTS; i++) b2Body_SetAwake(r->body[i], true);

    /* un ragdoll fortement incliné se relève immédiatement, avant que le
       contrôleur debout ne redresse le torse sans remonter les jambes.
       On accepte aussi un torse bas sans contact des pieds (couché sur le dos). */
    float chest_y_px = m2px(b2Body_GetPosition(r->body[RD_CHEST])).y;
    bool down = r->on_ground || chest_y_px > (r->ground_y - 55.0f);
    if (r->gait != RD_GAIT_GETUP && down) {
        if (upright > deg2rad(50.0f)) ragdoll_get_up(r);
        else r->fall_timer = 0.0f;
    } else if (r->gait != RD_GAIT_GETUP) {
        r->fall_timer = 0.0f;
    }

    if (r->gait == RD_GAIT_GETUP) {
        update_getup(r, adt);
        apply_targets(r);
        return;
    }

    int dir_in = (in->right ? 1 : 0) - (in->left ? 1 : 0);
    bool moving = dir_in != 0;
    if (moving && dir_in != r->dir) ragdoll_set_dir(r, dir_in);

    int gait;
    if (in->crouch) gait = RD_GAIT_CROUCH;
    else if (moving) gait = in->run ? RD_GAIT_RUN : RD_GAIT_WALK;
    else gait = RD_GAIT_STAND;

    if (gait != r->gait) {
        if (gait == RD_GAIT_STAND) r->stepping = false;
        else if (moving && !r->stepping) start_step(r, gait);
        r->gait = gait;
    }
    if (gait != RD_GAIT_STAND && moving && !r->stepping) start_step(r, gait);

    float lean_deg = 0.0f;
    if (gait == RD_GAIT_WALK) lean_deg = 4.0f;
    else if (gait == RD_GAIT_RUN) lean_deg = 5.0f;
    else if (gait == RD_GAIT_CROUCH) lean_deg = moving ? 3.0f : 1.0f;
    r->lean = deg2rad(lean_deg * (float)r->dir);

    float target_vx = 0.0f;
    if (moving) {
        if (gait == RD_GAIT_RUN) target_vx = 22.0f * (float)r->dir;
        else if (gait == RD_GAIT_CROUCH) target_vx = 0.0f;
        else target_vx = 20.0f * (float)r->dir;
    }

    if (gait == RD_GAIT_CROUCH) {
        float support = 0.5f * (r->leg1.point.x + r->leg2.point.x);
        update_crouch_pose(r);
        torso_control(r, 0.0f, support - r->com_x, 8.0f, 1.0f, 15.0f);
        apply_targets(r);
        return;
    }

    if (gait != RD_GAIT_STAND && moving && r->stepping) step_update(r, adt, gait);
    else set_stand_targets(r, adt, gait);

    float pos_err = 0.0f;
    if (gait == RD_GAIT_STAND || (gait == RD_GAIT_CROUCH && !moving)) {
        float support = 0.5f * (r->leg1.point.x + r->leg2.point.x);
        pos_err = support - r->com_x;
    }
    torso_control(r, target_vx, pos_err, 3.2f, 0.45f, 7.0f);

    float leg_force = (gait == RD_GAIT_RUN) ? 3.0f : ((gait == RD_GAIT_CROUCH) ? 2.6f : 2.2f);
    drive_leg(r, 0, r->leg1.point, leg_force);
    drive_leg(r, 1, r->leg2.point, leg_force);
    drive_arms(r, gait);
    apply_targets(r);
}

/* ---------- capteurs de sol ---------- */

static void touch(b2ShapeId sensor, b2ShapeId visitor, int delta) {
    if (!b2Shape_IsValid(sensor)) return;
    Ragdoll* r = b2Shape_GetUserData(sensor);
    if (!r) return;
    if (b2Shape_IsValid(visitor) && b2Shape_GetFilter(visitor).groupIndex == -r->group) return;

    uint64_t sb = b2StoreBodyId(b2Shape_GetBody(sensor));
    int leg = (sb == b2StoreBodyId(r->body[RD_FOOT1])) ? 0 : ((sb == b2StoreBodyId(r->body[RD_FOOT2])) ? 1 : -1);
    if (leg >= 0) {
        r->foot_contact[leg] += delta;
        if (r->foot_contact[leg] < 0) r->foot_contact[leg] = 0;
        r->foot_ground[leg] = r->foot_contact[leg] > 0;
    }

    r->n_contact += delta;
    if (r->n_contact < 0) r->n_contact = 0;
    r->on_ground = r->n_contact > 0;
}

void ragdoll_process_sensor_events(b2WorldId w) {
    b2SensorEvents ev = b2World_GetSensorEvents(w);
    for (int i = 0; i < ev.beginCount; i++) touch(ev.beginEvents[i].sensorShapeId, ev.beginEvents[i].visitorShapeId, +1);
    for (int i = 0; i < ev.endCount; i++)   touch(ev.endEvents[i].sensorShapeId, ev.endEvents[i].visitorShapeId, -1);
}
