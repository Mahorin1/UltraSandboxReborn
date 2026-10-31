#include "sim_pelvis14.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void motor_to(b2JointId j, float target, float force) {
    if (!b2Joint_IsValid(j)) return;
    float cur = b2RevoluteJoint_GetCurrentAngle(j);
    b2RevoluteJoint_EnableMotor(j, true);
    b2RevoluteJoint_SetMaxMotorTorque(j, force);
    b2RevoluteJoint_SetTargetMotorSpeed(j, clampf(12.0f * (target - cur), -20.0f, 20.0f));
}

P14Sim* p14_sim_create(void) {
    P14Sim* s = calloc(1, sizeof *s);
    if (!s) return NULL;
    b2WorldDef wd = b2DefaultWorldDef();
    /* Le prototype est authoré y vers le haut (tête 1.30 > bassin 0.72 > pieds 0.34),
       contrairement au jeu (y vers le bas) : la gravité doit être négative. */
    wd.gravity = (b2Vec2){ 0.0f, -9.8f };
    s->world = b2CreateWorld(&wd);
    s->ragdoll = p14_create(s->world);
    if (!s->ragdoll) { b2DestroyWorld(s->world); free(s); return NULL; }

    b2BodyDef gd = b2DefaultBodyDef();
    gd.type = b2_staticBody;
    gd.position = (b2Pos){ 0.0f, 0.30f };
    s->ground = b2CreateBody(s->world, &gd);
    b2ShapeDef gsd = b2DefaultShapeDef();
    gsd.material.friction = 1.0f;
    /* Box2D 3.2 ignore une paire capteur si l'une des deux formes n'active pas
       les événements de capteur : le sol doit donc les activer aussi. */
    gsd.enableSensorEvents = true;
    b2Polygon gb = b2MakeBox(2.0f, 0.02f);
    b2CreatePolygonShape(s->ground, &gsd, &gb);

    const int sensor_body[6] = { P14_FOOT1, P14_FOOT2, P14_BACKARM2, P14_FRONTARM2, P14_BACKLEG2, P14_FRONTLEG2 };
    b2ShapeDef ssd = b2DefaultShapeDef();
    ssd.isSensor = true; ssd.density = 0.0f; ssd.enableSensorEvents = true;
    ssd.filter.groupIndex = -1;
    b2Circle sc = { b2Vec2_zero, 0.08f };
    for (int i = 0; i < 6; i++)
        s->sensor[i] = b2CreateCircleShape(s->ragdoll->body[sensor_body[i]], &ssd, &sc);

    /* Racine assistée : le pelvis est piloté, les 13 autres corps restent dynamiques. */
    b2Body_SetType(s->ragdoll->body[P14_PELVIS], b2_kinematicBody);
    s->py = b2Body_GetPosition(s->ragdoll->body[P14_PELVIS]).y;
    return s;
}

void p14_sim_destroy(P14Sim* s) {
    if (!s) return;
    p14_destroy(s->ragdoll);
    b2DestroyWorld(s->world);
    free(s);
}

float p14_sim_pelvis_y(const P14Sim* s) {
    return b2Body_GetPosition(s->ragdoll->body[P14_PELVIS]).y;
}

float p14_sim_angle(const P14Sim* s) {
    return b2Rot_GetAngle(b2Body_GetTransform(s->ragdoll->body[P14_PELVIS]).q);
}

bool p14_sim_foot_contact(const P14Sim* s) {
    return s->contact[0] > 0 || s->contact[1] > 0;
}

static void process_sensors(P14Sim* s) {
    b2SensorEvents ev = b2World_GetSensorEvents(s->world);
    const int sensor_body[6] = { P14_FOOT1, P14_FOOT2, P14_BACKARM2, P14_FRONTARM2, P14_BACKLEG2, P14_FRONTLEG2 };
    for (int k = 0; k < ev.beginCount; k++) {
        b2BodyId b = b2Shape_GetBody(ev.beginEvents[k].sensorShapeId);
        for (int i = 0; i < 6; i++)
            if (b2StoreBodyId(b) == b2StoreBodyId(s->ragdoll->body[sensor_body[i]])) s->contact[i]++;
    }
    for (int k = 0; k < ev.endCount; k++) {
        b2BodyId b = b2Shape_GetBody(ev.endEvents[k].sensorShapeId);
        for (int i = 0; i < 6; i++)
            if (b2StoreBodyId(b) == b2StoreBodyId(s->ragdoll->body[sensor_body[i]]) && s->contact[i] > 0) s->contact[i]--;
    }
}

void p14_sim_step(P14Sim* s, P14Controller* c, float dt, P14Gait gait, bool crouch, float lean) {
    bool hands = s->force_hands || s->contact[2] > 0 || s->contact[3] > 0;
    bool knees = s->force_knees || s->contact[4] > 0 || s->contact[5] > 0;
    p14_controller_set_contacts(c, hands, knees);
    p14_controller_update(c, dt, gait, crouch, lean);

    if (!crouch && gait == P14_WALK) s->x += 0.25f * dt;
    if (!crouch && gait == P14_RUN) s->x += 0.45f * dt;

    /* La hauteur du bassin suit la consigne avec une vitesse bornée : pas de
       téléportation d'un état debout à un état couché en un seul pas. */
    float dy = c->hip[0].y - s->py;
    float max_dy = 1.2f * dt;
    s->py += dy > max_dy ? max_dy : (dy < -max_dy ? -max_dy : dy);
    b2Body_SetTransform(s->ragdoll->body[P14_PELVIS], (b2Pos){ s->x, s->py }, b2Rot_identity);
    b2Body_SetAwake(s->ragdoll->body[P14_PELVIS], true);

    for (int i = 0; i < 2; i++) {
        /* Angles déjà résolus par le contrôleur : mêmes longueurs d'os que le modèle. */
        P14LegSolution sol = c->leg[i];
        int jh = i == 0 ? P14_J_HIP_BACK : P14_J_HIP_FRONT;
        int jk = i == 0 ? P14_J_KNEE_BACK : P14_J_KNEE_FRONT;
        int jf = i == 0 ? P14_J_ANKLE_BACK : P14_J_ANKLE_FRONT;
        motor_to(s->ragdoll->joint[jh], sol.hip_angle + 1.5707964f, 80.0f);
        motor_to(s->ragdoll->joint[jk], sol.knee_angle, 80.0f);
        motor_to(s->ragdoll->joint[jf], -(sol.hip_angle + 1.5707964f + sol.knee_angle), 20.0f);
    }
    for (int j = 0; j < P14_JOINTS; j++) {
        if (j == P14_J_HIP_BACK || j == P14_J_HIP_FRONT || j == P14_J_KNEE_BACK ||
            j == P14_J_KNEE_FRONT || j == P14_J_ANKLE_BACK || j == P14_J_ANKLE_FRONT) continue;
        motor_to(s->ragdoll->joint[j], 0.0f, 12.0f);
    }

    b2World_Step(s->world, dt, 4);
    process_sensors(s);
}

typedef struct {
    const char* name;
    float y, angle, dx;
    bool foot, hand_knee_forced;
} P14Metric;

P14Metric p14_sim_run(P14Sim* s, P14Controller* c, P14Gait gait, bool crouch, int steps, float dt) {
    float x0 = b2Body_GetPosition(s->ragdoll->body[P14_PELVIS]).x;
    for (int i = 0; i < steps; i++) p14_sim_step(s, c, dt, gait, crouch, 0.0f);
    P14Metric m;
    m.name = "sim";
    m.y = p14_sim_pelvis_y(s);
    m.angle = p14_sim_angle(s);
    m.dx = b2Body_GetPosition(s->ragdoll->body[P14_PELVIS]).x - x0;
    m.foot = p14_sim_foot_contact(s);
    m.hand_knee_forced = s->force_hands || s->force_knees;
    return m;
}
