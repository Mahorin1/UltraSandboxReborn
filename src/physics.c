#include "physics.h"
#include "ragdoll.h"
#include "util.h"
#include <stdlib.h>
#include <unistd.h>

int physics_default_workers(void) {
    const char* e = getenv("US_WORKERS");
    if (e) return atoi(e);
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 2 ? (int)(n / 2) : 1; /* ~cœurs physiques, SMT inutile pour Box2D */
}

void physics_init(Physics* p, int workers) {
    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = (b2Vec2){ 0.0f, 18.0f }; /* y vers le bas, comme l'ancien code */
    wd.workerCount = workers;
    p->world = b2CreateWorld(&wd);
    p->acc = 0.0f; p->step = 1.0f / 80.0f; p->substeps = 4;

    b2BodyDef bd = b2DefaultBodyDef();
    bd.position = px2m((b2Vec2){ 850.0f, 1000.0f });
    p->ground = b2CreateBody(p->world, &bd);
    b2ShapeDef sd = b2DefaultShapeDef();
    sd.enableSensorEvents = true;
    sd.material.friction = 1.0f;
    b2Polygon box = b2MakeBox(9000.0f / PPM, 50.0f / PPM);
    b2CreatePolygonShape(p->ground, &sd, &box);

    /* ancre statique sans forme : cible mobile du DragJoint */
    b2BodyDef ad = b2DefaultBodyDef();
    p->drag_anchor = b2CreateBody(p->world, &ad);
    p->drag = (DragJoint){ 0 };
}

void physics_update(Physics* p, float dt, float speed) {
    p->acc += dt;
    while (p->acc >= p->step) {
        b2World_Step(p->world, p->step * speed, p->substeps);
        ragdoll_process_sensor_events(p->world);
        p->acc -= p->step;
    }
}

void physics_shutdown(Physics* p) { b2DestroyWorld(p->world); }

/* ---------- drag souris ---------- */

typedef struct { b2Vec2 point; b2BodyId body; } DragPick;

static bool drag_pick(b2ShapeId shape, void* ctx) {
    DragPick* pick = ctx;
    if (!b2Shape_IsValid(shape) || b2Shape_IsSensor(shape)) return true;
    b2BodyId body = b2Shape_GetBody(shape);
    if (b2Body_GetType(body) != b2_dynamicBody) return true;
    b2Pos point = { pick->point.x, pick->point.y };
    if (!b2Shape_TestPoint(shape, point)) return true;
    pick->body = body;
    return false;
}

void physics_drag_begin(Physics* p, b2Vec2 target_px) {
    physics_drag_end(p);

    b2Vec2 target = px2m(target_px);
    const float r = 10.0f / PPM; /* même zone de saisie que l'ancien DragJoint */
    b2AABB aabb = { .lowerBound = { target.x - r, target.y - r },
                    .upperBound = { target.x + r, target.y + r } };
    DragPick pick = { .point = target, .body = b2_nullBodyId };
    b2QueryFilter filter = b2DefaultQueryFilter();
    b2World_OverlapAABB(p->world, (b2Pos){ 0.0f, 0.0f }, aabb, filter, drag_pick, &pick);
    if (!b2Body_IsValid(pick.body)) return;

    b2Body_SetAwake(pick.body, true);
    b2Body_SetTransform(p->drag_anchor, (b2Pos){ target.x, target.y }, b2Rot_identity);

    b2MotorJointDef jd = b2DefaultMotorJointDef();
    jd.base.bodyIdA = p->drag_anchor;
    jd.base.bodyIdB = pick.body;
    jd.base.localFrameA.p = b2Vec2_zero;
    jd.base.localFrameB.p = b2Body_GetLocalPoint(pick.body, (b2Pos){ target.x, target.y });
    jd.linearHertz = 17.0f;       /* raideur de l'ancien mouse joint */
    jd.linearDampingRatio = 2.5f;
    float mass = b2Body_GetMass(pick.body);
    jd.maxSpringForce = 3000.0f * (mass > 0.0f ? mass : 1.0f);
    jd.maxVelocityForce = 0.0f;
    jd.angularHertz = 0.0f;       /* rotation libre (Q/E la pilote) */
    jd.maxSpringTorque = 0.0f;
    jd.maxVelocityTorque = 0.0f;
    b2JointId joint = b2CreateMotorJoint(p->world, &jd);
    if (!b2Joint_IsValid(joint)) return;

    p->drag = (DragJoint){
        .body = pick.body, .joint = joint, .local_point = jd.base.localFrameB.p,
        .target = target, .spin = 0.0f, .active = true,
    };
}

void physics_drag_update(Physics* p, b2Vec2 target_px) {
    DragJoint* d = &p->drag;
    if (!d->active) return;
    if (!b2Body_IsValid(d->body) || !b2Joint_IsValid(d->joint)) { physics_drag_end(p); return; }

    d->target = px2m(target_px);
    b2Body_SetTransform(p->drag_anchor, (b2Pos){ d->target.x, d->target.y }, b2Rot_identity);
    b2Body_SetAwake(d->body, true);
}

void physics_drag_rotate(Physics* p, int dir, float dt) {
    DragJoint* d = &p->drag;
    if (!d->active || !b2Body_IsValid(d->body)) return;
    if (dir == 0) { d->spin = 0.0f; return; }
    d->spin = clampf(d->spin + (float)dir * 30.0f * dt, -8.0f, 8.0f);
    b2Body_SetAngularVelocity(d->body, d->spin);
}

void physics_drag_end(Physics* p) {
    DragJoint* d = &p->drag;
    if (d->active && b2Joint_IsValid(d->joint)) b2DestroyJoint(d->joint);
    *d = (DragJoint){ 0 };
}
