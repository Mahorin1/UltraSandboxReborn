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
