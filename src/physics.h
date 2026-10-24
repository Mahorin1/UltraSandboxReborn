#pragma once
#include <box2d/box2d.h>

typedef struct {
    b2WorldId world;
    float acc, step;
    int substeps;
    b2BodyId ground;
} Physics;

/* workers <= 1 : mono-thread ; sinon scheduler interne de Box2D */
void physics_init(Physics* p, int workers);
void physics_update(Physics* p, float dt, float speed);
void physics_shutdown(Physics* p);
int  physics_default_workers(void);
