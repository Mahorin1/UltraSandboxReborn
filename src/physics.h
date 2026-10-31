#pragma once
#include <box2d/box2d.h>

/* Drag souris : Box2D 3.2 a retiré b2MouseJoint, le motor joint à ressort le remplace.
   Le point saisi (local_point) est tiré vers target par un ressort amorti. */
typedef struct {
    b2BodyId body;
    b2JointId joint;
    b2Vec2 local_point; /* m, repère du corps */
    b2Vec2 target;      /* m, monde */
    float spin;         /* rad/s, rampe Q/E */
    bool active;
} DragJoint;

typedef struct {
    b2WorldId world;
    float acc, step;
    int substeps;
    b2BodyId ground;
    b2BodyId drag_anchor; /* corps statique sans forme, cible du DragJoint */
    DragJoint drag;
} Physics;

/* workers <= 1 : mono-thread ; sinon scheduler interne de Box2D */
void physics_init(Physics* p, int workers);
void physics_update(Physics* p, float dt, float speed);
void physics_shutdown(Physics* p);
int  physics_default_workers(void);

/* drag : begin prend le corps sous target_px ; rotate(dir=-1/0/1) tourne le corps saisi */
void physics_drag_begin(Physics* p, b2Vec2 target_px);
void physics_drag_update(Physics* p, b2Vec2 target_px);
void physics_drag_rotate(Physics* p, int dir, float dt);
void physics_drag_end(Physics* p);
