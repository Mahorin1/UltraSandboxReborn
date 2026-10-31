#pragma once
#include "pelvis14.h"

typedef struct {
    b2WorldId world;
    P14Ragdoll* ragdoll;
    b2BodyId ground;
    b2ShapeId sensor[6];
    int contact[6];
    float x, py;
    bool force_hands, force_knees;
} P14Sim;

P14Sim* p14_sim_create(void);
void p14_sim_destroy(P14Sim* s);
void p14_sim_step(P14Sim* s, P14Controller* c, float dt, P14Gait gait, bool crouch, float lean);
float p14_sim_pelvis_y(const P14Sim* s);
float p14_sim_angle(const P14Sim* s);
bool p14_sim_foot_contact(const P14Sim* s);
