#pragma once
#include <stdbool.h>
#include <box2d/box2d.h>

/* Prototype isolé : 14 corps, bassin et colonne inclus. */
enum {
    P14_PELVIS = 0, P14_SPINE = 1, P14_CHEST = 2, P14_HEAD = 3,
    P14_BACKARM1 = 4, P14_BACKARM2 = 5, P14_FRONTARM1 = 6, P14_FRONTARM2 = 7,
    P14_BACKLEG1 = 8, P14_BACKLEG2 = 9, P14_FRONTLEG1 = 10, P14_FRONTLEG2 = 11,
    P14_FOOT1 = 12, P14_FOOT2 = 13,
    P14_PARTS = 14
};

enum {
    P14_J_PELVIS_SPINE = 0, P14_J_SPINE_CHEST = 1, P14_J_NECK = 2,
    P14_J_SHOULDER_BACK = 3, P14_J_SHOULDER_FRONT = 4,
    P14_J_ELBOW_BACK = 5, P14_J_ELBOW_FRONT = 6,
    P14_J_HIP_BACK = 7, P14_J_HIP_FRONT = 8,
    P14_J_KNEE_BACK = 9, P14_J_KNEE_FRONT = 10,
    P14_J_ANKLE_BACK = 11, P14_J_ANKLE_FRONT = 12,
    P14_JOINTS = 13
};

typedef struct { float x, y; } P14Vec2;

typedef struct {
    b2WorldId world;
    b2BodyId body[P14_PARTS];
    b2JointId joint[P14_JOINTS];
    float dim[P14_PARTS][2]; /* largeur x hauteur, m (rendu) */
} P14Ragdoll;

typedef struct {
    bool reachable;
    float hip_angle;   /* angle absolu de la cuisse */
    float knee_angle;  /* angle relatif du tibia */
    P14Vec2 knee;
    P14Vec2 foot;
} P14LegSolution;

typedef enum { P14_STAND, P14_WALK, P14_RUN, P14_CROUCH, P14_GETUP } P14Gait;
typedef enum { P14_ROLL, P14_PUSH, P14_KNEEL, P14_EXTEND, P14_DONE } P14GetupState;

typedef struct {
    P14Gait gait;
    P14Gait last_input;  /* entrée du pas précédent (front montant de GETUP) */
    float phase;
    P14Vec2 hip[2];
    P14Vec2 foot[2];
    P14LegSolution leg[2];
    float balance;
    P14GetupState getup;
    float getup_t;
    bool hand_contact, knee_contact;
} P14Controller;

P14Ragdoll* p14_create(b2WorldId world);
void p14_destroy(P14Ragdoll* r);

P14LegSolution p14_leg_ik(P14Vec2 hip, P14Vec2 foot, float thigh, float shin, int bend);

void p14_controller_init(P14Controller* c);
void p14_controller_begin_getup(P14Controller* c);
void p14_controller_set_contacts(P14Controller* c, bool hands, bool knees);
void p14_controller_update(P14Controller* c, float dt, P14Gait gait, bool crouch, float lean);
