#pragma once
#include <stdint.h>
#include "util.h"

/* Indices des parties (identiques à l'ancien code) */
enum {
    RD_HEAD = 0, RD_CHEST = 1,
    RD_BACKARM1 = 2, RD_BACKARM2 = 3, RD_FRONTARM1 = 4, RD_FRONTARM2 = 5,
    RD_BACKLEG1 = 6, RD_BACKLEG2 = 7, RD_FRONTLEG1 = 8, RD_FRONTLEG2 = 9,
    RD_FOOT1 = 10, RD_FOOT2 = 11,
    RD_PARTS = 12, RD_JOINTS = 11
};

typedef struct { bool left, right, up, down, foot_inc, foot_dec; } RagdollInput;

typedef struct { b2JointId id; bool active, short_mode; float target, speed; } Articulation;
typedef struct { bool active; b2Vec2 point, dims; } IkAnim; /* px */

typedef struct Ragdoll {
    b2BodyId body[RD_PARTS];
    float    dims[RD_PARTS][2];  /* px (tête : [0] = rayon) */
    uint32_t color[RD_PARTS];    /* 0xRRGGBBAA */
    float    outline;            /* px */
    Articulation art[RD_JOINTS];
    IkAnim leg1, leg2, arm1, arm2;
    Pid balancer;
    float chest_mid_y, leg_dim_y, walk_angle, foot_step, wake_timer;
    int   dir, group, n_contact;
    bool  active, wake_up, on_ground;
} Ragdoll;

Ragdoll* ragdoll_create(b2WorldId w, b2Vec2 pos_px, float size);
void ragdoll_destroy(Ragdoll* r);
void ragdoll_update(Ragdoll* r, float dt, float game_speed, const RagdollInput* in);
void ragdoll_set_active(Ragdoll* r, bool on);
void ragdoll_set_dir(Ragdoll* r, int dir);
void ragdoll_process_sensor_events(b2WorldId w); /* après chaque b2World_Step */
