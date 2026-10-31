#pragma once
#include <stdbool.h>
#include <box2d/box2d.h>

/* Portage C du ragdoll du repo UltraSandbox_Old (C++/Box2D v2).
   Mêmes 12 corps, 11 articulations, mêmes dimensions, mêmes références
   d'angle (chevilles à 90°) et mêmes matériaux que l'original. */

enum { RD_OLD_PARTS = 12, RD_OLD_JOINTS = 11 };

enum {
    RD_HEAD = 0, RD_CHEST,
    RD_BACKARM1, RD_BACKARM2, RD_FRONTARM1, RD_FRONTARM2,
    RD_BACKLEG1, RD_BACKLEG2, RD_FRONTLEG1, RD_FRONTLEG2,
    RD_FOOT1, RD_FOOT2
};

typedef struct {
    b2WorldId world;
    b2BodyId  body[RD_OLD_PARTS];
    b2JointId joint[RD_OLD_JOINTS];
    float     dim[RD_OLD_PARTS][2]; /* m */
    int       contact[2];           /* compteurs de capteurs de pied */
    int       n_contact;
} RdOld;

/* pos_m : centre du torse, en mètres. size : échelle (1 = taille d'origine). */
RdOld* rd_old_create(b2WorldId world, b2Vec2 pos_m, float size, int group);
void   rd_old_destroy(RdOld* r);

/* à appeler après chaque b2World_Step */
void rd_old_process_sensors(RdOld* r);

/* cibles moteur (rad, repère articulaire) + vitesse/frottement par joint */
void  rd_old_set_targets(RdOld* r, const float target[RD_OLD_JOINTS], float speed,
                         const float torque[RD_OLD_JOINTS]);

/* couple moteur normalisé [-1, 1] par articulation (contrôle RL) */
void  rd_old_apply_torques(RdOld* r, const float action[RD_OLD_JOINTS]);

/* couple brut (N·m), borné par les limites de l'articulation */
void  rd_old_apply_tau(RdOld* r, const float tau[RD_OLD_JOINTS]);
float rd_old_joint_angle(const RdOld* r, int j);
float rd_old_joint_vel(const RdOld* r, int j);
