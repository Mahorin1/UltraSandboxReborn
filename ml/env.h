#pragma once
#include <stdbool.h>
#include "rd_old.h"

#define ENV_OBS 32
#define ENV_ACT RD_OLD_JOINTS
#define ENV_DT_DEFAULT (1.0f / 60.0f)

typedef struct Env Env;

Env*  env_create(unsigned seed);

/* physique alternative : le jeu tourne en gravité 18, pas 1/80, 4 sous-pas */
void  env_set_physics(Env* e, float gravity, float dt, int substeps);
float env_dt(const Env* e);
void  env_destroy(Env* e);

/* cmd : -1 (gauche), 0 (rester), +1 (droite). push : perturbation aléatoire. */
void  env_reset(Env* e, unsigned seed, float cmd, bool push);
void  env_obs(const Env* e, float* obs);

/* une décision (1/60 s) ; renvoie la reward, met *done à jour */
float env_step(Env* e, const float* action, bool* done);

float env_upright(const Env* e);   /* cos(angle du torse) : 1 = droit */
int   env_contact(const Env* e, int leg);
float env_joint_angle(const Env* e, int j);
float env_chest_y(const Env* e);   /* hauteur du torse au-dessus du sol, m */
float env_x(const Env* e);         /* position x du centre de masse, m */
float env_vx(const Env* e);        /* vitesse x du centre de masse, m/s */

/* accès lecture seule pour le rendu (view.c) */
b2WorldId    env_world(const Env* e);
const RdOld* env_rag(const Env* e);

/* impulsion horizontale sur le torse, pour tester l'équilibre */
void env_push(Env* e, float dir);
