#pragma once
#include <stdbool.h>
#include "ragdoll.h"

/* Politique neuronale apprise dans ml/ (MLP 32-32-32-11) et son application
   au ragdoll du jeu. Aucune dépendance à Raylib : compilable en headless. */

#define POLICY_OBS 32
#define POLICY_ACT RD_JOINTS
#define POLICY_H1  32
#define POLICY_H2  32

typedef struct {
    float w1[POLICY_OBS * POLICY_H1], b1[POLICY_H1];
    float w2[POLICY_H1 * POLICY_H2], b2[POLICY_H2];
    float w3[POLICY_H2 * POLICY_ACT], b3[POLICY_ACT];
    bool  loaded;
} Policy;

/* charge ml/policy_game.bin (magic USRNN1) ; false si absent ou invalide */
bool policy_load(Policy* p, const char* path);

void policy_forward(const Policy* p, const float obs[POLICY_OBS], float act[POLICY_ACT]);

/* pilote le ragdoll à pas fixe 1/80 (même physique que l'entraînement).
   cmd : -1 gauche, 0 immobile, +1 droite. */
void ragdoll_policy_update(Ragdoll* r, const Policy* p, float dt, float cmd);
