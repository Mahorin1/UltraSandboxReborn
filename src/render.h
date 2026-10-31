#pragma once
#include "ragdoll.h"
#include "physics.h"
void render_ragdoll(const Ragdoll* r);
void render_ground(void);
void render_drag(const Physics* p);
void render_ragdoll_debug(const Ragdoll* r); /* F1 */
