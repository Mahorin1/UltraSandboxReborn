/* Test sans fenêtre : le ragdoll doit se tenir debout ~10 s simulées. */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "physics.h"
#include "ragdoll.h"

int main(int argc, char** argv) {
    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, argc > 1 ? (float)atof(argv[1]) : 300.0f }, 1.0f);
    RagdollInput in = {0};
    for (int i = 0; i <= 800; i++) {
        ragdoll_update(r, 1.0f / 80.0f, 1.0f, &in);
        physics_update(&ph, 1.0f / 80.0f, 1.0f);
        if (i % 100 == 0) {
            b2Vec2 c = m2px(b2Body_GetPosition(r->body[RD_CHEST]));
            printf("t=%.2fs chest=(%.0f,%.0f) angle=%.1f deg ground=%d active=%d\n", i / 80.0, c.x, c.y,
                   rad2deg(b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_CHEST]))), r->on_ground, r->active);
        }
    }
    ragdoll_destroy(r); physics_shutdown(&ph);
    return 0;
}
