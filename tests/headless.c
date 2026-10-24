/* Test sans fenêtre : le ragdoll doit se tenir debout après 10 s simulées. */
#include <stdio.h>
#include <stdlib.h>
#include "physics.h"
#include "ragdoll.h"

int main(int argc, char** argv) {
    float y = argc > 1 ? (float)atof(argv[1]) : 845.0f;
    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, y }, 1.0f);
    RagdollInput in = { 0 };
    float angle = 0.0f, cy = 0.0f;
    for (int i = 0; i <= 800; i++) {
        ragdoll_update(r, 1.0f / 80.0f, 1.0f, &in);
        physics_update(&ph, 1.0f / 80.0f, 1.0f);
        angle = rad2deg(b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_CHEST])));
        cy = m2px(b2Body_GetPosition(r->body[RD_CHEST])).y;
        if (i % 200 == 0)
            printf("t=%.2fs chest_y=%.0f angle=%.1f ground=%d active=%d\n", i / 80.0, cy, angle, r->on_ground, r->active);
    }
    int ok = r->on_ground && r->active && angle > -30.0f && angle < 30.0f && cy < 900.0f;
    printf(ok ? "PASS\n" : "FAIL\n");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok ? 0 : 1;
}
