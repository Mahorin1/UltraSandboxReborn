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

    /* drag : saisir une caisse au curseur et la tirer de 100 px */
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = px2m((b2Vec2){ 300.0f, 900.0f });
    b2BodyId box_body = b2CreateBody(ph.world, &bd);
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Polygon poly = b2MakeBox(0.3f, 0.3f);
    b2CreatePolygonShape(box_body, &sd, &poly);

    b2Vec2 drag_start = m2px(b2Body_GetPosition(box_body));
    physics_drag_begin(&ph, drag_start);
    int grabbed = ph.drag.active && b2Body_IsValid(ph.drag.body);
    physics_drag_rotate(&ph, 1, 0.5f);
    int spun = b2Body_GetAngularVelocity(box_body) > 0.0f;
    b2Body_SetAngularVelocity(box_body, 0.0f);
    for (int i = 0; i < 80; i++) {
        physics_drag_update(&ph, (b2Vec2){ drag_start.x + 100.0f, drag_start.y });
        physics_update(&ph, 1.0f / 80.0f, 1.0f);
    }
    float moved = m2px(b2Body_GetPosition(box_body)).x - drag_start.x;
    physics_drag_end(&ph);
    int drag_ok = grabbed && spun && moved > 40.0f && !ph.drag.active;
    printf("drag: grabbed=%d spun=%d moved=%.0fpx released=%d\n", grabbed, spun, moved, !ph.drag.active);
    b2DestroyBody(box_body);

    ok = ok && drag_ok;
    printf(ok ? "PASS\n" : "FAIL\n");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok ? 0 : 1;
}
