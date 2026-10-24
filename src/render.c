#include "render.h"
#include <raylib.h>

static Color unpack(uint32_t c) {
    return (Color){ (c >> 24) & 255, (c >> 16) & 255, (c >> 8) & 255, c & 255 };
}

static void draw_part(const Ragdoll* r, int i) {
    b2Vec2 p = m2px(b2Body_GetPosition(r->body[i]));
    float deg = rad2deg(b2Rot_GetAngle(b2Body_GetRotation(r->body[i])));
    float o = r->outline, w = r->dims[i][0], h = r->dims[i][1];
    if (i == RD_HEAD) {
        DrawCircleV((Vector2){ p.x, p.y }, w + o, BLACK);
        DrawCircleV((Vector2){ p.x, p.y }, w, unpack(r->color[i]));
        return;
    }
    DrawRectanglePro((Rectangle){ p.x, p.y, w + 2 * o, h + 2 * o }, (Vector2){ w / 2 + o, h / 2 + o }, deg, BLACK);
    DrawRectanglePro((Rectangle){ p.x, p.y, w, h }, (Vector2){ w / 2, h / 2 }, deg, unpack(r->color[i]));
}

void render_ragdoll(const Ragdoll* r) {
    static const int order[RD_PARTS] = { 2, 4, 6, 8, 10, 1, 0, 7, 9, 11, 3, 5 }; /* ordre de l'ancien draw() */
    for (int k = 0; k < RD_PARTS; k++) draw_part(r, order[k]);
}

void render_ground(void) {
    DrawRectangle(-8150, 950, 18000, 100, (Color){ 127, 127, 76, 255 });
    DrawRectangleLinesEx((Rectangle){ -8150, 950, 18000, 100 }, 4, BLACK);
}

void render_ragdoll_debug(const Ragdoll* r) {
    for (int i = 0; i < RD_JOINTS; i++) {
        b2JointId j = r->art[i].id;
        b2Vec2 a = m2px(b2Body_GetWorldPoint(b2Joint_GetBodyA(j), b2Joint_GetLocalFrameA(j).p));
        DrawCircleV((Vector2){ a.x, a.y }, 3.0f, RED);
    }
    for (int i = RD_FOOT1; i <= RD_FOOT2; i++) {
        b2Vec2 p = m2px(b2Body_GetPosition(r->body[i]));
        DrawCircleLines((int)p.x, (int)p.y, 30.0f, r->on_ground ? GREEN : YELLOW);
    }
    DrawCircleV((Vector2){ r->leg1.point.x, r->leg1.point.y }, 5.0f, SKYBLUE);
    DrawCircleV((Vector2){ r->leg2.point.x, r->leg2.point.y }, 5.0f, ORANGE);
    b2Vec2 c = m2px(b2Body_GetPosition(r->body[RD_CHEST]));
    DrawText(TextFormat("active=%d ground=%d wake=%d dir=%d", r->active, r->on_ground, r->wake_up, r->dir),
             (int)c.x - 80, (int)c.y - 110, 14, WHITE);
}
