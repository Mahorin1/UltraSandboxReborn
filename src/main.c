#include <raylib.h>
#include <stdio.h>
#include "physics.h"
#include "ragdoll.h"
#include "render.h"

#define MAX_RAGDOLLS 64

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1700, 950, "Ultra Sandbox Reborn");

    Physics ph;
    physics_init(&ph, physics_default_workers());

    Ragdoll* dolls[MAX_RAGDOLLS] = { 0 };
    int n = 0;
    dolls[n++] = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 300.0f }, 1.0f);

    bool debug = false;
    Camera2D cam = { .offset = { 850, 475 }, .target = { 850, 475 }, .zoom = 1.0f };

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.1f) dt = 0.1f;
        if (IsKeyPressed(KEY_F1)) debug = !debug;
        float speed = IsKeyDown(KEY_S) ? 0.15f : 1.0f;

        /* caméra : clic droit = pan, molette = zoom */
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            cam.target.x -= d.x / cam.zoom; cam.target.y -= d.y / cam.zoom;
        }
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) cam.zoom = cam.zoom * (wheel > 0 ? 1.15f : 1.0f / 1.15f);

        if (IsKeyPressed(KEY_H) && n < MAX_RAGDOLLS) {
            Vector2 m = GetScreenToWorld2D(GetMousePosition(), cam);
            dolls[n++] = ragdoll_create(ph.world, (b2Vec2){ m.x, m.y }, 1.0f);
        }

        RagdollInput in = {
            .left = IsKeyDown(KEY_A), .right = IsKeyDown(KEY_D),
            .up = IsKeyDown(KEY_UP), .down = IsKeyDown(KEY_DOWN),
            .foot_inc = IsKeyDown(KEY_I), .foot_dec = IsKeyDown(KEY_K),
        };
        for (int i = 0; i < n; i++)
            ragdoll_update(dolls[i], dt, speed, i == 0 ? &in : &(RagdollInput){ 0 });
        physics_update(&ph, dt, speed);

        BeginDrawing();
        ClearBackground((Color){ 31, 68, 112, 255 });
        BeginMode2D(cam);
            render_ground();
            for (int i = 0; i < n; i++) { render_ragdoll(dolls[i]); if (debug) render_ragdoll_debug(dolls[i]); }
        EndMode2D();
        DrawText(TextFormat("%d fps | workers %d | bodies %d", GetFPS(), physics_default_workers(), n * RD_PARTS), 19, 910, 20, WHITE);
        DrawText("A/D marcher | Haut/Bas actif/inactif | H spawn | S slow-mo | F1 debug | clic droit pan | molette zoom", 19, 18, 18, WHITE);
        EndDrawing();
    }

    for (int i = 0; i < n; i++) ragdoll_destroy(dolls[i]);
    physics_shutdown(&ph);
    CloseWindow();
    return 0;
}
