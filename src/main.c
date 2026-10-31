#include <raylib.h>
#include <stdio.h>
#include "physics.h"
#include "policy.h"
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

    /* politique entraînée avec la physique du jeu : ./ml/train 1500 --game */
    Policy policy;
    bool has_policy = policy_load(&policy, "ml/policy_game.bin") ||
                      policy_load(&policy, "policy_game.bin");

    bool debug = false;
    Camera2D cam = { .offset = { 850, 475 }, .target = { 850, 475 }, .zoom = 1.0f };

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.1f) dt = 0.1f;
        if (IsKeyPressed(KEY_F1)) debug = !debug;
        if (IsKeyPressed(KEY_F2) && has_policy)
            ragdoll_set_policy(dolls[0], !dolls[0]->policy_on);
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

        /* drag souris : gauche = saisir le corps sous le curseur, Q/E = rotation */
        Vector2 mouse = GetScreenToWorld2D(GetMousePosition(), cam);
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            physics_drag_begin(&ph, (b2Vec2){ mouse.x, mouse.y });
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && ph.drag.active)
            physics_drag_update(&ph, (b2Vec2){ mouse.x, mouse.y });
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
            physics_drag_end(&ph);
        physics_drag_rotate(&ph, (IsKeyDown(KEY_E) ? 1 : 0) - (IsKeyDown(KEY_Q) ? 1 : 0), dt);

        RagdollInput in = {
            .left = IsKeyDown(KEY_A), .right = IsKeyDown(KEY_D),
            .up = IsKeyDown(KEY_UP), .down = IsKeyDown(KEY_DOWN),
            .run = IsKeyDown(KEY_LEFT_SHIFT),
            .crouch = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_C),
            .foot_inc = IsKeyDown(KEY_I), .foot_dec = IsKeyDown(KEY_K),
        };
        if (dolls[0]->policy_on) {
            float cmd = (in.right ? 1.0f : 0.0f) - (in.left ? 1.0f : 0.0f);
            ragdoll_policy_update(dolls[0], &policy, dt * speed, cmd);
        } else {
            ragdoll_update(dolls[0], dt, speed, &in);
        }
        for (int i = 1; i < n; i++)
            ragdoll_update(dolls[i], dt, speed, &(RagdollInput){ 0 });
        physics_update(&ph, dt, speed);

        BeginDrawing();
        ClearBackground((Color){ 31, 68, 112, 255 });
        BeginMode2D(cam);
            render_ground();
            for (int i = 0; i < n; i++) { render_ragdoll(dolls[i]); if (debug) render_ragdoll_debug(dolls[i]); }
            render_drag(&ph);
        EndMode2D();
        DrawText(TextFormat("%d fps | workers %d | bodies %d", GetFPS(), physics_default_workers(), n * RD_PARTS), 19, 910, 20, WHITE);
        DrawText("A/D bouger | Shift courir | Ctrl/C accroupi | Haut relever | Bas ragdoll | clic gauche drag | Q/E rotation | H spawn | S slow-mo | F1 debug", 19, 18, 18, WHITE);
        DrawText(has_policy
                     ? TextFormat("F2 politique neuronale : %s (A/D commande)",
                                  dolls[0]->policy_on ? "ON" : "OFF")
                     : "F2 politique neuronale : ml/policy_game.bin absent",
                 19, 42, 18, dolls[0]->policy_on ? (Color){ 120, 230, 150, 255 } : WHITE);
        EndDrawing();
    }

    for (int i = 0; i < n; i++) ragdoll_destroy(dolls[i]);
    physics_shutdown(&ph);
    CloseWindow();
    return 0;
}
