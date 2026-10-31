/* Démo jouable du prototype 14 corps : rendu Raylib, contrôles clavier.
   Isolée : ne touche pas au ragdoll du jeu. Le bassin est cinématique. */
#include <raylib.h>
#include <math.h>
#include "sim_pelvis14.h"

#define WIN_W 1280
#define WIN_H 720
#define SCALE 220.0f       /* pixels par mètre */
#define GROUND_TOP 0.32f   /* dessus du sol, m */
#define GROUND_SCREEN 620.0f
#define STEP (1.0f / 100.0f)

static const Color PALETTE[P14_PARTS] = {
    { 205, 185, 150, 255 }, { 185, 165, 130, 255 }, { 120, 170, 220, 255 }, { 240, 212, 175, 255 },
    { 140, 200, 150, 255 }, { 110, 170, 120, 255 }, { 140, 200, 150, 255 }, { 110, 170, 120, 255 },
    { 230, 150, 90, 255 },  { 200, 120, 70, 255 },  { 230, 150, 90, 255 },  { 200, 120, 70, 255 },
    { 90, 90, 110, 255 },   { 90, 90, 110, 255 }
};

static Vector2 w2s(float wx, float wy, float cam) {
    return (Vector2){ WIN_W * 0.5f + (wx - cam) * SCALE, GROUND_SCREEN - (wy - GROUND_TOP) * SCALE };
}

static const char* gait_name(P14Gait g) {
    switch (g) {
        case P14_STAND: return "stand";
        case P14_WALK: return "walk";
        case P14_RUN: return "run";
        case P14_CROUCH: return "crouch";
        default: return "getup";
    }
}

static const char* getup_name(P14GetupState s) {
    switch (s) {
        case P14_ROLL: return "roll";
        case P14_PUSH: return "push";
        case P14_KNEEL: return "kneel";
        case P14_EXTEND: return "extend";
        default: return "done";
    }
}

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(WIN_W, WIN_H, "Ultra Sandbox Reborn - prototype 14 corps");
    if (!IsWindowReady()) {
        TraceLog(LOG_ERROR, "fenetre indisponible (DISPLAY/Wayland manquant)");
        return 1;
    }
    SetTargetFPS(60);

    P14Sim* sim = p14_sim_create();
    if (!sim) { CloseWindow(); return 1; }
    P14Controller ctl;
    p14_controller_init(&ctl);

    const int sensor_body[6] = { P14_FOOT1, P14_FOOT2, P14_BACKARM2, P14_FRONTARM2, P14_BACKLEG2, P14_FRONTLEG2 };
    bool debug = false, getup_req = false;
    float acc = 0.0f;

    while (!WindowShouldClose()) {
        float frame = GetFrameTime();
        if (frame > 0.1f) frame = 0.1f;
        if (IsKeyPressed(KEY_F1)) debug = !debug;
        if (IsKeyPressed(KEY_R)) getup_req = true;
        float speed = IsKeyDown(KEY_S) ? 0.15f : 1.0f;
        bool crouch = IsKeyDown(KEY_C) || IsKeyDown(KEY_LEFT_CONTROL);
        bool forward = IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT);
        bool run = IsKeyDown(KEY_LEFT_SHIFT);

        acc += frame * speed;
        int guard = 0;
        while (acc >= STEP && guard++ < 8) {
            P14Gait gait = P14_STAND;
            if (getup_req) { gait = P14_GETUP; getup_req = false; }
            else if (forward && !crouch) gait = run ? P14_RUN : P14_WALK;
            p14_sim_step(sim, &ctl, STEP, gait, crouch, 0.0f);
            acc -= STEP;
        }

        const P14Ragdoll* r = sim->ragdoll;
        float cam = b2Body_GetPosition(r->body[P14_PELVIS]).x;

        BeginDrawing();
        ClearBackground((Color){ 31, 68, 112, 255 });
        DrawRectangle(0, (int)GROUND_SCREEN, WIN_W, WIN_H - (int)GROUND_SCREEN, (Color){ 127, 127, 76, 255 });
        DrawLine(0, (int)GROUND_SCREEN, WIN_W, (int)GROUND_SCREEN, BLACK);

        for (int i = 0; i < P14_PARTS; i++) {
            b2Pos p = b2Body_GetPosition(r->body[i]);
            float deg = b2Rot_GetAngle(b2Body_GetRotation(r->body[i])) * 57.29578f;
            float w = r->dim[i][0] * SCALE, h = r->dim[i][1] * SCALE;
            Vector2 c = w2s(p.x, p.y, cam);
            DrawRectanglePro((Rectangle){ c.x, c.y, w + 4, h + 4 },
                             (Vector2){ w * 0.5f + 2, h * 0.5f + 2 }, -deg, BLACK);
            DrawRectanglePro((Rectangle){ c.x, c.y, w, h },
                             (Vector2){ w * 0.5f, h * 0.5f }, -deg, PALETTE[i]);
        }

        if (debug) {
            for (int i = 0; i < P14_JOINTS; i++) {
                b2JointId j = r->joint[i];
                b2Pos a = b2Body_GetWorldPoint(b2Joint_GetBodyA(j), b2Joint_GetLocalFrameA(j).p);
                DrawCircleV(w2s(a.x, a.y, cam), 4.0f, RED);
            }
            for (int i = 0; i < 6; i++) {
                b2Pos p = b2Body_GetPosition(r->body[sensor_body[i]]);
                Vector2 c = w2s(p.x, p.y, cam);
                DrawCircleLines((int)c.x, (int)c.y, 0.08f * SCALE, sim->contact[i] ? GREEN : DARKGRAY);
            }
            for (int i = 0; i < 2; i++) {
                Vector2 f = w2s(ctl.foot[i].x + sim->x, ctl.foot[i].y, cam);
                Vector2 h = w2s(ctl.hip[i].x + sim->x, ctl.hip[i].y, cam);
                DrawCircleLines((int)f.x, (int)f.y, 5.0f, SKYBLUE);
                DrawCircleLines((int)h.x, (int)h.y, 5.0f, ORANGE);
            }
        }

        DrawText("PROTOTYPE 14 CORPS (bassin + colonne) - racine cinematique", 18, 16, 20, RAYWHITE);
        DrawText(TextFormat("gait %s | getup %s | pied au sol %s | bassin %.3f m | x %.2f m",
                            gait_name(ctl.gait), getup_name(ctl.getup),
                            p14_sim_foot_contact(sim) ? "oui" : "non",
                            p14_sim_pelvis_y(sim), sim->x),
                 18, 44, 18, LIGHTGRAY);
        DrawText("assiste : deplacement et releve imposes - prototype isole, ragdoll du jeu inchange",
                 18, WIN_H - 58, 16, (Color){ 255, 220, 140, 255 });
        DrawText("A/D marcher  Shift courir  C accroupi  R relever  S ralenti  F1 debug  Echap quitter",
                 18, WIN_H - 30, 18, WHITE);
        EndDrawing();
    }

    p14_sim_destroy(sim);
    CloseWindow();
    return 0;
}
