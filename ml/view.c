#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#include "env.h"
#include "nn.h"

/* Visualiseur du ragdoll du repo Old piloté par policy.bin.
   Même Env que eval.c : ce qu'on voit ici est exactement ce qui est évalué.
   A/D (ou flèches) : commander gauche/droite ; R : reset ; Espace : pousser ;
   molette : zoom. */

#define H1 32
#define H2 32
#define WIN_W 1280
#define WIN_H 720
#define ZOOM_MIN 140.0f
#define ZOOM_MAX 520.0f
#define FALL_PAUSE 0.9f

static const Color PART_COLOR[RD_OLD_PARTS] = {
    { 235, 196, 166, 255 }, /* tête */
    {  74, 124, 196, 255 }, /* torse */
    { 198, 116,  68, 255 }, { 186, 108,  62, 255 }, /* bras arrière */
    { 226, 146,  92, 255 }, { 214, 136,  84, 255 }, /* bras avant */
    {  72, 158, 110, 255 }, {  66, 148, 102, 255 }, /* jambe arrière */
    {  92, 186, 130, 255 }, {  84, 176, 122, 255 }, /* jambe avant */
    {  48,  58,  74, 255 }, {  54,  64,  80, 255 }, /* pieds */
};

static int load_policy(const char* path, float* out, int n) {
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }
    char magic[6];
    int count = 0;
    if (fread(magic, 1, 6, f) != 6 || memcmp(magic, "USRNN1", 6) != 0 ||
        fread(&count, sizeof count, 1, f) != 1 || count != n ||
        fread(out, sizeof(float), (size_t)n, f) != (size_t)n) {
        fprintf(stderr, "%s: format invalide (attendu %d params)\n", path, n);
        fclose(f);
        return 0;
    }
    fclose(f);
    return 1;
}

static Vector2 v2(b2Vec2 p) { return (Vector2){ p.x, p.y }; }

static void draw_disc(Vector2 c, float r, Color fill, Color edge) {
    DrawCircleV(c, r, fill);
    const int seg = 24;
    Vector2 prev = { c.x + r, c.y };
    for (int i = 1; i <= seg; i++) {
        float a = 6.2831853f * (float)i / (float)seg;
        Vector2 p = { c.x + cosf(a) * r, c.y + sinf(a) * r };
        DrawLineEx(prev, p, 0.012f, edge);
        prev = p;
    }
}

static void draw_part(const RdOld* rd, int i) {
    const Color edge = Fade(BLACK, 0.45f);
    b2Transform xf = b2Body_GetTransform(rd->body[i]);
    b2ShapeId ids[4];
    int n = b2Body_GetShapes(rd->body[i], ids, 4);
    for (int k = 0; k < n; k++) {
        if (b2Shape_IsSensor(ids[k])) continue;
        b2ShapeType type = b2Shape_GetType(ids[k]);
        if (type == b2_polygonShape) {
            b2Polygon poly = b2Shape_GetPolygon(ids[k]);
            if (poly.count < 3) continue;
            Vector2 p[8];
            int c = poly.count > 8 ? 8 : poly.count;
            for (int j = 0; j < c; j++) p[j] = v2(b2TransformPoint(xf, poly.vertices[j]));
            for (int j = 1; j + 1 < c; j++) DrawTriangle(p[0], p[j], p[j + 1], PART_COLOR[i]);
            for (int j = 0; j < c; j++) DrawLineEx(p[j], p[(j + 1) % c], 0.012f, edge);
        } else if (type == b2_circleShape) {
            b2Circle c = b2Shape_GetCircle(ids[k]);
            draw_disc(v2(b2TransformPoint(xf, c.center)), c.radius, PART_COLOR[i], edge);
        }
    }
}

static void draw_dashes(float x, float y0, float y1, float w, Color c) {
    for (float y = y0; y < y1; y += 0.18f) {
        float y2 = y + 0.10f;
        DrawLineEx((Vector2){ x, y }, (Vector2){ x, y2 > y1 ? y1 : y2 }, w, c);
    }
}

static void draw_scene(const Env* env, float cam_x, float x_start, float zoom) {
    (void)zoom;
    float left = cam_x - 45.0f;
    DrawRectangleRec((Rectangle){ left, 0.0f, 90.0f, 2.0f }, (Color){ 38, 42, 50, 255 });
    DrawRectangleRec((Rectangle){ left, 0.0f, 90.0f, 0.03f }, (Color){ 104, 112, 130, 255 });

    for (int i = (int)floorf(left); i <= (int)ceilf(cam_x + 45.0f); i++) {
        float h = (i % 5 == 0) ? 0.22f : 0.10f;
        DrawLineEx((Vector2){ (float)i, 0.0f }, (Vector2){ (float)i, h }, 0.012f,
                   (Color){ 112, 122, 142, 255 });
    }

    /* origine du monde */
    DrawLineEx((Vector2){ 0.0f, -3.0f }, (Vector2){ 0.0f, 0.0f }, 0.008f,
               (Color){ 64, 74, 94, 255 });
    /* point de départ de l'épisode : repère de déplacement */
    draw_dashes(x_start, -2.4f, 0.0f, 0.012f, (Color){ 236, 196, 92, 255 });

    const RdOld* rd = env_rag(env);
    if (rd) for (int i = 0; i < RD_OLD_PARTS; i++) draw_part(rd, i);
}

static const char* cmd_label(float cmd) {
    if (cmd > 0.5f) return "DROITE";
    if (cmd < -0.5f) return "GAUCHE";
    return "IMMOBILE";
}

int main(int argc, char** argv) {
    const char* path = "policy.bin";
    if (argc > 1 && argv[1][0] != '-') path = argv[1];

    Nn* nn = nn_create(ENV_OBS, H1, H2, ENV_ACT, 1u);
    if (!nn) { fprintf(stderr, "nn_create: echec\n"); return 1; }
    const int np = nn_param_count(nn);
    float* theta = malloc(sizeof(float) * (size_t)np);
    if (!theta) { nn_destroy(nn); return 1; }
    if (!load_policy(path, theta, np)) {
        fprintf(stderr, "aucune politique : lance d'abord ./ml/train 1500\n");
        free(theta);
        nn_destroy(nn);
        return 1;
    }
    nn_set_params(nn, theta);

    Env* env = env_create(1234u);
    if (!env) { free(theta); nn_destroy(nn); return 1; }

    InitWindow(WIN_W, WIN_H, "ragdoll Old - politique ES (A/D pour marcher)");
    SetTargetFPS(60);

    Camera2D cam = { 0 };
    cam.offset = (Vector2){ WIN_W * 0.5f, WIN_H * 0.58f };
    cam.zoom = 240.0f;

    float cmd = 0.0f;
    float frozen = 0.0f;   /* temps restant avant reset après chute */
    float ep_time = 0.0f;  /* durée de l'épisode courant */
    float last_time = 0.0f;/* durée du dernier épisode terminé */
    unsigned seed = 1;

    env_reset(env, seed, cmd, false);
    float x_start = env_x(env);
    cam.target = (Vector2){ x_start, -0.85f };

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        float want = 0.0f;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) want += 1.0f;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) want -= 1.0f;
        if (want != cmd || IsKeyPressed(KEY_R)) {
            cmd = want;
            env_reset(env, ++seed, cmd, false);
            x_start = env_x(env);
            ep_time = 0.0f;
            frozen = 0.0f;
        }
        if (IsKeyPressed(KEY_SPACE)) env_push(env, GetRandomValue(0, 1) ? 1.0f : -1.0f);

        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f)
            cam.zoom = fminf(ZOOM_MAX, fmaxf(ZOOM_MIN, cam.zoom * (1.0f + 0.12f * wheel)));

        if (frozen > 0.0f) {
            frozen -= dt;
            if (frozen <= 0.0f) {
                env_reset(env, ++seed, cmd, false);
                x_start = env_x(env);
                ep_time = 0.0f;
            }
        } else {
            float obs[ENV_OBS], act[ENV_ACT];
            bool done = false;
            env_obs(env, obs);
            nn_forward(nn, obs, act);
            env_step(env, act, &done);
            ep_time += 1.0f / 60.0f;
            if (done) { last_time = ep_time; frozen = FALL_PAUSE; }
        }

        float tx = env_x(env);
        cam.target.x += (tx - cam.target.x) * fminf(1.0f, dt * 5.0f);
        cam.target.y = -0.85f;

        BeginDrawing();
        ClearBackground((Color){ 26, 30, 38, 255 });
        BeginMode2D(cam);
        draw_scene(env, cam.target.x, x_start, cam.zoom);
        EndMode2D();

        /* HUD */
        DrawRectangle(0, 0, 470, 122, Fade(BLACK, 0.45f));
        DrawText("ragdoll Old - politique ES", 16, 12, 20, (Color){ 226, 232, 244, 255 });
        DrawText(TextFormat("commande : %s", cmd_label(cmd)), 16, 40, 20,
                 cmd > 0.5f ? (Color){ 120, 220, 150, 255 }
                 : cmd < -0.5f ? (Color){ 140, 190, 255, 255 }
                               : (Color){ 220, 200, 120, 255 });
        DrawText(TextFormat("dx %+6.2f m   vitesse %+5.2f m/s", env_x(env) - x_start, env_vx(env)),
                 16, 68, 18, (Color){ 200, 210, 226, 255 });
        DrawText(TextFormat("upright %.2f   torse %.2f m   t %.2f s", env_upright(env),
                            env_chest_y(env), ep_time), 16, 92, 18,
                 (Color){ 200, 210, 226, 255 });

        /* pieds au sol */
        for (int i = 0; i < 2; i++) {
            Color c = env_contact(env, i) ? (Color){ 110, 220, 140, 255 }
                                          : (Color){ 70, 78, 94, 255 };
            DrawRectangle(486 + i * 26, 14, 20, 12, c);
        }
        DrawText("pieds", 486, 32, 14, (Color){ 150, 160, 180, 255 });

        DrawText("A/D ou fleches : marcher   R : reset   Espace : pousser   molette : zoom",
                 16, WIN_H - 26, 18, (Color){ 140, 150, 170, 255 });
        if (frozen > 0.0f)
            DrawText(TextFormat("CHUTE apres %.2f s", last_time), WIN_W / 2 - 80, 40, 24,
                     (Color){ 240, 130, 120, 255 });

        EndDrawing();
    }

    CloseWindow();
    env_destroy(env);
    free(theta);
    nn_destroy(nn);
    return 0;
}
