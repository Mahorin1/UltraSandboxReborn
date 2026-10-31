#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "env.h"
#include "es.h"
#include "nn.h"

#define H1 32
#define H2 32
#define POP 48
#define ROLL_SECONDS 4.0f

typedef struct { float reward, upright, dx, steps; } Roll;

static float frand(unsigned* s) {
    unsigned x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}

static void save_policy(const char* path, const float* p, int n) {
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); return; }
    fwrite("USRNN1", 1, 6, f);
    fwrite(&n, sizeof n, 1, f);
    fwrite(p, sizeof(float), (size_t)n, f);
    fclose(f);
}

static int load_policy(const char* path, float* out, int n) {
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    char magic[6];
    int count = 0;
    int ok = fread(magic, 1, 6, f) == 6 && memcmp(magic, "USRNN1", 6) == 0 &&
             fread(&count, sizeof count, 1, f) == 1 && count == n &&
             fread(out, sizeof(float), (size_t)n, f) == (size_t)n;
    fclose(f);
    return ok;
}

static Roll rollout(Env* env, Nn* nn, const float* theta, unsigned seed, float cmd, bool push,
                    int steps) {
    nn_set_params(nn, theta);
    env_reset(env, seed, cmd, push);
    float obs[ENV_OBS], act[ENV_ACT];
    Roll r = { 0.0f, 0.0f, 0.0f, 0.0f };
    float x0 = env_x(env);
    for (int t = 0; t < steps; t++) {
        env_obs(env, obs);
        nn_forward(nn, obs, act);
        bool done = false;
        r.reward += env_step(env, act, &done);
        r.upright += env_upright(env);
        r.steps += 1.0f;
        if (done) break;
    }
    r.dx = env_x(env) - x0;
    r.upright /= r.steps;
    return r;
}

/* score de validation : mêmes conditions que eval (sans poussée, 3 commandes).
   Sert à ne sauvegarder que la meilleure politique réellement évaluée. */
static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

static float validate(Env* env, Nn* nn, const float* theta, int steps) {
    float s = 0.0f;
    for (int c = 0; c < 3; c++) {
        float cmd = (float)(c - 1);
        Roll r = rollout(env, nn, theta, 4242u + (unsigned)c, cmd, false, steps);
        s += r.reward;
        /* même critère que eval : récompenser le bon sens, pénaliser la dérive.
           Sans ça, une politique qui part toujours du même côté gagne en score. */
        if (cmd < -0.5f) s += 40.0f * clamp01(-r.dx);
        else if (cmd > 0.5f) s += 40.0f * clamp01(r.dx);
        else s -= 40.0f * clamp01(fabsf(r.dx));
    }
    return s / 3.0f;
}

int main(int argc, char** argv) {
    int gens = 200;
    bool game = false, resume = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--game") == 0) game = true;
        else if (strcmp(argv[i], "--resume") == 0) resume = true;
        else if (argv[i][0] != '-') gens = atoi(argv[i]);
    }
    const char* ckpt_path = game ? "policy_game.bin" : "policy.bin";
    const char* latest_path = game ? "latest_game.bin" : "latest.bin";
    const int pop = POP, pairs = POP / 2;
    const unsigned seed = 12345u;

    Nn* nn = nn_create(ENV_OBS, H1, H2, ENV_ACT, seed);
    if (!nn) { fprintf(stderr, "nn_create\n"); return 1; }
    const int np = nn_param_count(nn);
    Es es;
    es_init(&es, np, POP, 0.08f, 0.03f, seed);

    Env* env = env_create(seed);
    /* --game : mêmes conditions que src/physics.c (gravité 18, pas 1/80, 4 sous-pas) */
    if (game) env_set_physics(env, 18.0f, 1.0f / 80.0f, 4);
    const int steps = (int)lroundf(ROLL_SECONDS / env_dt(env));

    float* theta = malloc(sizeof(float) * (size_t)np);
    float* rewards = malloc(sizeof(float) * (size_t)pop);
    if (!env || !theta || !rewards) { fprintf(stderr, "alloc\n"); return 1; }

    unsigned crng = seed ^ 0xabcdu;
    float best_val = -1e9f;
    if (resume) {
        /* repartir du meilleur des deux checkpoints : reprendre aveuglément
           latest.bin écraserait un record par une politique moins bonne */
        float* alt = malloc(sizeof(float) * (size_t)np);
        bool ok_latest = alt && load_policy(latest_path, es.mean, np);
        bool ok_ckpt = alt && load_policy(ckpt_path, alt, np);
        float v_latest = ok_latest ? validate(env, nn, es.mean, steps) : -1e9f;
        float v_ckpt = ok_ckpt ? validate(env, nn, alt, steps) : -1e9f;
        if (v_ckpt >= v_latest) {
            if (ok_ckpt) memcpy(es.mean, alt, sizeof(float) * (size_t)np);
            best_val = v_ckpt;
            printf("reprise depuis %s\n", ckpt_path);
        } else {
            best_val = v_latest;
            printf("reprise depuis %s\n", latest_path);
        }
        free(alt);
        if (best_val < -1e8f) {
            resume = false;
            best_val = -1e9f;
        } else {
            es.lr = 0.015f; /* passe d'affinage : on ne casse pas ce qui marche */
            printf("validation initiale %+.2f\n", best_val);
        }
    }
    printf("policy: %d params, pop %d, %d genera%s%s, %d pas/rollout\n", np, pop, gens,
           game ? ", physique du jeu (g=18, dt=1/80)" : "", resume ? ", reprise" : "", steps);
    fflush(stdout);

    for (int g = 0; g < gens; g++) {
        /* curriculum : équilibre, puis chaque direction par bloc (pour que la
           politique apprenne les deux), puis mélange aléatoire. */
        float cmd = 0.0f;
        if (!resume && g < 100) cmd = 0.0f;
        else if (!resume && g < 200) cmd = -1.0f;
        else if (!resume && g < 300) cmd = 1.0f;
        else {
            float u = frand(&crng);
            cmd = (u < 0.5f) ? 0.0f : (u < 0.75f ? -1.0f : 1.0f);
        }
        bool push = g >= 50;

        float sum = 0.0f, max = -1e9f, up = 0.0f;
        for (int p = 0; p < pairs; p++) {
            unsigned ep_seed = (unsigned)(g * 977 + p * 13 + 1);
            es_sample_fresh(&es, p, +1, theta);
            Roll rp = rollout(env, nn, theta, ep_seed, cmd, push, steps);
            es_sample_fresh(&es, p, -1, theta);
            Roll rm = rollout(env, nn, theta, ep_seed, cmd, push, steps);
            rewards[2 * p] = rp.reward;
            rewards[2 * p + 1] = rm.reward;
            sum += rp.reward + rm.reward;
            up += rp.upright + rm.upright;
            if (rp.reward > max) max = rp.reward;
            if (rm.reward > max) max = rm.reward;
        }
        es_train(&es, rewards);
        es.sigma = fmaxf(0.015f, es.sigma * 0.995f);

        printf("gen %3d cmd %+.0f push %d | mean %8.2f max %8.2f upright %.2f sigma %.3f\n",
               g, cmd, push ? 1 : 0, sum / pop, max, up / pop, es.sigma);
        fflush(stdout);

        if (g % 25 == 24) {
            save_policy(latest_path, es.mean, np);
            float v = validate(env, nn, es.mean, steps);
            if (v > best_val) {
                best_val = v;
                save_policy(ckpt_path, es.mean, np);
                printf("  -> nouveau meilleur (validation %+.1f)\n", v);
                fflush(stdout);
            }
        }
    }

    if (best_val < -1e8f) save_policy(ckpt_path, es.mean, np);
    es_free(&es);
    env_destroy(env);
    nn_destroy(nn);
    free(theta);
    free(rewards);
    printf("saved %s (meilleure validation %+.2f)\n", ckpt_path, best_val);
    return 0;
}
