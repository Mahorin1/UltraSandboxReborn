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
#define STEPS 240 /* 4 s à 60 Hz */

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

static Roll rollout(Env* env, Nn* nn, const float* theta, unsigned seed, float cmd, bool push) {
    nn_set_params(nn, theta);
    env_reset(env, seed, cmd, push);
    float obs[ENV_OBS], act[ENV_ACT];
    Roll r = { 0.0f, 0.0f, 0.0f, 0.0f };
    float x0 = env_x(env);
    for (int t = 0; t < STEPS; t++) {
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
static float validate(Env* env, Nn* nn, const float* theta) {
    float s = 0.0f;
    for (int c = 0; c < 3; c++)
        s += rollout(env, nn, theta, 4242u + (unsigned)c, (float)(c - 1), false).reward;
    return s / 3.0f;
}

int main(int argc, char** argv) {
    int gens = (argc > 1) ? atoi(argv[1]) : 200;
    const int pop = POP, pairs = POP / 2;
    const unsigned seed = 12345u;

    Nn* nn = nn_create(ENV_OBS, H1, H2, ENV_ACT, seed);
    if (!nn) { fprintf(stderr, "nn_create\n"); return 1; }
    const int np = nn_param_count(nn);
    Es es;
    es_init(&es, np, POP, 0.08f, 0.03f, seed);

    Env* env = env_create(seed);
    float* theta = malloc(sizeof(float) * (size_t)np);
    float* rewards = malloc(sizeof(float) * (size_t)pop);
    if (!env || !theta || !rewards) { fprintf(stderr, "alloc\n"); return 1; }

    unsigned crng = seed ^ 0xabcdu;
    float best_val = -1e9f;
    bool resume = (argc > 2 && strcmp(argv[2], "--resume") == 0);
    if (resume && (load_policy("latest.bin", es.mean, np) || load_policy("policy.bin", es.mean, np))) {
        printf("reprise depuis le dernier checkpoint\n");
        es.lr = 0.015f; /* passe d'affinage : on ne casse pas ce qui marche */
        best_val = validate(env, nn, es.mean);
        printf("validation initiale %+.2f\n", best_val);
    } else {
        resume = false;
    }
    printf("policy: %d params, pop %d, %d genera\n", np, pop, gens);
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
            Roll rp = rollout(env, nn, theta, ep_seed, cmd, push);
            es_sample_fresh(&es, p, -1, theta);
            Roll rm = rollout(env, nn, theta, ep_seed, cmd, push);
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
            save_policy("latest.bin", es.mean, np);
            float v = validate(env, nn, es.mean);
            if (v > best_val) {
                best_val = v;
                save_policy("policy.bin", es.mean, np);
                printf("  -> nouveau meilleur (validation %+.1f)\n", v);
                fflush(stdout);
            }
        }
    }

    if (best_val < -1e8f) save_policy("policy.bin", es.mean, np);
    es_free(&es);
    env_destroy(env);
    nn_destroy(nn);
    free(theta);
    free(rewards);
    printf("saved policy.bin (meilleure validation %+.2f)\n", best_val);
    return 0;
}
