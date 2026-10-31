#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "env.h"
#include "nn.h"

#define H1 32
#define H2 32
#define STEPS 240 /* 4 s */

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

static void run(Env* env, const Nn* nn, const float* theta, float cmd, unsigned seed,
                float* dx, float* up, float* steps) {
    nn_set_params((Nn*)nn, theta);
    env_reset(env, seed, cmd, false);
    float obs[ENV_OBS], act[ENV_ACT];
    float x0 = env_x(env), u = 0.0f;
    int n = 0;
    for (int t = 0; t < STEPS; t++) {
        env_obs(env, obs);
        nn_forward(nn, obs, act);
        bool done = false;
        env_step(env, act, &done);
        u += env_upright(env);
        n++;
        if (done) break;
    }
    *dx = env_x(env) - x0;
    *up = u / (float)n;
    *steps = (float)n;
}

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "policy.bin";
    Nn* nn = nn_create(ENV_OBS, H1, H2, ENV_ACT, 1u);
    const int np = nn_param_count(nn);
    float* theta = malloc(sizeof(float) * (size_t)np);
    if (!theta || !load_policy(path, theta, np)) return 1;

    Env* env = env_create(7u);
    const float cmds[3] = { -1.0f, 0.0f, 1.0f };
    float mean_dx[3] = { 0 }, mean_up[3] = { 0 }, mean_steps[3] = { 0 };

    printf("cmd      dx(m)   upright  steps\n");
    for (int c = 0; c < 3; c++) {
        for (unsigned s = 1; s <= 3; s++) {
            float dx, up, st;
            run(env, nn, theta, cmds[c], s, &dx, &up, &st);
            mean_dx[c] += dx / 3.0f;
            mean_up[c] += up / 3.0f;
            mean_steps[c] += st / 3.0f;
        }
        printf("%+4.0f  %8.3f   %5.2f   %5.0f\n", cmds[c], mean_dx[c], mean_up[c], mean_steps[c]);
    }

    bool left_ok = mean_dx[0] < -0.03f;
    bool right_ok = mean_dx[2] > 0.03f;
    bool stay_ok = fabsf(mean_dx[1]) < 0.10f;
    bool up_ok = mean_up[1] > 0.70f;
    printf("\n%s: gauche %s, droite %s, immobile %s, equilibre %s\n",
           (left_ok && right_ok && stay_ok && up_ok) ? "PASS" : "FAIL",
           left_ok ? "ok" : "ko", right_ok ? "ok" : "ko", stay_ok ? "ok" : "ko",
           up_ok ? "ok" : "ko");

    env_destroy(env);
    nn_destroy(nn);
    free(theta);
    return (left_ok && right_ok && stay_ok && up_ok) ? 0 : 2;
}
