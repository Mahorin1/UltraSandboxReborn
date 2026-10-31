#include <math.h>
#include <stdio.h>
#include "env.h"

/* Diagnostic : aucune action (pose neutre), on regarde si le ragdoll tient. */
int main(void) {
    Env* env = env_create(3u);
    env_reset(env, 3u, 0.0f, false);
    float act[ENV_ACT] = { 0 };
    printf("step   t     chest_y  upright  feet\n");
    for (int t = 0; t < 120; t++) {
        bool done = false;
        env_step(env, act, &done);
        if (t % 10 == 0 || done)
            printf("%4d  %5.2f   %6.3f   %5.2f   %d%d%s\n", t, (t + 1) / 60.0f, env_chest_y(env),
                   env_upright(env), env_contact(env, 0) > 0, env_contact(env, 1) > 0,
                   done ? "  done" : "");
        if (done) break;
    }
    printf("angles:");
    for (int j = 0; j < ENV_ACT; j++) printf(" %+.2f", env_joint_angle(env, j));
    printf("\n");
    env_destroy(env);
    return 0;
}
