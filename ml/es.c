#include "es.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static float frand(unsigned* s) {
    unsigned x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}

static float grand(unsigned* s) {
    float u1 = frand(s) + 1e-7f, u2 = frand(s);
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

void es_init(Es* e, int n, int pop, float sigma, float lr, unsigned seed) {
    memset(e, 0, sizeof *e);
    e->n = n; e->pop = pop; e->sigma = sigma; e->lr = lr; e->rng = seed | 1u;
    e->mean = calloc((size_t)n, sizeof(float));
    e->eps = calloc((size_t)(pop / 2) * (size_t)n, sizeof(float));
    e->m = calloc((size_t)n, sizeof(float));
    e->v = calloc((size_t)n, sizeof(float));
    if (!e->mean || !e->eps || !e->m || !e->v) abort();
}

void es_free(Es* e) {
    free(e->mean); free(e->eps); free(e->m); free(e->v);
    memset(e, 0, sizeof *e);
}

void es_sample(const Es* e, int pair, int sign, float* theta) {
    const float* eps = e->eps + (size_t)pair * (size_t)e->n;
    for (int i = 0; i < e->n; i++)
        theta[i] = e->mean[i] + (float)sign * e->sigma * eps[i];
}

void es_train(Es* e, const float* rewards) {
    const int pop = e->pop, pairs = pop / 2;
    int order[256];
    if (pop > 256) return;
    for (int i = 0; i < pop; i++) order[i] = i;
    /* tri par insertion (pop petit) : reward croissant */
    for (int i = 1; i < pop; i++) {
        int k = order[i], j = i - 1;
        while (j >= 0 && rewards[order[j]] > rewards[k]) { order[j + 1] = order[j]; j--; }
        order[j + 1] = k;
    }
    /* rangs centrés : le meilleur échantillon a la plus grande utilité */
    float util[256];
    for (int rank = 0; rank < pop; rank++) {
        int idx = order[rank];
        util[idx] = (float)rank / (float)(pop - 1) - 0.5f;
    }

    /* gradient : somme des (util+ - util-) * eps, normalisée */
    float* g = calloc((size_t)e->n, sizeof(float));
    if (!g) return;
    for (int p = 0; p < pairs; p++) {
        float du = util[2 * p] - util[2 * p + 1];
        const float* eps = e->eps + (size_t)p * (size_t)e->n;
        for (int i = 0; i < e->n; i++) g[i] += du * eps[i];
    }
    float scale = 1.0f / ((float)pop * e->sigma);
    e->t++;
    const float b1 = 0.9f, b2 = 0.999f, eps_adam = 1e-8f;
    float c1 = 1.0f - powf(b1, (float)e->t);
    float c2 = 1.0f - powf(b2, (float)e->t);
    for (int i = 0; i < e->n; i++) {
        float gi = g[i] * scale;
        e->m[i] = b1 * e->m[i] + (1.0f - b1) * gi;
        e->v[i] = b2 * e->v[i] + (1.0f - b2) * gi * gi;
        float mh = e->m[i] / c1, vh = e->v[i] / c2;
        e->mean[i] += e->lr * mh / (sqrtf(vh) + eps_adam);
    }
    free(g);
}

/* remplit eps[pair] ; appelé une fois par paire avant les deux tirages */
static void es_draw_noise(Es* e, int pair) {
    float* eps = e->eps + (size_t)pair * (size_t)e->n;
    for (int i = 0; i < e->n; i++) eps[i] = grand(&e->rng);
}

/* variante utilisée par train.c : tire le bruit puis renvoie le theta demandé */
void es_sample_fresh(Es* e, int pair, int sign, float* theta) {
    if (sign > 0) es_draw_noise(e, pair);
    es_sample(e, pair, sign, theta);
}
