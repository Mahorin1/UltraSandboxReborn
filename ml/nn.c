#include "nn.h"
#include <math.h>
#include <stdlib.h>

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

static float* alloc_layer(int n, unsigned* seed, float scale) {
    float* w = malloc(sizeof(float) * (size_t)n);
    if (!w) abort();
    for (int i = 0; i < n; i++) w[i] = grand(seed) * scale;
    return w;
}

Nn* nn_create(int n_in, int n_h1, int n_h2, int n_out, unsigned seed) {
    if (n_h1 > NN_MAX_H || n_h2 > NN_MAX_H) return NULL;
    Nn* n = calloc(1, sizeof *n);
    if (!n) return NULL;
    n->n_in = n_in; n->n_h1 = n_h1; n->n_h2 = n_h2; n->n_out = n_out;
    n->w1 = alloc_layer(n_in * n_h1, &seed, sqrtf(2.0f / (float)n_in));
    n->b1 = calloc((size_t)n_h1, sizeof(float));
    n->w2 = alloc_layer(n_h1 * n_h2, &seed, sqrtf(2.0f / (float)n_h1));
    n->b2 = calloc((size_t)n_h2, sizeof(float));
    /* couche de sortie petite : la politique démarre proche de la pose neutre */
    n->w3 = alloc_layer(n_h2 * n_out, &seed, 0.1f);
    n->b3 = calloc((size_t)n_out, sizeof(float));
    return n;
}

void nn_destroy(Nn* n) {
    if (!n) return;
    free(n->w1); free(n->b1); free(n->w2); free(n->b2); free(n->w3); free(n->b3);
    free(n);
}

int nn_param_count(const Nn* n) {
    return n->n_in * n->n_h1 + n->n_h1 + n->n_h1 * n->n_h2 + n->n_h2 + n->n_h2 * n->n_out + n->n_out;
}

void nn_get_params(const Nn* n, float* out) {
    int k = 0;
    for (int i = 0; i < n->n_in * n->n_h1; i++) out[k++] = n->w1[i];
    for (int i = 0; i < n->n_h1; i++) out[k++] = n->b1[i];
    for (int i = 0; i < n->n_h1 * n->n_h2; i++) out[k++] = n->w2[i];
    for (int i = 0; i < n->n_h2; i++) out[k++] = n->b2[i];
    for (int i = 0; i < n->n_h2 * n->n_out; i++) out[k++] = n->w3[i];
    for (int i = 0; i < n->n_out; i++) out[k++] = n->b3[i];
}

void nn_set_params(Nn* n, const float* in) {
    int k = 0;
    for (int i = 0; i < n->n_in * n->n_h1; i++) n->w1[i] = in[k++];
    for (int i = 0; i < n->n_h1; i++) n->b1[i] = in[k++];
    for (int i = 0; i < n->n_h1 * n->n_h2; i++) n->w2[i] = in[k++];
    for (int i = 0; i < n->n_h2; i++) n->b2[i] = in[k++];
    for (int i = 0; i < n->n_h2 * n->n_out; i++) n->w3[i] = in[k++];
    for (int i = 0; i < n->n_out; i++) n->b3[i] = in[k++];
}

void nn_forward(const Nn* n, const float* x, float* y) {
    float a[NN_MAX_H], b[NN_MAX_H];
    for (int i = 0; i < n->n_h1; i++) {
        float s = n->b1[i];
        for (int j = 0; j < n->n_in; j++) s += n->w1[i * n->n_in + j] * x[j];
        a[i] = tanhf(s);
    }
    for (int i = 0; i < n->n_h2; i++) {
        float s = n->b2[i];
        for (int j = 0; j < n->n_h1; j++) s += n->w2[i * n->n_h1 + j] * a[j];
        b[i] = tanhf(s);
    }
    for (int i = 0; i < n->n_out; i++) {
        float s = n->b3[i];
        for (int j = 0; j < n->n_h2; j++) s += n->w3[i * n->n_h2 + j] * b[j];
        y[i] = tanhf(s);
    }
}
