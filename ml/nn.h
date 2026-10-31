#pragma once

#define NN_MAX_H 64

typedef struct {
    int n_in, n_h1, n_h2, n_out;
    float *w1, *b1, *w2, *b2, *w3, *b3;
} Nn;

Nn*  nn_create(int n_in, int n_h1, int n_h2, int n_out, unsigned seed);
void nn_destroy(Nn* n);
int  nn_param_count(const Nn* n);
void nn_get_params(const Nn* n, float* out);
void nn_set_params(Nn* n, const float* in);
void nn_forward(const Nn* n, const float* x, float* y);
