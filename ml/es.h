#pragma once

/* Évolution stratégique (OpenAI-ES) : pas de rétropropagation, seulement des
   tirages ± bruit gaussiens et une montée de gradient sur la moyenne. */

typedef struct {
    int n, pop;          /* pop doit être pair */
    float sigma, lr;
    float *mean;         /* n */
    float *eps;          /* (pop/2) * n */
    float *m, *v;        /* Adam, n */
    int t;
    unsigned rng;
} Es;

void es_init(Es* e, int n, int pop, float sigma, float lr, unsigned seed);
void es_free(Es* e);

/* theta = moyenne ± sigma * eps[pair] ; sign = +1 ou -1 */
void es_sample(const Es* e, int pair, int sign, float* theta);

/* comme es_sample, mais tire un nouveau bruit pour sign = +1
   (appeler +1 puis -1 pour la même paire) */
void es_sample_fresh(Es* e, int pair, int sign, float* theta);

/* rewards[0..pop) : reward des échantillons dans l'ordre des pairs
   (2*p = +, 2*p+1 = -). Met à jour la moyenne (Adam, rangs centrés). */
void es_train(Es* e, const float* rewards);
