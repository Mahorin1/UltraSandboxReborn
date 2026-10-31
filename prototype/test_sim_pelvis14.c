#include "sim_pelvis14.h"
#include <math.h>
#include <stdio.h>

static int failures = 0;
#define CHECK(c,m) do { if (!(c)) { printf("FAIL: %s\n", m); failures++; } } while (0)
#define DT 0.01f

static float foot_x(const P14Sim* s, int i) {
    return b2Body_GetPosition(s->ragdoll->body[i == 0 ? P14_FOOT1 : P14_FOOT2]).x;
}

int main(void) {
    P14Sim* s = p14_sim_create();
    CHECK(s != NULL, "sim created");
    P14Controller c;
    p14_controller_init(&c);
    for (int i = 0; i < 240; i++) p14_sim_step(s, &c, DT, P14_STAND, false, 0.0f);
    printf("sim idle: y=%.3f angle=%.3f foot=%d\n", p14_sim_pelvis_y(s), p14_sim_angle(s), p14_sim_foot_contact(s));
    CHECK(p14_sim_pelvis_y(s) > 0.60f, "idle pelvis height");
    CHECK(fabsf(p14_sim_angle(s)) < 0.5f, "idle upright");
    CHECK(p14_sim_foot_contact(s), "idle foot contact");

    /* Marche : on mesure aussi le glissement du pied en appui. La racine est
       cinématique, donc un appui immobile dans le monde serait le seul résultat
       physiquement valable ; tout le reste est du glissement commandé. */
    P14Sim* w = p14_sim_create();
    P14Controller wc;
    p14_controller_init(&wc);
    float px[2] = { foot_x(w, 0), foot_x(w, 1) }, slip = 0.0f;
    int slip_n = 0;
    for (int i = 0; i < 240; i++) {
        p14_sim_step(w, &wc, DT, P14_WALK, false, 0.0f);
        for (int f = 0; f < 2; f++) {
            float x = foot_x(w, f);
            if (w->contact[f] > 0) { slip += fabsf(x - px[f]); slip_n++; }
            px[f] = x;
        }
    }
    float wdx = b2Body_GetPosition(w->ragdoll->body[P14_PELVIS]).x;
    printf("sim walk: dx=%.3f y=%.3f angle=%.3f foot=%d slip=%.3f m/s\n", wdx,
           p14_sim_pelvis_y(w), p14_sim_angle(w), p14_sim_foot_contact(w),
           slip_n > 0 ? slip / (slip_n * DT) : 0.0f);
    CHECK(wdx > 0.15f, "walk displacement");
    CHECK(p14_sim_pelvis_y(w) > 0.60f, "walk height");
    CHECK(p14_sim_foot_contact(w), "walk foot contact");

    P14Sim* r = p14_sim_create();
    P14Controller rc;
    p14_controller_init(&rc);
    float rx0 = b2Body_GetPosition(r->ragdoll->body[P14_PELVIS]).x;
    for (int i = 0; i < 240; i++) p14_sim_step(r, &rc, DT, P14_RUN, false, 0.0f);
    float rdx = b2Body_GetPosition(r->ragdoll->body[P14_PELVIS]).x - rx0;
    printf("sim run: dx=%.3f y=%.3f foot=%d\n", rdx, p14_sim_pelvis_y(r), p14_sim_foot_contact(r));
    CHECK(rdx > wdx, "run faster than walk");
    CHECK(p14_sim_foot_contact(r), "run foot contact");

    P14Sim* q = p14_sim_create();
    P14Controller qc;
    p14_controller_init(&qc);
    for (int i = 0; i < 120; i++) p14_sim_step(q, &qc, DT, P14_STAND, true, 0.0f);
    float qy = p14_sim_pelvis_y(q);
    for (int i = 0; i < 120; i++) p14_sim_step(q, &qc, DT, P14_STAND, false, 0.0f);
    printf("sim crouch: low=%.3f stand=%.3f angle=%.3f\n", qy, p14_sim_pelvis_y(q), p14_sim_angle(q));
    CHECK(qy < 0.65f, "crouch lowers pelvis");
    CHECK(p14_sim_pelvis_y(q) > 0.65f, "crouch returns to stand");

    P14Sim* g = p14_sim_create();
    P14Controller gc;
    p14_controller_init(&gc);
    p14_controller_begin_getup(&gc);
    g->force_hands = g->force_knees = true;
    int done = -1;
    for (int i = 0; i < 220; i++) {
        p14_sim_step(g, &gc, DT, P14_GETUP, false, 0.0f);
        if (done < 0 && gc.getup == P14_DONE) done = i + 1;
    }
    printf("sim getup: state=%d gait=%d y=%.3f foot=%d done_at=%.2fs (appuis mains/genoux forces)\n",
           gc.getup, gc.gait, p14_sim_pelvis_y(g), p14_sim_foot_contact(g),
           done > 0 ? done * DT : -1.0f);
    CHECK(gc.getup == P14_DONE, "getup completes");
    CHECK(gc.gait == P14_STAND, "getup ends standing");
    CHECK(p14_sim_pelvis_y(g) > 0.65f, "getup height");
    CHECK(p14_sim_foot_contact(g), "getup foot contact");

    p14_sim_destroy(s); p14_sim_destroy(w); p14_sim_destroy(r);
    p14_sim_destroy(q); p14_sim_destroy(g);
    printf(failures == 0 ? "PASS\n" : "FAIL\n");
    return failures == 0 ? 0 : 1;
}
