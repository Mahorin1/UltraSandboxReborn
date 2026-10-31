/* Tests sans fenêtre : équilibre, marche, course, squat, relevé et drag. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "physics.h"
#include "policy.h"
#include "ragdoll.h"

#define DT (1.0f / 80.0f)

static void run(Ragdoll* r, Physics* ph, RagdollInput* in, int steps) {
    for (int i = 0; i < steps; i++) {
        ragdoll_update(r, DT, 1.0f, in);
        physics_update(ph, DT, 1.0f);
    }
}

static float chest_y(const Ragdoll* r) {
    return m2px(b2Body_GetPosition(r->body[RD_CHEST])).y;
}

static float chest_angle(const Ragdoll* r) {
    return rad2deg(b2Rot_GetAngle(b2Body_GetRotation(r->body[RD_CHEST])));
}

static int test_idle(void) {
    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 845.0f }, 1.0f);
    RagdollInput in = { 0 };
    for (int i = 0; i <= 800; i++) {
        run(r, &ph, &in, 1);
        if (i % 200 == 0)
            printf("idle t=%.2fs chest_y=%.0f angle=%.1f ground=%d active=%d\n",
                   i * DT, chest_y(r), chest_angle(r), r->on_ground, r->active);
    }
    int ok = r->on_ground && r->active && fabsf(chest_angle(r)) < 15.0f && chest_y(r) < 880.0f;
    ragdoll_destroy(r); physics_shutdown(&ph);
    printf("idle: %s\n", ok ? "ok" : "FAIL");
    return ok;
}

static int test_walk(void) {
    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 845.0f }, 1.0f);
    RagdollInput in = { 0 };
    run(r, &ph, &in, 160); /* 2 s pour se poser */
    float x0 = m2px(b2Body_GetPosition(r->body[RD_CHEST])).x;
    in.right = true;
    run(r, &ph, &in, 240); /* 3 s de marche */
    float dx = m2px(b2Body_GetPosition(r->body[RD_CHEST])).x - x0;
    int ok = dx > 60.0f && fabsf(chest_angle(r)) < 20.0f && chest_y(r) < 880.0f;
    printf("walk: dx=%.0fpx y=%.0f angle=%.1f gait=%d %s\n",
           dx, chest_y(r), chest_angle(r), r->gait, ok ? "ok" : "FAIL");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok;
}

static int test_run(void) {
    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 845.0f }, 1.0f);
    RagdollInput in = { 0 };
    run(r, &ph, &in, 160);
    float x0 = m2px(b2Body_GetPosition(r->body[RD_CHEST])).x;
    in.right = true; in.run = true;
    run(r, &ph, &in, 240);
    float dx = m2px(b2Body_GetPosition(r->body[RD_CHEST])).x - x0;
    int ok = dx > 80.0f && fabsf(chest_angle(r)) < 20.0f && chest_y(r) < 880.0f;
    printf("run:  dx=%.0fpx y=%.0f angle=%.1f gait=%d %s\n",
           dx, chest_y(r), chest_angle(r), r->gait, ok ? "ok" : "FAIL");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok;
}

static int test_crouch(void) {
    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 845.0f }, 1.0f);
    RagdollInput in = { 0 };
    run(r, &ph, &in, 160);
    in.crouch = true;
    run(r, &ph, &in, 320);
    float low_y = chest_y(r), low_a = chest_angle(r);
    in.crouch = false;
    run(r, &ph, &in, 160);
    int ok = low_y > 865.0f && fabsf(low_a) < 25.0f &&
             chest_y(r) < 865.0f && fabsf(chest_angle(r)) < 15.0f;
    printf("crouch: low_y=%.0f low_a=%.1f stand_y=%.0f stand_a=%.1f %s\n",
           low_y, low_a, chest_y(r), chest_angle(r), ok ? "ok" : "FAIL");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok;
}

static int test_getup(void) {
    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 845.0f }, 1.0f);
    RagdollInput in = { 0 };
    run(r, &ph, &in, 120);
    ragdoll_set_active(r, false);
    run(r, &ph, &in, 120); /* chute passive : le ragdoll est couché */
    ragdoll_set_active(r, true);
    for (int i = 0; i < 400; i++) {
        run(r, &ph, &in, 1);
    }
    int ok = r->gait == RD_GAIT_STAND && fabsf(chest_angle(r)) < 30.0f && chest_y(r) < 880.0f;
    printf("getup: y=%.0f angle=%.1f gait=%d active=%d %s\n",
           chest_y(r), chest_angle(r), r->gait, r->active, ok ? "ok" : "FAIL");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok;
}

static int test_drag(void) {
    Physics ph; physics_init(&ph, 1);
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = px2m((b2Vec2){ 300.0f, 900.0f });
    b2BodyId box = b2CreateBody(ph.world, &bd);
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Polygon poly = b2MakeBox(0.3f, 0.3f);
    b2CreatePolygonShape(box, &sd, &poly);

    b2Vec2 start = m2px(b2Body_GetPosition(box));
    physics_drag_begin(&ph, start);
    int grabbed = ph.drag.active && b2Body_IsValid(ph.drag.body);
    physics_drag_rotate(&ph, 1, 0.5f);
    int spun = b2Body_GetAngularVelocity(box) > 0.0f;
    b2Body_SetAngularVelocity(box, 0.0f);

    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 845.0f }, 1.0f);
    for (int i = 0; i < 80; i++) {
        physics_drag_update(&ph, (b2Vec2){ start.x + 100.0f, start.y });
        physics_update(&ph, DT, 1.0f);
    }
    float moved = m2px(b2Body_GetPosition(box)).x - start.x;
    physics_drag_end(&ph);
    int ok = grabbed && spun && moved > 40.0f && !ph.drag.active;
    printf("drag: grabbed=%d spun=%d moved=%.0fpx released=%d %s\n",
           grabbed, spun, moved, !ph.drag.active, ok ? "ok" : "FAIL");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok;
}

/* politique : forward borné/déterministe et pilotage sans NaN (poids aléatoires) */
static int test_policy(void) {
    Policy pol;
    memset(&pol, 0, sizeof pol);
    unsigned s = 7u;
    float* fields[6] = { pol.w1, pol.b1, pol.w2, pol.b2, pol.w3, pol.b3 };
    int sizes[6] = { POLICY_OBS * POLICY_H1, POLICY_H1, POLICY_H1 * POLICY_H2,
                     POLICY_H2, POLICY_H2 * POLICY_ACT, POLICY_ACT };
    for (int f = 0; f < 6; f++) {
        for (int i = 0; i < sizes[f]; i++) {
            s = s * 1664525u + 1013904223u;
            fields[f][i] = ((float)(s >> 8) / 16777216.0f - 0.5f) * 0.6f;
        }
    }
    pol.loaded = true;

    float obs[POLICY_OBS], a1[POLICY_ACT], a2[POLICY_ACT];
    for (int i = 0; i < POLICY_OBS; i++) obs[i] = 0.1f * (float)(i % 7) - 0.3f;
    policy_forward(&pol, obs, a1);
    policy_forward(&pol, obs, a2);
    int deterministic = 1, bounded = 1;
    for (int i = 0; i < POLICY_ACT; i++) {
        if (a1[i] != a2[i]) deterministic = 0;
        if (!(a1[i] >= -1.0f && a1[i] <= 1.0f)) bounded = 0;
    }

    Physics ph; physics_init(&ph, 1);
    Ragdoll* r = ragdoll_create(ph.world, (b2Vec2){ 600.0f, 845.0f }, 1.0f);
    ragdoll_set_policy(r, true);
    for (int i = 0; i < 160; i++) {
        ragdoll_policy_update(r, &pol, DT, 0.0f);
        physics_update(&ph, DT, 1.0f);
    }
    b2Vec2 p = m2px(b2Body_GetPosition(r->body[RD_CHEST]));
    int finite = isfinite(p.x) && isfinite(p.y) && r->policy_on;
    int ok = deterministic && bounded && finite;
    printf("policy: deterministe=%d borne=%d fini=%d chest=(%.0f,%.0f) %s\n",
           deterministic, bounded, finite, p.x, p.y, ok ? "ok" : "FAIL");
    ragdoll_destroy(r); physics_shutdown(&ph);
    return ok;
}

int main(void) {
    int ok = 1;
    ok &= test_idle();
    ok &= test_walk();
    ok &= test_run();
    ok &= test_crouch();
    ok &= test_getup();
    ok &= test_drag();
    ok &= test_policy();
    printf(ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
