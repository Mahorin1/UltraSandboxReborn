#include "pelvis14.h"
#include <math.h>
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
} while (0)

static int test_model(void) {
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld(&wd);
    P14Ragdoll* r = p14_create(world);
    CHECK(r != NULL, "model created");
    if (!r) return 1;
    int bodies = 0, joints = 0;
    for (int i = 0; i < P14_PARTS; i++) if (b2Body_IsValid(r->body[i])) bodies++;
    for (int i = 0; i < P14_JOINTS; i++) if (b2Joint_IsValid(r->joint[i])) joints++;
    CHECK(bodies == 14, "14 valid bodies");
    CHECK(joints == 13, "13 valid joints");
    p14_destroy(r);
    b2DestroyWorld(world);
    printf("model: 14 bodies / 13 joints\n");
    return 0;
}

static int test_leg_ik(void) {
    P14Vec2 hip = { 0.0f, 0.80f };
    P14Vec2 target = { 0.10f, 0.20f };
    P14LegSolution s = p14_leg_ik(hip, target, 0.32f, 0.30f, 1);
    CHECK(s.reachable, "reachable IK target");
    CHECK(isfinite(s.hip_angle) && isfinite(s.knee_angle), "finite IK angles");
    CHECK(fabsf(s.foot.x - target.x) < 2e-3f, "IK reconstructs x");
    CHECK(fabsf(s.foot.y - target.y) < 2e-3f, "IK reconstructs y");

    P14LegSolution other = p14_leg_ik(hip, target, 0.32f, 0.30f, -1);
    CHECK(other.reachable, "other bend reachable");
    CHECK(fabsf(other.foot.x - target.x) < 2e-3f, "other bend x");
    CHECK(fabsf(other.foot.y - target.y) < 2e-3f, "other bend y");

    P14LegSolution far = p14_leg_ik(hip, (P14Vec2){ 3.0f, 0.2f }, 0.32f, 0.30f, 1);
    CHECK(!far.reachable, "unreachable target flagged");
    CHECK(isfinite(far.foot.x) && isfinite(far.foot.y), "unreachable output finite");
    printf("leg_ik: reconstruction + clamp ok\n");
    return 0;
}

static int test_gait_controller(void) {
    P14Controller c;
    p14_controller_init(&c);
    CHECK(c.gait == P14_STAND, "init stand");
    CHECK(c.leg[0].reachable && c.leg[1].reachable, "init legs reachable");

    p14_controller_update(&c, 0.10f, P14_WALK, false, 0.0f);
    float walk0 = c.foot[0].x;
    float walk1 = c.foot[1].x;
    CHECK(c.gait == P14_WALK, "walk gait");
    CHECK(walk0 > -0.12f && walk1 < 0.12f, "walk alternates feet");
    CHECK(c.leg[0].reachable && c.leg[1].reachable, "walk IK reachable");

    P14Controller r;
    p14_controller_init(&r);
    p14_controller_update(&r, 0.10f, P14_RUN, false, 0.0f);
    float run0 = r.foot[0].x;
    CHECK(r.gait == P14_RUN, "run gait");
    CHECK(fabsf(run0 + 0.12f) > fabsf(walk0 + 0.12f), "run stride larger than walk");
    CHECK(r.leg[0].reachable && r.leg[1].reachable, "run IK reachable");

    P14Controller q;
    p14_controller_init(&q);
    p14_controller_update(&q, 0.10f, P14_STAND, true, 0.0f);
    CHECK(q.gait == P14_CROUCH, "crouch gait");
    CHECK(q.hip[0].y < c.hip[0].y, "crouch lowers pelvis");
    CHECK(q.leg[0].reachable && q.leg[1].reachable, "crouch IK reachable");

    p14_controller_update(&c, 0.0f, P14_STAND, false, 0.2f);
    CHECK(c.balance < 0.0f, "positive lean gives negative correction");
    p14_controller_update(&c, 0.0f, P14_STAND, false, -0.2f);
    CHECK(c.balance > 0.0f, "negative lean gives positive correction");
    printf("gait: stand/walk/run/crouch + balance ok\n");
    return 0;
}

static int test_getup_state_machine(void) {
    P14Controller c;
    p14_controller_init(&c);
    p14_controller_begin_getup(&c);
    CHECK(c.getup == P14_ROLL, "getup starts at roll");
    p14_controller_update(&c, 0.30f, P14_GETUP, false, 0.0f);
    CHECK(c.getup == P14_PUSH, "roll -> push");
    p14_controller_update(&c, 0.30f, P14_GETUP, false, 0.0f);
    CHECK(c.getup == P14_PUSH, "push waits for hand contact");
    p14_controller_set_contacts(&c, true, false);
    p14_controller_update(&c, 0.30f, P14_GETUP, false, 0.0f);
    CHECK(c.getup == P14_KNEEL, "push -> kneel");
    p14_controller_set_contacts(&c, true, true);
    p14_controller_update(&c, 0.30f, P14_GETUP, false, 0.0f);
    CHECK(c.getup == P14_EXTEND, "kneel -> extend");
    p14_controller_update(&c, 0.30f, P14_GETUP, false, 0.0f);
    CHECK(c.getup == P14_DONE, "extend -> done");
    CHECK(c.gait == P14_STAND, "done -> stand");
    CHECK(c.leg[0].reachable && c.leg[1].reachable, "getup IK reachable");
    printf("getup: contact-gated state machine ok\n");
    return 0;
}

int main(void) {
    test_model();
    test_leg_ik();
    test_gait_controller();
    test_getup_state_machine();
    printf(failures == 0 ? "PASS\n" : "FAIL\n");
    return failures == 0 ? 0 : 1;
}
