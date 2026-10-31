CC ?= cc
CFLAGS ?= -std=c17 -Wall -Wextra -I.
B2_CFLAGS := $(shell pkg-config --cflags box2d)
B2_LIBS := $(shell pkg-config --libs box2d)
RAY_CFLAGS := $(shell pkg-config --cflags raylib 2>/dev/null)
RAY_LIBS := $(or $(shell pkg-config --libs raylib 2>/dev/null),-lraylib)

HDR := pelvis14.h sim_pelvis14.h

test_pelvis14: pelvis14.c test_pelvis14.c pelvis14.h
	$(CC) $(CFLAGS) $(B2_CFLAGS) pelvis14.c test_pelvis14.c -o $@ $(B2_LIBS) -lm

test_sim_pelvis14: pelvis14.c sim_pelvis14.c test_sim_pelvis14.c $(HDR)
	$(CC) $(CFLAGS) $(B2_CFLAGS) pelvis14.c sim_pelvis14.c test_sim_pelvis14.c -o $@ $(B2_LIBS) -lm

demo_pelvis14: pelvis14.c sim_pelvis14.c demo_pelvis14.c $(HDR)
	$(CC) $(CFLAGS) $(B2_CFLAGS) $(RAY_CFLAGS) pelvis14.c sim_pelvis14.c demo_pelvis14.c -o $@ $(B2_LIBS) $(RAY_LIBS) -lm

test: test_pelvis14 test_sim_pelvis14
	./test_pelvis14
	./test_sim_pelvis14

run: demo_pelvis14
	./demo_pelvis14

clean:
	rm -f test_pelvis14 test_sim_pelvis14 demo_pelvis14

.PHONY: test run clean
