CC ?= cc
CFLAGS ?= -std=c17 -O2 -g -Wall -Wextra -I.
B2_CFLAGS := $(shell pkg-config --cflags box2d)
B2_LIBS := $(shell pkg-config --libs box2d)
SRC := rd_old.c env.c nn.c es.c
HDR := rd_old.h env.h nn.h es.h

all: train eval probe

train: train.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(B2_CFLAGS) train.c $(SRC) -o $@ $(B2_LIBS) -lm

eval: eval.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(B2_CFLAGS) eval.c $(SRC) -o $@ $(B2_LIBS) -lm

probe: probe.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(B2_CFLAGS) probe.c $(SRC) -o $@ $(B2_LIBS) -lm

clean:
	rm -f train eval probe

.PHONY: all clean
