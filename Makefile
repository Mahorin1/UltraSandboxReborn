NAME   := ultrasandbox
CC     ?= cc
PKG_R  := $(or $(shell pkg-config --cflags raylib 2>/dev/null),)
LIB_R  := $(or $(shell pkg-config --libs raylib 2>/dev/null),-lraylib)
PKG_B  := $(shell pkg-config --cflags box2d 2>/dev/null)
LIB_B  := $(or $(shell pkg-config --libs box2d 2>/dev/null),-lbox2d)
WARN   := -Wall -Wextra -Wno-unused-parameter
CFLAGS := -std=c17 $(WARN) -Isrc $(PKG_R) $(PKG_B) -MMD -MP
LDLIBS := $(LIB_R) $(LIB_B) -lm -lpthread

SRC  := $(wildcard src/*.c)
OBJ  := $(SRC:src/%.c=build/%.o)
# sources sans rendu (tests headless : Box2D seulement)
CORE := $(filter-out build/main.o build/render.o,$(OBJ))

.PHONY: all debug release run test clean
all: debug

debug:   CFLAGS += -O0 -g3 -fsanitize=address,undefined
debug:   LDLIBS += -fsanitize=address,undefined
release: CFLAGS += -O3 -DNDEBUG -march=native
debug release: $(NAME)

$(NAME): $(OBJ)
	$(CC) $^ -o $@ $(LDLIBS)

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p build

run: debug
	./$(NAME)

test: $(CORE) tests/headless.c
	$(CC) $(CFLAGS) $^ -o build/headless $(LIB_B) -lm -lpthread
	./build/headless

clean:
	rm -rf build $(NAME)

-include $(OBJ:.o=.d)
