CC ?= cc
CFLAGS ?= -O3 -std=c11 -Wall -Wextra -Wpedantic
CFLAGS += -MMD -MP
LDFLAGS ?= -lm
TARGET ?= voice_fx
OUTPUT_TARGET = $(TARGET)$(if $(filter 1,$(PROFILE)),_profile,$(if $(filter 1,$(BENCHMARK)),_benchmark,))

SRC = src/main.c src/dsp.c src/robot.c src/monster.c src/pitched_voice.c \
	src/male.c src/female.c src/donald.c src/wav.c
BUILD_DIR = build$(if $(filter 1,$(PROFILE)),_profile,$(if $(filter 1,$(BENCHMARK)),_benchmark,))
OBJ = $(SRC:src/%.c=$(BUILD_DIR)/%.o)
DEP = $(OBJ:.o=.d)
FULL_RATE_TEST = build/test_full_rate
SANITIZE_TEST = build_sanitize/test_full_rate
SANITIZER_FLAGS = -O1 -g -fsanitize=address,undefined \
	-fno-omit-frame-pointer

ifeq ($(PROFILE),1)
CFLAGS += -DVC_UNIFIED_PROFILE -DVC_ROBOT_PROFILE -DVC_MONSTER_PROFILE \
	-DVC_PITCHED_VOICE_PROFILE
endif
ifeq ($(BENCHMARK),1)
CFLAGS += -DVC_UNIFIED_PROFILE
endif

.PHONY: all benchmark clean dependency-check profile sanitize-test test

all: $(OUTPUT_TARGET)

$(OUTPUT_TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LDFLAGS)

$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -Isrc -c $< -o $@

profile:
	$(MAKE) PROFILE=1

benchmark:
	$(MAKE) BENCHMARK=1

$(FULL_RATE_TEST): tests/full_rate.c src/pitched_voice.c src/pitched_voice.h \
		src/dsp.c src/dsp.h
	@mkdir -p build
	$(CC) $(CFLAGS) -Isrc -o $@ \
		tests/full_rate.c src/pitched_voice.c src/dsp.c \
		$(LDFLAGS)

$(SANITIZE_TEST): tests/full_rate.c src/pitched_voice.c src/pitched_voice.h \
		src/dsp.c src/dsp.h
	@mkdir -p build_sanitize
	$(CC) $(CFLAGS) $(SANITIZER_FLAGS) -Isrc -o $@ \
		tests/full_rate.c src/pitched_voice.c src/dsp.c \
		$(LDFLAGS) $(SANITIZER_FLAGS)

sanitize-test: $(SANITIZE_TEST)
	ASAN_OPTIONS=detect_leaks=0 ./$(SANITIZE_TEST)

dependency-check: $(TARGET) tests/check_dependencies.sh
	sh tests/check_dependencies.sh ./$(TARGET)

test: $(TARGET) $(FULL_RATE_TEST) dependency-check
	python3 tests/regression.py ./$(TARGET)
	./$(FULL_RATE_TEST)

clean:
	rm -f $(TARGET) $(TARGET)_profile $(TARGET)_benchmark
	rm -rf build build_profile build_benchmark build_sanitize
	rm -f src/*.o src/*.d

-include $(DEP)
