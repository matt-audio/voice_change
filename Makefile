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

ifeq ($(PROFILE),1)
CFLAGS += -DVC_UNIFIED_PROFILE -DVC_ROBOT_PROFILE -DVC_MONSTER_PROFILE \
	-DVC_PITCHED_VOICE_PROFILE
endif
ifeq ($(BENCHMARK),1)
CFLAGS += -DVC_UNIFIED_PROFILE
endif

.PHONY: all benchmark clean profile

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

clean:
	rm -f $(TARGET) $(TARGET)_profile $(TARGET)_benchmark
	rm -rf build build_profile build_benchmark build_sanitize
	rm -f src/*.o src/*.d

-include $(DEP)
