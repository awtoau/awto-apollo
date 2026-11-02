# Host-native unit tests for Apollo firmware logic that has no MCU dependency.
#
# These compile with the host's gcc (NOT the ARM cross-compiler) so the pure-C
# control-plane logic can be exercised directly. Currently covers the JTAG/UART
# mode-exclusion lock (apollo_mode.c). See awtoau/cynthion-workspace#65.
#
# Usage:
#   make          # build and run all tests
#   make clean

SRC_DIR := ../src
CFLAGS  := -std=c11 -Wall -Wextra -Werror -I$(SRC_DIR)

BUILD   := _build
TESTS   := test_apollo_mode test_fpga_adv_wd

.PHONY: all clean
all: $(addprefix run-,$(TESTS))

# apollo_mode test: link the real apollo_mode.c against the test driver.
$(BUILD)/test_apollo_mode: test_apollo_mode.c $(SRC_DIR)/apollo_mode.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@

# fpga_adv_wd test: header-only logic, the same file the firmware includes.
$(BUILD)/test_fpga_adv_wd: test_fpga_adv_wd.c $(SRC_DIR)/fpga_adv_wd.h | $(BUILD)
	$(CC) $(CFLAGS) test_fpga_adv_wd.c -o $@

run-%: $(BUILD)/%
	@echo "=== running $* ==="
	@./$<

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
