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
TESTS   := test_apollo_mode test_fpga_adv_crc test_fpga_adv_wd test_hang_wd test_jtag_tail test_jtag_scan_tail

.PHONY: all clean
all: $(addprefix run-,$(TESTS))

# apollo_mode test: link the real apollo_mode.c against the test driver.
$(BUILD)/test_apollo_mode: test_apollo_mode.c $(SRC_DIR)/apollo_mode.c | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@

# fpga_adv_wd test: header-only logic, the same file the firmware includes.
$(BUILD)/test_fpga_adv_wd: test_fpga_adv_wd.c $(SRC_DIR)/fpga_adv_wd.h | $(BUILD)
	$(CC) $(CFLAGS) test_fpga_adv_wd.c -o $@

# fpga_adv_crc test: header-only, the same file the firmware includes.
$(BUILD)/test_fpga_adv_crc: test_fpga_adv_crc.c $(SRC_DIR)/fpga_adv_crc.h | $(BUILD)
	$(CC) $(CFLAGS) test_fpga_adv_crc.c -o $@

# hang_wd test: header-only logic, the same file the firmware includes.
$(BUILD)/test_hang_wd: test_hang_wd.c $(SRC_DIR)/hang_wd.h | $(BUILD)
	$(CC) $(CFLAGS) test_hang_wd.c -o $@

# jtag_tail test: header-only logic, the same file the firmware includes.
$(BUILD)/test_jtag_tail: test_jtag_tail.c $(SRC_DIR)/jtag_tail.h | $(BUILD)
	$(CC) $(CFLAGS) test_jtag_tail.c -o $@

# jtag_scan_tail test: the real jtag.c against stub headers (test/stubs) and fake SPI / TAP functions.
$(BUILD)/test_jtag_scan_tail: test_jtag_scan_tail.c $(SRC_DIR)/jtag.c $(SRC_DIR)/jtag_tail.h | $(BUILD)
	$(CC) -std=gnu11 -Wall -Wextra -Werror -Wno-unused-function -Istubs -I$(SRC_DIR) -I$(SRC_DIR)/boards/cynthion_d11 -DJTAG_SRC='"$(CURDIR)/$(SRC_DIR)/jtag.c"' test_jtag_scan_tail.c -o $@

run-%: $(BUILD)/%
	@echo "=== running $* ==="
	@./$<

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
