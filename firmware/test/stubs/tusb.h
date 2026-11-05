// Host stub of the TinyUSB surface jtag.c uses: just enough to compile the real file on a host.
#ifndef TEST_STUB_TUSB_H
#define TEST_STUB_TUSB_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
	uint8_t  bmRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} tusb_control_request_t;
bool tud_control_xfer(uint8_t rhport, tusb_control_request_t const *request, void *buffer, uint16_t len);
void tud_task(void);
#endif
