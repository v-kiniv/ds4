#ifndef _DS4_INTERNAL_H
#define _DS4_INTERNAL_H

#include "mgos_ds4.h"

// Max supported controllers number
#define DS4_MAX_CTRLS 2

// Size of the report buffer(input)
#define DS4_REPORT_BUFFER_SIZE 79

// Size of the control buffer(output)
#define DS4_CONTROL_BUFFER_SIZE 77

#define DS4_REPORT_TYPE 0x11

enum ds4_hid_cmd_code {
  DS4_HID_CC_SET_REPORT   = 0x50,
  DS4_HID_CC_TYPE_OUTPUT  = 0x02,
  DS4_HID_CC_TYPE_FEATURE = 0x03
};

struct ds4_hid_cmd {
  uint8_t code;
  uint8_t identifier;
  uint8_t data[DS4_CONTROL_BUFFER_SIZE];
};

struct ds4_config {
  uint8_t rumble_small;
  uint8_t rumble_large;
  uint8_t r, g, b;
  uint8_t blink_on;
  uint8_t blink_off;
};

enum ds4_control_packet_index {
  DS4_CPI_RUMBLE_SMALL  = 5,
  DS4_CPI_RUMBLE_LARGE  = 6,

  DS4_CPI_LED_RED       = 7,
  DS4_CPI_LED_GREEN     = 8,
  DS4_CPI_LED_BLUE      = 9,

  DS4_CPI_LED_BLINK_ON  = 10,
  DS4_CPI_LED_BLINK_OFF = 11,
};

enum ds4_packet_index {
  DS4_PI_REPORT_TYPE = 10,
  DS4_PI_ANALOG      = 13,
  DS4_PI_BUTTONS     = 17,
  DS4_PI_TRIGGER     = 20,
  DS4_PI_GYRO        = 25,
  DS4_PI_ACCEL       = 31,
  DS4_PI_BATTERY     = 42,
  DS4_PI_TOUCHPAD    = 45
};

enum ds4_button_mask {
  DS4_BM_UP        = 0,
  DS4_BM_RIGHT     = 1 << 1,
  DS4_BM_DOWN      = 1 << 2,
  DS4_BM_LEFT      = (1 << 1) + (1 << 2),

  DS4_BM_UPRIGHT   = 1,
  DS4_BM_UPLEFT    = 1 + (1 << 1) + (1 << 2),
  DS4_BM_DOWNRIGHT = 1 + (1 << 1),
  DS4_BM_DOWNLEFT  = 1 + (1 << 2),

  DS4_BM_ARROWS    = 0xf,

  DS4_BM_SQUARE    = 1 << 4,
  DS4_BM_CROSS     = 1 << 5,
  DS4_BM_CIRCLE    = 1 << 6,
  DS4_BM_TRIANGLE  = 1 << 7,

  DS4_BM_L1        = 1 << 8,
  DS4_BM_R1        = 1 << 9,
	DS4_BM_L2        = 1 << 10,
  DS4_BM_R2        = 1 << 11,

  DS4_BM_SHARE     = 1 << 12,
  DS4_BM_OPTIONS   = 1 << 13,

  DS4_BM_L3        = 1 << 14,
  DS4_BM_R3        = 1 << 15,

  DS4_BM_PS        = 1 << 16,
  DS4_BM_TOUCHPAD  = 1 << 17
};

enum ds4_status_mask {
  DS4_SM_BATTERY = 0xf,
  DS4_SM_CABLE   = 1 << 4,
};

bool ds4_parse_packet(struct mgos_ds4_state *state, uint8_t *packet);
void ds4_print_packet(const uint8_t *d);
void ds4_gen_event(struct mgos_ds4_input_arg *event, 
                   struct mgos_ds4_state *prev, struct mgos_ds4_state *next);

#endif // _DS4_INTERNAL_H
