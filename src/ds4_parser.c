#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "mgos_ds4.h"
#include "ds4_internal.h"

static void parse_stick(uint8_t *packet, struct mgos_ds4_3axis *s)
{
  const uint8_t int_offset = 0x80;

  s[0].x =  (uint8_t)packet[DS4_PI_ANALOG + 0] - int_offset;
  s[0].y = -(uint8_t)packet[DS4_PI_ANALOG + 1] + int_offset - 1;
  s[0].z = 0;

  s[1].x =  (uint8_t)packet[DS4_PI_ANALOG + 2] - int_offset;
  s[1].y = -(uint8_t)packet[DS4_PI_ANALOG + 3] + int_offset - 1;
  s[1].z = 0;
}

static void parse_trigger(uint8_t *packet, uint8_t *s)
{
  s[0] = packet[DS4_PI_TRIGGER + 0];
  s[1] = packet[DS4_PI_TRIGGER + 1];
}

static void parse_buttons(uint8_t *packet, struct mgos_ds4_button *s)
{
  uint32_t ds4_buttons_raw = *((uint32_t*)&packet[DS4_PI_BUTTONS]);
  uint8_t arrow_buttons_only = DS4_BM_ARROWS & ds4_buttons_raw;

  s->up        = arrow_buttons_only == DS4_BM_UP;
  s->right     = arrow_buttons_only == DS4_BM_RIGHT;
  s->down      = arrow_buttons_only == DS4_BM_DOWN;
  s->left      = arrow_buttons_only == DS4_BM_LEFT;

  s->upright   = arrow_buttons_only == DS4_BM_UPRIGHT;
  s->upleft    = arrow_buttons_only == DS4_BM_UPLEFT;
  s->downright = arrow_buttons_only == DS4_BM_DOWNRIGHT;
  s->downleft  = arrow_buttons_only == DS4_BM_DOWNLEFT;

  s->options   = (ds4_buttons_raw & DS4_BM_OPTIONS)  > 0;
  s->l3        = (ds4_buttons_raw & DS4_BM_L3)       > 0;
  s->r3        = (ds4_buttons_raw & DS4_BM_R3)       > 0;
  s->share     = (ds4_buttons_raw & DS4_BM_SHARE)    > 0;

  s->l2        = (ds4_buttons_raw & DS4_BM_L2)       > 0;
  s->r2        = (ds4_buttons_raw & DS4_BM_R2)       > 0;
  s->l1        = (ds4_buttons_raw & DS4_BM_L1)       > 0;
  s->r1        = (ds4_buttons_raw & DS4_BM_R1)       > 0;

  s->triangle  = (ds4_buttons_raw & DS4_BM_TRIANGLE) > 0;
  s->circle    = (ds4_buttons_raw & DS4_BM_CIRCLE)   > 0;
  s->cross     = (ds4_buttons_raw & DS4_BM_CROSS)    > 0;
  s->square    = (ds4_buttons_raw & DS4_BM_SQUARE)   > 0;

  s->ps        = (ds4_buttons_raw & DS4_BM_PS)       > 0;
  s->touchpad  = (ds4_buttons_raw & DS4_BM_TOUCHPAD) > 0;
}

static void parse_battery(uint8_t *packet, struct mgos_ds4_battery *s)
{
  bool    cable_state  = packet[DS4_PI_BATTERY] & DS4_SM_CABLE;
  uint8_t battery_data = packet[DS4_PI_BATTERY] & DS4_SM_BATTERY;

  /*
   * https://github.com/torvalds/linux/blob/master/drivers/hid/hid-sony.c#L1152
   * Interpretation of the battery_capacity data depends on the cable state.
   * When no cable is connected (bit4 is 0):
   * - 0:10: percentage in units of 10%.
   * When a cable is plugged in:
   * - 0-10: percentage in units of 10%.
   * - 11: battery is full
   * - 14: not charging due to Voltage or temperature error
   * - 15: charge error
   */
  if (cable_state) {
    if (battery_data < 10) {
      /* Take the mid-point for each battery capacity value,
       * because on the hardware side 0 = 0-9%, 1=10-19%, etc.
       * This matches official platform behavior, which does
       * the same.
       */
      s->capacity = battery_data * 10 + 5;
      s->status = DS4_PS_CHARGING;
    } else if (battery_data == 10) {
      s->capacity = 100;
      s->status = DS4_PS_CHARGING;
    } else if (battery_data == 11) {
      s->capacity = 100;
      s->status = DS4_PS_FULL;
    } else { /* 14, 15 and undefined values */
      s->capacity = 0;
      s->status = DS4_PS_FAILURE;
    }
  } else {
    s->capacity = battery_data < 10 ? battery_data * 10 + 5 : 100;
    s->status = DS4_PS_DISCHARGING;
  }
}

static inline int16_t get_le16(uint8_t *data)
{
  return (data[0] << 0) | (data[1] << 8);
}

static void parse_sensor(uint8_t *packet, uint8_t offset, 
                         struct mgos_ds4_3axis *s)
{
  s->x = get_le16(&packet[offset + 0]);
  s->y = get_le16(&packet[offset + 2]);
  s->z = get_le16(&packet[offset + 4]);
}

static void parse_touchpad(uint8_t *packet, struct mgos_ds4_3axis *s)
{
  uint8_t *rd = packet;
  int offset = DS4_PI_TOUCHPAD;
  int num_touch_data = 0;
	if (rd[offset] > 0 && rd[offset] <= 4)
		num_touch_data = rd[offset];
	else
		num_touch_data = 1;
	offset += 1;

	for (int m = 0; m < num_touch_data; m++) {
		/* Skip past timestamp */
		offset += 1;

		/*
     * https://github.com/torvalds/linux/blob/master/drivers/hid/hid-sony.c#L1204
		 * The first 7 bits of the first byte is a counter and bit 8 is
		 * a touch indicator that is 0 when pressed and 1 when not
		 * pressed.
		 * The next 3 bytes are two 12 bit touch coordinates, X and Y.
		 * The data for the second touch is in the same format and
		 * immediately follows the data for the first.
		 */
		for (int n = 0; n < 2; n++) {
			s[n].x = rd[offset+1] | ((rd[offset+2] & 0xF) << 8);
			s[n].y = ((rd[offset+2] & 0xF0) >> 4) | (rd[offset+3] << 4);
      s[n].z = !(rd[offset] >> 7);

			offset += 4;
		}
	}
}

bool ds4_parse_packet(struct mgos_ds4_state *state, uint8_t *packet)
{
  if(packet[DS4_PI_REPORT_TYPE] != DS4_REPORT_TYPE) {
    return false;
  }
  
  parse_buttons(packet, &state->button);
  parse_stick(packet, state->stick);
  parse_trigger(packet, state->trigger);
  parse_sensor(packet, DS4_PI_GYRO, &state->gyro);
  parse_sensor(packet, DS4_PI_ACCEL, &state->accel);
  parse_battery(packet, &state->battery);
  parse_touchpad(packet, state->touchpad);

  return true;
}

void ds4_print_packet(const uint8_t *d)
{
  char row[DS4_REPORT_BUFFER_SIZE] = {0};
  uint8_t cols = 8;

  memset(row, 0, DS4_REPORT_BUFFER_SIZE);
  sprintf(row, "%s", "\n    ");
  for (int j = 0; j < cols; j++) {
    sprintf(row, "%s _%02d_", row, j);
  }
  printf("%s\n\n", row);
  memset(row, 0, DS4_REPORT_BUFFER_SIZE);

  for (int i = 0; i < DS4_REPORT_BUFFER_SIZE; i += cols) {
    for (int j = 0; j < cols; j++) {
      sprintf(row, "%s 0x%02x", row, d[i+j]);
    }
    printf("%02d |%s\n", i, row);
    memset(row, 0, DS4_REPORT_BUFFER_SIZE);
  }
}
