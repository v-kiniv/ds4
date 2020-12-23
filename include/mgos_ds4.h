#ifndef _MGOS_DS4_H
#define _MGOS_DS4_H

#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include "mgos_event.h"
#include "common/mg_str.h"

#define MGOS_DS4_BASE MGOS_EVENT_BASE('D', 'S', '4')

enum mgos_ds4_event {
  MGOS_DS4_CONNECTION = MGOS_DS4_BASE, // ev_data: struct mgos_ds4_connection_arg
  MGOS_DS4_STARTED,                    // ev_data: NULL
  MGOS_DS4_PAIRED,                     // ev_data: struct mgos_ds4_paired_arg
  MGOS_DS4_INPUT,                      // ev_data: struct mgos_ds4_input_arg
  MGOS_DS4_DISCOVERY_STARTED,          // ev_data: NULL
  MGOS_DS4_DISCOVERY_STOPPED,          // ev_data: NULL
};

struct mgos_ds4_button {
  uint8_t options  : 1;
  uint8_t l3       : 1;
  uint8_t r3       : 1;
  uint8_t share    : 1;

  uint8_t up       : 1;
  uint8_t right    : 1;
  uint8_t down     : 1;
  uint8_t left     : 1;

  uint8_t upright  : 1;
  uint8_t upleft   : 1;
  uint8_t downright: 1;
  uint8_t downleft : 1;

  uint8_t l2       : 1;
  uint8_t r2       : 1;
  uint8_t l1       : 1;
  uint8_t r1       : 1;

  uint8_t triangle : 1;
  uint8_t circle   : 1;
  uint8_t cross    : 1;
  uint8_t square   : 1;

  uint8_t ps       : 1;
  uint8_t touchpad : 1;
};

enum mgos_ds4_ps_status {
  DS4_PS_DISCHARGING = 0,
  DS4_PS_CHARGING,
  DS4_PS_FULL,

  /* Voltage, temperature or other battery error */
  DS4_PS_FAILURE,
};

struct mgos_ds4_battery {
  /* Battery capacity percentage, ranges from 0-100% */
  uint8_t capacity;

  /* Power supply status */
  enum mgos_ds4_ps_status status;
};

struct mgos_ds4_3axis {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct mgos_ds4_state {
  /* Left and right analog stick.
   * Each axis ranges from -128 to 128. */
  struct mgos_ds4_3axis  stick[2];

  /* Touchpad.
   * Each element of the array represents a finger with coords and state.
   * X axis ranges from 0 to 1920
   * Y axis ranges from 0 to 942
   * Z indicates tracking state, 1 - active, 0 non-active */
  struct mgos_ds4_3axis  touchpad[2];

  /* Accelerometer sensor.
   * Each axis ranges from -8192 to 8192. */
  struct mgos_ds4_3axis  accel;

  /* Gyroscope sensor.
   * Each axis ranges from -8192 to 8192.
   * X - pitch.
   * Y - yaw.
   * Z - roll. */
  struct mgos_ds4_3axis  gyro;

  /* Left and right analog trigger buttons.
   * Each button's value ranges from 0 to 255. */
  uint8_t                trigger[2];
  
  /* Digital buttons, see struct mgos_ds4_button for details. */
  struct mgos_ds4_button button;

  /* Device status, see struct mgos_ds4_battery for details. */
  struct mgos_ds4_battery battery;
};

struct mgos_ds4_connection_arg {
  uint8_t       index;
  bool          connected;
  struct mg_str device_address;
};

struct mgos_ds4_paired_arg {
  struct mg_str device_address;
};

struct mgos_ds4_input_arg {
  uint8_t                index;
  struct mgos_ds4_state  state;
  struct mgos_ds4_button button_down;
  struct mgos_ds4_button button_up;
  bool                   stick_move[2];
  bool                   trigger_move[2];
  bool                   gyro_move;
  bool                   accel_move;
};

/* Start ds4 server.
 * Called automatically if ds4.enable configuration set to true. */
bool mgos_ds4_start(void);

/* Stop ds4 server. */
bool mgos_ds4_stop(void);

/* Start device discovery and connect first found device */
bool mgos_ds4_discover_and_pair(void);

/* Cancel device discovery */
bool mgos_ds4_cancel_discovery(void);

/* Returns true if controller with specified index is connected. */
bool mgos_ds4_is_connected(int controller_index);

/* Returns true if discovery is in progress. */
bool mgos_ds4_is_discovering(void);

/* Disconnect controller with specified index */
bool mgos_ds4_disconnect(int controller_index);

/* Set RGB value for the controller's LED.
 * Each color component ranges from 0 to 255.
 * Change is applied asynchronously so other commands can be called immediately.
 */
bool mgos_ds4_led_set(int controller_index, int r, int g, int b);

/* Set blink function of the controller's LED.
 * Delays specified in milliseconds and ranges from 0 to 2550.
 * Change is applied asynchronously so other commands can be called immediately.
 */
bool mgos_ds4_led_blink_set(int controller_index, int delay_on_ms, 
                            int delay_off_ms);

/* Helper method to turn off blink function of the LED after specified timeout.
 * Timeout is specified in milliseconds and ranges from 0 to 65534.
 * Change is applied asynchronously so other commands can be called immediately.
 */
bool mgos_ds4_led_blink_timeout(int controller_index, int timeout_ms);

/* Set controller's small and large rumbler.
 * Each value ranges from 0 to 255.
 * Change is applied asynchronously so other commands can be called immediately.
 */
bool mgos_ds4_rumble_set(int controller_index, int small, int large);

/* Helper method to turn off the rumblers after specified timeout.
 * Timeout is specified in milliseconds and ranges from 0 to 65536.
 * Change is applied asynchronously so other commands can be called immediately.
 */
bool mgos_ds4_rumble_timeout(int controller_index, int timeout_ms);

/* Get string representation of the controller's MAC address. */
struct mg_str mgos_ds4_get_controller_address(int controller_index);

/* Flush control buffer immediately */
bool mgos_ds4_flush(int controller_index);

#endif // _MGOS_DS4_H
