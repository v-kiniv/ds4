# DualShock 4 driver for Mongoose OS

<img align="right" width="60%" src="https://upload.wikimedia.org/wikipedia/commons/6/68/Dualshock_4_Layout_2.svg">

* Buttons, Sticks, Analog Triggers
* Accelerometer
* Gyroscope
* Touchpad
* LED
* Haptic Feedback
* Battery State
* Pairing
* Up to 2 controllers
- Limitations:
  - Compatible with ESP32 only
  - WIFI is blocked during bluetooth transmission

## Setup
Update your `mos.yml`:
```yml
#...
config_schema:
  - ["ds4.enable", true]

libs:
  - origin: https://github.com/v-kiniv/ds4
#...
```

If you want to start DS4 bluetooth service manually, leave `ds4.enable` 
as-is(`false` by default), and call `mgos_ds4_start()` and `mgos_ds4_stop()` 
accordingly.


## Example
Check out full source code of the example project at: https://github.com/v-kiniv/ds4-example
```c
// ...
static void ds4_input_cb(int ev, void *evd, void *userdata)
{
  bool res = false;
  struct mgos_ds4_input_arg *event  = (struct mgos_ds4_input_arg *) evd;
  struct mgos_ds4_3axis     *s      = NULL;
  
  // Share button: print battery status
  if(event->button_down.share) {
    LOG(LL_INFO, ("Controller #%u battery: %d%%(%s)", 
                  event->index,
                  event->state.battery.capacity,
                  event->state.battery.status == DS4_PS_CHARGING ? "charging" : "discharging"
    ));
  }

  // PS button: disconnect controller
  if(event->button_down.ps) {
    mgos_ds4_led_set(event->index, 0, 0, 0);
    res = mgos_ds4_disconnect(event->index);
    LOG(LL_INFO, ("Disconnect controller #%u: %s", 
                  event->index, res ? "success" : "fail"));
  }
  
  // Cross button: button_down event
  if(event->button_down.cross) {
    LOG(LL_INFO, ("X DOWN"));
  }
  if(event->button_up.cross) {
    LOG(LL_INFO, ("X UP"));
  }

  // Sticks: stick_move event update LED color and brightness
  if(event->stick_move[0] || event->stick_move[1])
  {
    uint8_t l = 255 - (127 - event->state.stick[1].y);
    uint8_t r =       (127 - event->state.stick[0].y) * l / 255;
    uint8_t g =       (127 - event->state.stick[0].x) * l / 255;
    uint8_t b =       (127 - event->state.stick[1].x) * l / 255;

    mgos_ds4_led_set(event->index, r, g, b);
  }

  // Triggers: trigger_move event set rumble and LED blink
  if(event->trigger_move[0] || event->trigger_move[1]) {
    mgos_ds4_rumble_set(event->index, event->state.trigger[1],
                        event->state.trigger[0]);
  }

  // Hold Circle: print gyroscope data
  if(event->state.button.circle && event->gyro_move) {
    s = &event->state.gyro;
    LOG(LL_INFO, ("Gyroscope PITCH: %05d, YAW: %05d, ROLL: %05d", 
                  s->x, s->y, s->z));
  }
  // Hold Triangle: print accelerometer data
  if(event->state.button.triangle && event->accel_move) {
    s = &event->state.accel;
    LOG(LL_INFO, ("Accelerometer X: %05d, Y: %05d, Z: %05d", s->x, s->y, s->z));
  }

  // Touchpad: print coords for each finger
  for(int i = 0; i < 2; i++) {
    s = &event->state.touchpad[i];
    if(s->z) {
      LOG(LL_INFO, ("Finger #%d X:%04d, Y: %04d", i + 1, s->x, s->y));
    }
  }
}
// ...
```

## Input Event
Input event triggered everytime new data received from a controller, the state 
of all buttons and sensors stored in `event->state` structure.
To track when stick, trigger, gyroscope or accelerometer data is actually
changed(stick moved, trigger pressed), use one of the available `event->*_move` flags.
Note that `event->*_move` flags for analog values rely on throttle, so when it is disabled, the flags will always be `true` due to 
jitter.

## Pairing Controller
* Call `mgos_ds4_discover_and_pair()`
* Hold PS button + Share on the controller until the LED blinks white
* First found DS4 controller will be paired
* The paired controller will be disconnected after 1 second and bluetooth service will switch back to listening mode
* Press PS button to connect paired controller

If no device discovered after 12.8 seconds, discovery will stop and bluetooth service will switch back to listening mode. You can adjust that timeout in the config - `ds4.discovery_timeout`.

Note: calling `mgos_ds4_discover_and_pair()` will disconnect all currently connected controllers.

## Analog Values Throttle
To reduce jitter in analog values stream, the library have a basic throttle
mechanism. Throttle thresholds can be adjusted in the config:

```yml
- ["ds4.stick_threshold", "i", 1]
- ["ds4.trigger_threshold", "i", 2]
- ["ds4.gyro_threshold", "i", 50]
- ["ds4.accel_threshold", "i", 300]
```


## Analog Values Deadzone
Sometimes after releasing stick or trigger, it may not return to the rest 
position, in which case, controller report values greater than zero. The library 
uses deadzone configuration, values lower than deadzone threshold will be 
replaced by zero.
```yml
- ["ds4.stick_deadzone", "i", 15}]
- ["ds4.trigger_deadzone", "i", 5}]
```

## Documentation
See `mgos_ds4.h` for more information regarding data structure and available methods.

## Thanks
Thanks to these projects and their authors for finding a way to connect to HID 
device over Classic Bluetooth(ESP-IDF implements high-level API for HID over BLE, 
but lack for Classic BT).
- https://github.com/jvpernis/esp32-ps3
- https://github.com/aed3/PS4-esp32

Thanks to the authors of the Sony HID driver for Linux:
- https://github.com/torvalds/linux/blob/master/drivers/hid/hid-sony.c

