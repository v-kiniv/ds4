#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <esp_system.h>
#include <esp_bt_main.h>
#include <esp_bt_defs.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "stack/gap_api.h"
#include "stack/bt_types.h"
#include "osi/allocator.h"
#include "mgos.h"
#include "mgos_timers.h"

#include "ds4_internal.h"
#include "mgos_ds4.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CONSTRAIN_8BIT(a) MIN(MAX(a, 0), 255)
#define FUNC_OFF 0xffff
#define SYNC_INTERVAL MAX(1, mgos_sys_config_get_ds4_sync_interval())

struct controller {
  bool                  blink_on;
  bool                  connected;
  bool                  rumble_on;
  bool                  stale;
  struct ds4_config     out;
  struct mg_str         mac;
  struct mgos_ds4_state state;
  uint16_t              blink_tl;
  uint16_t              hidc;
  uint16_t              hidi;
  uint16_t              rumble_tl;
  uint8_t               index;
};

struct discovery_state {
  bool found;
  bool in_progress;
  esp_bd_addr_t bda;
  uint32_t cod;
};

static bool                   m_server_is_running    = false;
static mgos_timer_id          m_sync_timer           = MGOS_INVALID_TIMER_ID;
static struct controller      m_ctrls[DS4_MAX_CTRLS] = { 0 };
static struct discovery_state m_disc_state           = { 0 };

static void restart_server_cb(void *arg);

static bool gap_hid_send(struct controller *c, struct ds4_hid_cmd *hid_cmd, 
                         uint8_t len) {
  BT_HDR *p_buf;

  if(!c->connected || c->hidc == GAP_INVALID_HANDLE) {
    LOG(LL_WARN, ("Invalid state of HIDC handle (%u)", c->index));
    return false;
  }

  p_buf = (BT_HDR *)osi_malloc(BT_DEFAULT_BUFFER_SIZE);
  if(!p_buf){
    LOG(LL_ERROR, ("Allocating buffer for sending the command failed"));
    return false;
  }

  p_buf->len = len + ( sizeof(*hid_cmd) - sizeof(hid_cmd->data) );
  p_buf->offset = L2CAP_MIN_OFFSET;

  memcpy ((uint8_t *)(p_buf + 1) + p_buf->offset, (uint8_t*)hid_cmd, p_buf->len);

  if (GAP_ConnBTWrite(c->hidc, p_buf) != BT_PASS) {
    LOG(LL_ERROR, ("HID command failed"));
    return false;
  }

  return true;
}

static struct mg_str address_to_str(esp_bd_addr_t bda)
{
  if (bda == NULL) {
    return mg_strdup_nul(mg_mk_str("00:00:00:00:00:00"));
  }

  char s[18];
  uint8_t *p = bda;
  sprintf(s, "%02x:%02x:%02x:%02x:%02x:%02x",
          p[0], p[1], p[2], p[3], p[4], p[5]);

  return mg_strdup_nul(mg_mk_str(s));
}

static void reset_controller_record(struct controller *c, uint8_t index) {
  memset(c, 0, sizeof(struct controller));

  c->index      = index;
  c->connected  = false;
  c->stale      = false;
  c->hidc       = GAP_INVALID_HANDLE;
  c->hidi       = GAP_INVALID_HANDLE;
  c->rumble_tl  = FUNC_OFF;
  c->blink_tl   = FUNC_OFF;
  
  if(c->mac.p != NULL) {
    mg_strfree(&c->mac);
  }
  c->mac = address_to_str(NULL);
}

static bool disconnect_controller(struct controller *c)
{ 
  bool res = true;

  if(c->hidi != GAP_INVALID_HANDLE) {
    res &= GAP_ConnClose(c->hidi) == BT_PASS;
  }
  if(c->hidc != GAP_INVALID_HANDLE) {
    res &= GAP_ConnClose(c->hidc) == BT_PASS;
  }

  if(c->connected) {
    struct mgos_ds4_connection_arg arg = {
      .index          = c->index,
      .device_address = c->mac,
      .connected      = false,
    };
    reset_controller_record(c, c->index);
    mgos_event_trigger(MGOS_DS4_CONNECTION, &arg);
  }

  return res;
}

static bool init_bt_stack()
{
  esp_err_t ret;

  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    LOG(LL_ERROR, ("Initialize controller failed: %s", esp_err_to_name(ret)));
    return false;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    LOG(LL_ERROR, ("Enable controller failed: %s", esp_err_to_name(ret)));
    return false;
  }

  if ((ret = esp_bluedroid_init()) != ESP_OK) {
    LOG(LL_ERROR, ("Initialize bluedroid failed: %s", esp_err_to_name(ret)));
    return false;
  }

  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    LOG(LL_ERROR, ("Enable bluedroid failed: %s", esp_err_to_name(ret)));
    return false;
  }

  if ((ret = esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE)) != ESP_OK) {
    LOG(LL_ERROR, ("Set scan mode failed: %s", esp_err_to_name(ret)));
    return false;
  }

  return true;
}

static bool deinit_bt_stack()
{
  esp_err_t ret;

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  if ((ret = esp_bluedroid_disable()) != ESP_OK) {
    LOG(LL_ERROR, ("Disable bluedroid failed: %s", esp_err_to_name(ret)));
    return false;
  }

  if ((ret = esp_bluedroid_deinit()) != ESP_OK) {
    LOG(LL_ERROR, ("Deinitialize bluedroid failed: %s", esp_err_to_name(ret)));
    return false;
  }

  if ((ret = esp_bt_controller_disable()) != ESP_OK) {
    LOG(LL_ERROR, ("Disable controller failed: %s", esp_err_to_name(ret)));
    return false;
  }

  if ((ret = esp_bt_controller_deinit()) != ESP_OK) {
    LOG(LL_ERROR, ("Deinitialize controller failed: %s", esp_err_to_name(ret)));
    return false;
  }

  return true;
}

static inline int throttle_value(int a, int b, uint16_t threshold, uint16_t deadzone)
{
  if(abs(b) < deadzone) {
    return 0;
  }
  if(abs(a - b) > threshold) {
    return b;
  }

  return a;
}

static inline void throttle_3asix(struct mgos_ds4_3axis *prev,
                                  struct mgos_ds4_3axis *next,
                                  uint16_t threshold,
                                  uint16_t deadzone)
{
  next->x = throttle_value(prev->x, next->x, threshold, deadzone);
  next->y = throttle_value(prev->y, next->y, threshold, deadzone);
  next->z = throttle_value(prev->z, next->z, threshold, deadzone);
}

static void throttle_state(struct mgos_ds4_state *prev,
                               struct mgos_ds4_state *next)
{
  int i;
  const struct mgos_config_ds4 *s = mgos_sys_config_get_ds4();

  throttle_3asix(&prev->accel, &next->accel, s->accel_threshold, 0);
  throttle_3asix(&prev->gyro, &next->gyro, s->gyro_threshold, 0);

  for(i = 0; i < 2; i++) {
    throttle_3asix(&prev->stick[i], &next->stick[i], 
                   s->stick_threshold, s->stick_deadzone);
  }

  for(i = 0; i < 2; i++) {
    next->trigger[i] = throttle_value(prev->trigger[i], next->trigger[i], 
                                      s->trigger_threshold, s->trigger_deadzone);
  }

  return true;
}

static bool update_controller_state(struct controller *c, uint8_t *data)
{
  struct mgos_ds4_state prev = c->state;

  if(!ds4_parse_packet(&c->state, data)) {
    return false;
  }

  throttle_state(&prev, &c->state);

  if(m_disc_state.in_progress) {
    return true;
  }

  struct mgos_ds4_input_arg arg = {
    .index       = c->index,
    .state       = c->state,
  };
  ds4_gen_event(&arg, &prev, &c->state);
  mgos_event_trigger(MGOS_DS4_INPUT, &arg);

  return true;
}

static void l2c_link_event_cb(uint16_t gap_handle, uint16_t event) {
  if(event != GAP_EVT_CONN_DATA_AVAIL) {
    LOG(LL_DEBUG, ("Event %u for handle: %u", event, gap_handle));
  }
  struct controller *c = NULL;
  for (size_t i = 0; i < DS4_MAX_CTRLS; i++)
  {
    if(gap_handle == m_ctrls[i].hidi || gap_handle == m_ctrls[i].hidc){
      c = &m_ctrls[i];
    }
  }

  if(c == NULL) {
    LOG(LL_WARN, ("Unknown handle: %u", gap_handle));
    return;
  }

  switch(event){
    case GAP_EVT_CONN_OPENED:
    case GAP_EVT_CONN_CLOSED: {
      const bool prev = c->connected;
      c->connected = GAP_ConnGetL2CAPCid(c->hidc) != 0
                  && GAP_ConnGetL2CAPCid(c->hidi) != 0;

      if(prev == c->connected) {
        return;
      }
      LOG(LL_INFO, ("Controller %u: %s", c->index, c->connected ? "connected" : "disconnected"));
      if(c->connected) {
        // send initial control packet to the controller to activate connection
        mgos_ds4_led_set(c->index, 64, 64, 64);
      }

      if(m_disc_state.in_progress) {
        LOG(LL_DEBUG, ("Controller paired"));
        mg_strfree(&c->mac);
        c->mac = address_to_str(m_disc_state.bda);
        mgos_set_timer(1000, 0, restart_server_cb, NULL);

        struct mgos_ds4_paired_arg arg = {
          .device_address = c->mac,
        };
        mgos_event_trigger(MGOS_DS4_PAIRED, &arg);
      } else {
        if(c->connected) {
          mg_strfree(&c->mac);
          c->mac = address_to_str(GAP_ConnGetRemoteAddr(c->hidi));
        }

        struct mgos_ds4_connection_arg arg = {
          .index          = c->index,
          .device_address = c->mac,
          .connected      = c->connected,
        };
        mgos_event_trigger(MGOS_DS4_CONNECTION, &arg);
      }

      break;
    }

    case GAP_EVT_CONN_DATA_AVAIL: {
      uint8_t result;
      BT_HDR *p_buf;

      result = GAP_ConnBTRead(gap_handle, &p_buf);
      if(result != BT_PASS) {
        LOG(LL_ERROR, ("GAP_ConnBTRead failed, index: %u, handle: %u, size: %u",
                          c->index, gap_handle, p_buf->len)
        );
      }
      
      if (p_buf->len == DS4_REPORT_BUFFER_SIZE) {
        update_controller_state(c, p_buf->data);
      }

      osi_free(p_buf);
      break;
    }

    default:
      LOG(LL_WARN, ("Unhandled GAP event %u", event));
      break;
  }
}

static bool create_l2c_link(struct controller *c, esp_bd_addr_t addr)
{
  tL2CAP_ERTM_INFO ertm_info = { 0 };
  LOG(LL_DEBUG, ("create_l2c_link for controller #%u (%s)", 
                      c->index, addr == NULL ? "server" : "client"));

  c->hidc = GAP_ConnOpen(
    "DS4-HIDC", BTM_SEC_SERVICE_HIDH_SEC_CTRL, addr == NULL, addr, BT_PSM_HIDC,
    NULL, &ertm_info, 0, 0, l2c_link_event_cb
  );
  if(c->hidc == GAP_INVALID_HANDLE) {
    LOG(LL_ERROR, ("Failed to register HIDC for controller #%u", c->index));
    return false;
  }

  c->hidi = GAP_ConnOpen(
    "DS4-HIDI", BTM_SEC_SERVICE_HIDH_INTR, addr == NULL, addr, BT_PSM_HIDI,
    NULL, &ertm_info, 0, 0, l2c_link_event_cb
  );

  if(c->hidi == GAP_INVALID_HANDLE) {
    LOG(LL_ERROR, ("Failed to register HIDI for controller #%u", c->index));
    return false;
  }

  LOG(LL_DEBUG, ("Register, HIDC: %u, HIDI: %u", c->hidc, c->hidi));
  return true;
}

static bool stop_server()
{
  for (size_t i = 0; i < DS4_MAX_CTRLS; i++) {
    struct controller *c = &m_ctrls[i];
    
    if(!disconnect_controller(c)) {
      LOG(LL_ERROR, ("Disconnect command failed for controller %u", c->index));
    }
  }

  if(!deinit_bt_stack()) {
    return false;
  }
  m_server_is_running = false;

  return true;
}

static bool start_server(bool allocate_listeners)
{
  if(m_server_is_running) {
    return true;
  }
  if(!init_bt_stack()) {
    return false;
  }

  for (size_t i = 0; i < DS4_MAX_CTRLS; i++) {
    struct controller *c = &m_ctrls[i];
    reset_controller_record(c, i);

    if(allocate_listeners) {
      if(!create_l2c_link(c, NULL)) {
        LOG(LL_ERROR, ("Failed to create L2CAP link for controller #%u", c->index));
        return false;
      }
    }
  }
  m_server_is_running = true;

  return true;
}

static void restart_server_cb(void *arg)
{
  (void) arg;
  m_disc_state.in_progress = false;
  stop_server();
  start_server(true);
}

static bool validate_cod(uint32_t class_of_device)
{
  uint32_t minor, major;
  if(!esp_bt_gap_is_valid_cod(class_of_device)) {
    return false;
  }

  minor = esp_bt_gap_get_cod_minor_dev(class_of_device);
  major = esp_bt_gap_get_cod_major_dev(class_of_device);
  if(major != ESP_BT_COD_MAJOR_DEV_PERIPHERAL || minor != 0x02) {
    return false;
  }

  return true;
}

static void device_found_cb(esp_bd_addr_t addr)
{
  struct controller *c = &m_ctrls[0];

  if(c->connected) {
    LOG(LL_ERROR, ("Invalid state of the first controller"));
    return;
  }
  
  // Connect found controller to the first slot using 'client' l2cap link
  create_l2c_link(c, addr);
}

static void gap_event_cb(esp_bt_gap_cb_event_t event, 
                          esp_bt_gap_cb_param_t *param)
{
    struct discovery_state *ds = &m_disc_state;
    esp_bt_gap_dev_prop_t *p;

    if(event == ESP_BT_GAP_DISC_RES_EVT) {
      for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        if (p->type == ESP_BT_GAP_DEV_PROP_COD) {
          ds->cod = *(uint32_t *)(p->val);
          if(validate_cod(ds->cod)) {
            ds->found = true;
            memcpy(ds->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
            LOG(LL_DEBUG, ("Cancel device discovery ..."));
            esp_bt_gap_cancel_discovery();
          }
        }
      }
    }

    if(event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
      if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        LOG(LL_DEBUG, ("Device discovery stopped."));
        mgos_event_trigger(MGOS_DS4_DISCOVERY_STOPPED, NULL);
        if(ds->found) {
          device_found_cb(ds->bda);
          ds->found = false;
        }
      } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
        LOG(LL_DEBUG, ("Discovery started."));
        mgos_event_trigger(MGOS_DS4_DISCOVERY_STARTED, NULL);
      }
    }
}

static bool sync(struct controller *c) {
  struct ds4_config *cfg     = &c->out;
  struct ds4_hid_cmd hid_cmd = { .data = {0x80, 0x00, 0xFF} };
  uint16_t           len     = sizeof(hid_cmd.data);

  hid_cmd.code = DS4_HID_CC_SET_REPORT | DS4_HID_CC_TYPE_OUTPUT;
  hid_cmd.identifier = 0x11;

  hid_cmd.data[DS4_CPI_RUMBLE_SMALL]  = cfg->rumble_small;
  hid_cmd.data[DS4_CPI_RUMBLE_LARGE]  = cfg->rumble_large;

  hid_cmd.data[DS4_CPI_LED_RED]       = cfg->r;
  hid_cmd.data[DS4_CPI_LED_GREEN]     = cfg->g;
  hid_cmd.data[DS4_CPI_LED_BLUE]      = cfg->b;

  hid_cmd.data[DS4_CPI_LED_BLINK_ON]  = cfg->blink_on;
  hid_cmd.data[DS4_CPI_LED_BLINK_OFF] = cfg->blink_off;

  return gap_hid_send(c, &hid_cmd, len);
}

static void update_cb(void *arg) {
  for(size_t i = 0; i < DS4_MAX_CTRLS; i++) {
    struct controller *c = &m_ctrls[i];

    if(c->rumble_tl != FUNC_OFF && c->rumble_on) {
      if(c->rumble_tl > 0) {
        c->rumble_tl -= 1;
      } else {
        mgos_ds4_rumble_set(c->index, 0, 0);
      }
    }

    if(c->blink_tl != FUNC_OFF && c->blink_on) {
      if(c->blink_tl > 0) {
        c->blink_tl -= 1;
      } else {
        mgos_ds4_led_blink_set(c->index, 0, 0);
      }
    }

    if(c->stale) {
      c->stale = false;
      sync(c);
    }
  }
}

static struct controller * get_controller(int controller_index)
{
  if(controller_index < 0 || controller_index > DS4_MAX_CTRLS - 1) {
    LOG(LL_WARN, ("Controller index is out of range"));
    return NULL;
  }
  return &m_ctrls[controller_index];
}

bool mgos_ds4_init(void) {
  mgos_event_register_base(MGOS_DS4_BASE, "ds4");
  if (!mgos_sys_config_get_ds4_enable()) {
    return true;
  }
  return mgos_ds4_start();
}

bool mgos_ds4_start(void)
{
  memset(&m_disc_state, 0, sizeof(struct discovery_state));

  if(!start_server(true)) {
    return false;
  }

  esp_bt_gap_register_callback(gap_event_cb);
  m_sync_timer = mgos_set_timer(SYNC_INTERVAL,
                                MGOS_TIMER_REPEAT, update_cb, NULL);

  mgos_event_trigger(MGOS_DS4_STARTED, NULL);
  return true;
}

bool mgos_ds4_stop(void)
{
  if(m_sync_timer != MGOS_INVALID_TIMER_ID) {
    mgos_clear_timer(m_sync_timer);
  }
  return stop_server();
}

bool mgos_ds4_discover_and_pair(void)
{
  int timeout = MIN(MAX(mgos_sys_config_get_ds4_discovery_timeout(), 1), 48);
  m_disc_state.in_progress = true;
  
  if(m_server_is_running) { // restart
    stop_server();
  }
  if(!start_server(false)) {
    return false;
  }
  
  
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, timeout, 0);
  
  return true;
}

bool mgos_ds4_cancel_discovery(void)
{
  if(!m_disc_state.in_progress) {
    return false;
  }

  esp_bt_gap_cancel_discovery();
  restart_server_cb(NULL);

  return true;
}

bool mgos_ds4_is_connected(int controller_index)
{
  struct controller *c = get_controller(controller_index);

  if(c == NULL) {
    return false;
  }

  return c->connected;
}

bool mgos_ds4_is_discovering(void)
{
  return m_disc_state.in_progress;
}

bool mgos_ds4_disconnect(int controller_index) {
  struct controller *c = get_controller(controller_index);

  if(c == NULL) {
    return false;
  }

  if(c->stale) {
    sync(c);
  }
  if(!disconnect_controller(c)) {
    return false;
  }
  create_l2c_link(c, NULL);

  return true;
}

bool mgos_ds4_led_set(int controller_index, int lr, int lg, int lb)
{
  struct controller *c = get_controller(controller_index);
  struct ds4_config *cfg;
  uint8_t r = CONSTRAIN_8BIT(lr);
  uint8_t g = CONSTRAIN_8BIT(lg);
  uint8_t b = CONSTRAIN_8BIT(lb);

  if(c == NULL || !c->connected) {
    return false;
  }

  cfg = &c->out;
  if(r != cfg->r || g != cfg->g || b != cfg->b) {
    cfg->r = r;
    cfg->g = g;
    cfg->b = b;

    c->stale = true;
  }

  return true;
}

bool mgos_ds4_led_blink_set(int controller_index, int on_time, int off_time)
{
  struct controller *c = get_controller(controller_index);
  struct ds4_config *cfg;
  uint8_t on = CONSTRAIN_8BIT(on_time / 10);
  uint8_t off = CONSTRAIN_8BIT(off_time / 10);

  if(c == NULL || !c->connected) {
    return false;
  }
  cfg = &c->out;
  c->blink_on = on > 0 || off > 0;
  c->blink_tl = FUNC_OFF;

  if(on != cfg->blink_on || off != cfg->blink_off) {
    cfg->blink_on = on;
    cfg->blink_off = off;

    c->stale = true;
  }
 
  return true;
}

bool mgos_ds4_led_blink_timeout(int controller_index, int timeout_ms)
{
  struct controller *c = get_controller(controller_index);
  if(c == NULL || !c->blink_on) {
    return false;
  }

  c->blink_tl = timeout_ms / SYNC_INTERVAL;
  return true;
}

bool mgos_ds4_rumble_set(int controller_index, int small, int large)
{
  struct controller *c = get_controller(controller_index);
  struct ds4_config *cfg;
  uint8_t s = CONSTRAIN_8BIT(small);
  uint8_t l = CONSTRAIN_8BIT(large);

  if(c == NULL || !c->connected) {
    return false;
  }

  cfg = &c->out;
  c->rumble_on = s > 0 || l > 0;
  c->rumble_tl = FUNC_OFF;

  if(s != cfg->rumble_small || l != cfg->rumble_large){
    cfg->rumble_small = s;
    cfg->rumble_large = l;
    
    c->stale = true;
  }

  return true;
}

bool mgos_ds4_rumble_timeout(int controller_index, int timeout_ms)
{
  struct controller *c = get_controller(controller_index);
  if(c == NULL || !c->rumble_on) {
    return false;
  }

  c->rumble_tl = timeout_ms / SYNC_INTERVAL;
  return true;
}

struct mg_str mgos_ds4_get_controller_address(int controller_index)
{
  struct controller *c = get_controller(controller_index);
  if(c == NULL) {
    return mg_mk_str("00:00:00:00:00:00");
  }

  return c->mac;
}

bool mgos_ds4_flush(int controller_index)
{
  struct controller *c = get_controller(controller_index);
  if(c == NULL || !c->stale) {
    return false;
  }

  c->stale = false;
  return sync(c);
}
