#include "kscan_m65legorev9.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include <zmk/debounce.h>

LOG_MODULE_DECLARE(m65legorev9, CONFIG_KSCAN_LOG_LEVEL)

#define DT_DRV_COMPAT m65legorev9_kscan

#if CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS >= 0
#define INST_DEBOUNCE_PRESS_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS
#else
#define INST_DEBOUNCE_PRESS_MS(n)                                                                  \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_press_ms))
#endif

#if CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS >= 0
#define INST_DEBOUNCE_RELEASE_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS
#else
#define INST_DEBOUNCE_RELEASE_MS(n)                                                                \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_release_ms))
#endif

struct m65legorev9_data {
  const struct device * dev;
  kscan_callback_t callback;
  struct k_work_delayable work;

  /** Timestamp of the current or scheduled scan. */
  int64_t scan_time;
  /**
   * Current state of the matrix as a flattened 2D array of length 66
   */
  struct zmk_debounce_state matrix_state[M65LEGOREV9_KEY_COUNT];
};

struct m65legorev9_config {
  struct gpio_dt_spec latch;
  struct gpio_dt_spec clock;
  struct gpio_dt_spec dout;
  struct gpio_dt_spec din;
  
  struct zmk_debounce_config debounce_config;
  int32_t debounce_scan_period_ms;
  int32_t poll_period_ms;
};

/**
 * Shift out data to 2 chained 74HC595
 * \param dout data out gpio
 * \param clk clock gpio
 * \param value data to shift
 * \param bits size of data
 * 
 * \warning does not manage latch
 */
static int shift_out(
  const struct gpio_dt_spec * dout, 
  const struct gpio_dt_spec * clock, 
  uint16_t value, int bits)
{
  for (int i = bits - 1; i >= 0; --i) {
    int err;
    err = gpio_pin_set_dt(dout, (value >> i) & 1);
    if (err) {
      return err;
    }
    err = gpio_pin_set_dt(clock, 1);
    if (err) {
      return err;
    }
    err = gpio_pin_set_dt(clock, 0);
    if (err) {
      return err;
    }
  } 

  return 0;
}

/**
 * Shift data in from 74HC165
 * \param [in]  din data in gpio
 * \param [in]  clk clock gpio
 * \param [out] out output value
 */
static int shift_in(
  const struct gpio_dt_spec * din, 
  const struct gpio_dt_spec * clock,
  uint8_t * out)
{
  uint8_t data = 0;
  int err = 0;
  for (int i = 0; i < 8; ++i) {
    data <<= 1;
    int val = gpio_pin_get_dt(din);
    data |= (val & 1);
    err = gpio_pin_set_dt(clock, 1);
    if (err) {
      return err;
    }
    err = gpio_pin_set_dt(clock, 0);
    if (err) {
      return err;
    }
  }
  *out = data;
  return 0;
}

static void m65legorev9_read_continue(const struct device *dev) {
  const struct m65legorev9_config * config = dev->config;
  struct m65legorev9_data * data = dev->data;

  data->scan_time += config->debounce_scan_period_ms;

  k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static void m65legorev9_read_end(const struct device *dev) {
  struct m65legorev9_data *data = dev->data;
  const struct m65legorev9_config *config = dev->config;

  data->scan_time += config->poll_period_ms;

  // Return to polling slowly.
  k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static int m65legorev9_read(const struct device * dev)
{
  const struct m65legorev9_config * config = dev->config;
  struct m65legorev9_data * data = dev->data;

  const int column_map[] = {6, 5, 4, 3, 2, 1, 0, 7, 8, 9, 10, 11, 15, 12};
  int err;

  for (int col = 0; col < 14; ++col) {
    // send data to 74HC595
    // latch low to send data
    err = gpio_pin_set_dt(&config->latch, 0);
    if (err) {
      LOG_ERR("Failed to set latch low: %i", err);
      return err;
    }
    // clear shift register
    err = shift_out(&config->dout, &config->clock, 0, 16);
    if (err) {
      LOG_ERR("Failed to shift out zeros: %i", err);
      return err;
    }
    // activate column of interest
    err = shift_out(&config->dout, &config->clock, 1 << column_map[col], 16);
    if (err) {
      LOG_ERR("Failed to shift out bit %i: %i", column_map[col], err);
      return err;
    }
    // activate
    err = gpio_pin_set_dt(&config->latch, 1);
    if (err) {
      LOG_ERR("Failed to set latch high: %i", err);
      return err;
    }
    

    // latch low to load
    err = gpio_pin_set_dt(&config->latch, 0);
    if (err) {
      LOG_ERR("Failed to set latch low: %i", err);
      return err;
    }
    // latch high to shift in
    err = gpio_pin_set_dt(&config->latch, 1);
    if (err) {
      LOG_ERR("Failed to set latch high: %i", err);
      return err;
    }
    uint8_t data_in = 0;
    err = shift_in(&config->din, &config->clock, &data_in);
    if (err) {
      LOG_ERR("Failed to shift in: %i", err);
      return err;
    }

    // number of rows, 1 for encoder on last hidden row
    uint8_t n_rows = (col != 13) ? 5 : 1;
    for (uint8_t row = 0; row < n_rows; ++row) {
      int key_n = 13 * row + col;  // key at column col and row row 
      
      // safety
      if (key_n >= M65LEGOREV9_KEY_COUNT) continue;

      zmk_debounce_update(
        &data->matrix_state[key_n], (data_in & (1 << row)),
        config->debounce_scan_period_ms, config->debounce_config);
      // encoder naturally goes to the 66th key. Nice!
    }
  }

  // process new state
  bool continue_scan = false;

  // yes I know I'm ignoring the encoder..
  int key = 0;
  for (int row = 0; row < 5; ++row) {
    for (int col = 0; col < 13; ++col) {
      if (key > M65LEGOREV9_KEY_COUNT) {
        break;
      }

      struct zmk_debounce_state * state = &data->matrix_state[key];
      if (zmk_debounce_get_changed(state)) {
        const bool pressed = zmk_debounce_is_pressed(state);

        LOG_DBG("Sending event at %i,%i state %s", row, col, pressed ? "on" : "off");
        data->callback(dev, row, col, pressed);
      }

      continue_scan = continue_scan || zmk_debounce_is_active(state);

      ++key;
    }
  }

  if (continue_scan) {
    // At least one key is pressed or the debouncer has not yet decided if
    // it is pressed. Poll quickly until everything is released.
    m65legorev9_read_continue(dev);
  } else {
    // All keys are released. Return to normal.
    m65legorev9_read_end(dev);
  }
  return 0;
}

static void m65legorev9_work_handler(struct k_work * work)
{
  struct k_work_delayable * dwork = k_work_delayable_from_work(work);
  struct m65legorev9_data * data = CONTAINER_OF(dwork, struct m65legorev9_data, work);
  m65legorev9_read(data->dev);
}

/**
 * Set callback
 */
static int m65legorev9_configure(
  const struct device * dev,
  const kscan_callback_t callback)
{
  struct m65legorev9_data * data = dev->data;
  
  if (!callback) {
    return -EINVAL;
  }

  data->callback = callback;
  return 0;
}

static int m65legorev9_enable(const struct device * dev) {
    struct m65legorev9_data * data = dev->data;

    data->scan_time = k_uptime_get();

    // Read will automatically start polling once done.
    return m65legorev9_read(dev);
}

static int m65legorev9_disable(const struct device *dev) {
    struct m65legorev9_data * data = dev->data;

    k_work_cancel_delayable(&data->work);

    return 0;
}

static int m65legorev9_init(const struct device * dev)
{
  struct m65legorev9_config * config = dev->config;
  struct m65legorev9_data * data = dev->data;

  data->dev = dev;

  // configure gpios
  int err;
  err = gpio_pin_configure_dt(&config->latch, GPIO_OUTPUT);
  if (err) {
    LOG_ERR("Unabled to configure latch pin %u on %s for output",
      config->latch.pin, config->latch.port->name);
      return err;
  }
  err = gpio_pin_configure_dt(&config->clock, GPIO_OUTPUT);
  if (err) {
    LOG_ERR("Unabled to configure clock pin %u on %s for output",
      config->clock.pin, config->clock.port->name);
      return err;
  }
  err = gpio_pin_configure_dt(&config->dout,  GPIO_OUTPUT);
  if (err) {
    LOG_ERR("Unabled to configure dout pin %u on %s for output",
      config->dout.pin, config->dout.port->name);
      return err;
  }
  err = gpio_pin_configure_dt(&config->din,   GPIO_INPUT);
  if (err) {
    LOG_ERR("Unabled to configure din pin %u on %s for input",
      config->din.pin, config->din.port->name);
      return err;
  }

  k_work_init_delayable(&data->work, m65legorev9_work_handler);

  return 0;
}

static const struct kscan_driver_api m65legorev9_api = {
  .config = m65legorev9_configure,
  .enable_callback = m65legorev9_enable,
  .disable_callback = m65legorev9_disable,
};

#define M65LEGOREV9_INIT(n)                                                           \
  BUILD_ASSERT(INST_DEBOUNCE_PRESS_MS(n) <= DEBOUNCE_COUNTER_MAX,                     \
    "ZMK_KSCAN_DEBOUNCE_PRESS_MS or debounce-press-ms is too large");                 \
  BUILD_ASSERT(INST_DEBOUNCE_RELEASE_MS(n) <= DEBOUNCE_COUNTER_MAX,                   \
    "ZMK_KSCAN_DEBOUNCE_RELEASE_MS or debounce-release-ms is too large");             \
                                                                                      \
  static struct m65legorev9_data m65legorev9_data_##n;                                \
  static const struct m65legorev9_config m65legorev9_config_##n = {                       \
    .latch = GPIO_DT_SPEC_INST_GET(n, latch_gpios),                                    \
    .clock = GPIO_DT_SPEC_INST_GET(n, clock_gpios),                                    \
    .dout  = GPIO_DT_SPEC_INST_GET(n, data_out_gpios),                                 \
    .din   = GPIO_DT_SPEC_INST_GET(n, data_in_gpios),                                  \
                                                                                      \
    .debounce_config =                                                                \
      {                                                                               \
        .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                               \
        .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                           \
      },                                                                              \
    .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms),              \
    .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                \
  };                                                                                  \
                                                                                      \
  DEVICE_DT_INST_DEFINE(                                                              \
    n,                                                                                \
    &m65legorev9_init,                                                                \
    NULL,                                                                             \
    &m65legorev9_data_##n,                                                            \
    &m65legorev9_config_##n,                                                          \
    POST_KERNEL, CONFIG_KSCAN_M65LEGOREV9_INIT_PRIORITY,                                          \
    &m65legorev9_api);

DT_INST_FOREACH_STATUS_OKAY(M65LEGOREV9_INIT);