#include "lpps_i2c.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpio.h"
#include "i2c_master.h"
#include "pointing_device.h"
#include "timer.h"

#ifdef CONSOLE_ENABLE
#include "print.h"

volatile uint32_t lpps_debug_get_report_count = 0;
volatile uint32_t lpps_debug_read_ok_count    = 0;
volatile uint32_t lpps_debug_nonzero_count    = 0;
volatile int8_t   lpps_debug_max_abs_x        = 0;
volatile int8_t   lpps_debug_max_abs_y        = 0;
volatile int8_t   lpps_debug_max_abs_z        = 0;
#endif

#ifdef MOUSE_EXTENDED_REPORT
typedef int16_t lpps_xy_report_t;
#    define LPPS_XY_MIN (-32767)
#    define LPPS_XY_MAX (32767)
#else
typedef int8_t lpps_xy_report_t;
#    define LPPS_XY_MIN (-127)
#    define LPPS_XY_MAX (127)
#endif

typedef struct {
    bool     calibration_pending;
    uint16_t runtime_cpi;
    int16_t  bias_x_q8;
    int16_t  bias_y_q8;
    uint16_t zero_streak;
    uint16_t last_calibration_ms;
} lpps_state_t;

static lpps_state_t lpps_state = {
    .runtime_cpi = LPPS_CPI_DEFAULT,
};

static inline uint16_t lpps_clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static inline int8_t lpps_abs8(int8_t value) {
    return value < 0 ? (int8_t)-value : value;
}

static inline int16_t lpps_abs16(int16_t value) {
    return value < 0 ? -value : value;
}

static inline lpps_xy_report_t lpps_clamp_xy(int32_t value) {
    if (value < LPPS_XY_MIN) {
        return (lpps_xy_report_t)LPPS_XY_MIN;
    }
    if (value > LPPS_XY_MAX) {
        return (lpps_xy_report_t)LPPS_XY_MAX;
    }
    return (lpps_xy_report_t)value;
}

static inline int8_t lpps_q8_to_i8_clamped(int16_t v_q8) {
    int16_t v = v_q8 / 256;

    if (v > 127) {
        v = 127;
    } else if (v < -127) {
        v = -127;
    }

    return (int8_t)v;
}

static inline int16_t lpps_apply_axis_scale(int8_t raw) {
    int16_t value = raw;

    value = (int16_t)(((int32_t)value * (int32_t)lpps_state.runtime_cpi) / 1000);
    return value;
}

static inline int8_t lpps_apply_deadzone(int8_t v) {
    return (v >= -LPPS_DEAD_ZONE && v <= LPPS_DEAD_ZONE) ? 0 : v;
}

static inline void lpps_track_bias_q8(int16_t *bias_q8, int16_t raw_q8) {
#if LPPS_BIAS_TRACKING > 0
    *bias_q8 += (int16_t)((((int32_t)raw_q8 - *bias_q8) * LPPS_BIAS_TRACKING) >> 8);
#else
    (void)bias_q8;
    (void)raw_q8;
#endif
}

static bool lpps_motion_pin_active(void) {
#ifdef POINTING_DEVICE_MOTION_PIN
#    ifdef POINTING_DEVICE_MOTION_PIN_ACTIVE_LOW
    return gpio_read_pin(POINTING_DEVICE_MOTION_PIN) == 0;
#    else
    return gpio_read_pin(POINTING_DEVICE_MOTION_PIN) != 0;
#    endif
#else
    return true;
#endif
}

static bool lpps_send_calibration_command(void) {
    const uint8_t data = LPPS_CALIBRATION_VALUE;

    for (uint8_t attempt = 0; attempt < (uint8_t)(LPPS_CALIBRATION_RETRY_COUNT + 1); ++attempt) {
        if (i2c_write_register(LPPS_I2C_ADDRESS << 1, LPPS_CALIBRATION_REGISTER, &data, 1, LPPS_I2C_TIMEOUT_MS) == I2C_STATUS_SUCCESS) {
            return true;
        }
    }

    return false;
}

bool lpps_read_raw(lpps_raw_data_t *raw) {
    if (raw == NULL) {
        return false;
    }

    uint8_t buffer[3] = {0, 0, 0};
    uint8_t close_byte;

    for (uint8_t attempt = 0; attempt < (uint8_t)(LPPS_I2C_RETRY_COUNT + 1); ++attempt) {
        if (i2c_receive(LPPS_I2C_ADDRESS << 1, buffer, sizeof(buffer), LPPS_I2C_TIMEOUT_MS) != I2C_STATUS_SUCCESS) {
            continue;
        }

        (void)i2c_receive(LPPS_I2C_ADDRESS << 1, &close_byte, 1, LPPS_I2C_TIMEOUT_MS); // close device.

        raw->x = (int8_t)buffer[0];
        raw->y = (int8_t)buffer[1];
        raw->z = (int8_t)buffer[2];
        return true;
    }

    return false;
}

void lpps_request_calibration(void) {
    lpps_state.calibration_pending = true;
}

bool lpps_calibration_pending(void) {
    return lpps_state.calibration_pending;
}

bool lpps_calibration_task(void) {
    if (!lpps_state.calibration_pending) {
        return false;
    }

    if (!lpps_motion_pin_active()) {
        return false;
    }

    if (!lpps_send_calibration_command()) {
        return false;
    }

#ifdef CONSOLE_ENABLE
    uprintf("Calibration is completed. Last bias(x,y) = %d,%d; ", lpps_state.bias_x_q8, lpps_state.bias_y_q8);
    uprintf("zero_streak = %d; last_calibration_ms_elapsed = %d\n", lpps_state.zero_streak, timer_elapsed(lpps_state.last_calibration_ms));
#endif

    lpps_state.calibration_pending = false;

    lpps_state.bias_x_q8           = 0;
    lpps_state.bias_y_q8           = 0;
    lpps_state.zero_streak         = 0;
    lpps_state.last_calibration_ms = timer_read();

    return true;
}

void lpps_reset_state(void) {
    lpps_state.calibration_pending = false;
    lpps_state.runtime_cpi         = lpps_clamp_u16(LPPS_CPI_DEFAULT, LPPS_CPI_MIN, LPPS_CPI_MAX);
    lpps_state.bias_x_q8           = 0;
    lpps_state.bias_y_q8           = 0;
    lpps_state.zero_streak         = 0;
    lpps_state.last_calibration_ms = timer_read();
}

bool pointing_device_driver_init(void) {
    i2c_init();
    lpps_reset_state();
    return true;
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
#ifdef CONSOLE_ENABLE
    lpps_debug_get_report_count++;
#endif

    lpps_raw_data_t raw = {0, 0, 0};

    mouse_report.x = 0;
    mouse_report.y = 0;

    (void)lpps_calibration_task();

    if (!lpps_read_raw(&raw)) {
        return mouse_report;
    }


    int16_t raw_x_q8 = (int16_t)raw.x << 8;
    int16_t raw_y_q8 = (int16_t)raw.y << 8;

    int16_t corrected_x_q8 = raw_x_q8 - lpps_state.bias_x_q8;
    int16_t corrected_y_q8 = raw_y_q8 - lpps_state.bias_y_q8;

    /*
     * Update bias only while at rest.
     */
    if (lpps_abs16(corrected_x_q8) <= ((int16_t)LPPS_REST_THRESHOLD << 8) &&
        lpps_abs16(corrected_y_q8) <= ((int16_t)LPPS_REST_THRESHOLD << 8)) {
        lpps_track_bias_q8(&lpps_state.bias_x_q8, raw_x_q8);
        lpps_track_bias_q8(&lpps_state.bias_y_q8, raw_y_q8);

        corrected_x_q8 = raw_x_q8 - lpps_state.bias_x_q8;
        corrected_y_q8 = raw_y_q8 - lpps_state.bias_y_q8;
    }

#ifdef CONSOLE_ENABLE
    lpps_debug_read_ok_count++;

    if (raw.x != 0 || raw.y != 0 || raw.z != 0) {
        lpps_debug_nonzero_count++;
    }

    if (lpps_abs8(raw.x) > lpps_debug_max_abs_x) {
        lpps_debug_max_abs_x = lpps_abs8(raw.x);
    }
    if (lpps_abs8(raw.y) > lpps_debug_max_abs_y) {
        lpps_debug_max_abs_y = lpps_abs8(raw.y);
    }
    if (lpps_abs8(raw.z) > lpps_debug_max_abs_z) {
        lpps_debug_max_abs_z = lpps_abs8(raw.z);
    }
#endif

    raw.x = lpps_q8_to_i8_clamped(corrected_x_q8);
    raw.y = lpps_q8_to_i8_clamped(corrected_y_q8);

    mouse_report.x = lpps_clamp_xy(lpps_apply_axis_scale(lpps_apply_deadzone(raw.x)));
    mouse_report.y = lpps_clamp_xy(lpps_apply_axis_scale(lpps_apply_deadzone(-raw.y)));

    /*
     * If MOTION stays active but corrected X/Y remain small movement for a long time,
     * the module is effectively stuck awake due to drift. Request a
     * recalibration, but rate-limit it.
     */
    if (lpps_abs8(raw.x) <= LPPS_AUTO_CALIB_MAX_X &&
        lpps_abs8(raw.y) <= LPPS_AUTO_CALIB_MAX_Y) {
        if (lpps_state.zero_streak < UINT16_MAX) {
            lpps_state.zero_streak++;
        }

        if (lpps_state.zero_streak >= LPPS_AUTO_CALIB_STREAK &&
            !lpps_state.calibration_pending &&
            timer_elapsed(lpps_state.last_calibration_ms) >= LPPS_AUTO_CALIB_COOLDOWN_MS) {
            lpps_request_calibration();
            lpps_state.zero_streak = 0;
        }
    } else {
        lpps_state.zero_streak = 0;
    }

    return mouse_report;
}

uint16_t pointing_device_driver_get_cpi(void) {
    return lpps_state.runtime_cpi;
}

void pointing_device_driver_set_cpi(uint16_t cpi) {
    lpps_state.runtime_cpi = lpps_clamp_u16(cpi, LPPS_CPI_MIN, LPPS_CPI_MAX);
}

#ifdef CONSOLE_ENABLE
void lpps_get_debug_state(lpps_debug_state_t *state) {
    if (state == NULL) {
        return;
    }

    state->runtime_cpi                = lpps_state.runtime_cpi;
    state->calibration_pending        = lpps_state.calibration_pending;
    state->bias_x_q8                  = lpps_state.bias_x_q8;
    state->bias_y_q8                  = lpps_state.bias_y_q8;
    state->zero_streak                = lpps_state.zero_streak;
    state->since_last_calibration_ms  = timer_elapsed(lpps_state.last_calibration_ms);
}
#endif
