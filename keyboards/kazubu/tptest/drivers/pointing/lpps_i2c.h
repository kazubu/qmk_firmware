#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * LPPS module defaults
 */
#ifndef LPPS_I2C_ADDRESS
#    define LPPS_I2C_ADDRESS 0x57
#endif

#ifndef LPPS_I2C_TIMEOUT_MS
#    define LPPS_I2C_TIMEOUT_MS 2
#endif

#ifndef LPPS_I2C_RETRY_COUNT
#    define LPPS_I2C_RETRY_COUNT 1
#endif

#ifndef LPPS_CALIBRATION_REGISTER
#    define LPPS_CALIBRATION_REGISTER 0x04
#endif

#ifndef LPPS_CALIBRATION_VALUE
#    define LPPS_CALIBRATION_VALUE 0x01
#endif

#ifndef LPPS_CALIBRATION_RETRY_COUNT
#    define LPPS_CALIBRATION_RETRY_COUNT 1
#endif

/*
 * Runtime sensitivity multiplier.
 * 1000 = 1.0x
 */
#ifndef LPPS_CPI_DEFAULT
#    define LPPS_CPI_DEFAULT 1000
#endif

#ifndef LPPS_CPI_MIN
#    define LPPS_CPI_MIN 125
#endif

#ifndef LPPS_CPI_MAX
#    define LPPS_CPI_MAX 4000
#endif

/*
 * Dead Zone (ignore equal or less than)
 */
#ifndef LPPS_DEAD_ZONE
#    define LPPS_DEAD_ZONE 1
#endif

/*
 * Drift compensation
 */
#ifndef LPPS_REST_THRESHOLD
#    define LPPS_REST_THRESHOLD 4
#endif

#ifndef LPPS_BIAS_TRACKING
#    define LPPS_BIAS_TRACKING 6
#endif

/*
 * Auto calibration when MOTION stays active but corrected X/Y remain small movement for a long time.
 * Values below assume roughly 70 reports/s.
 */
#ifndef LPPS_AUTO_CALIB_STREAK
#    define LPPS_AUTO_CALIB_STREAK 210
#endif

#ifndef LPPS_AUTO_CALIB_COOLDOWN_MS
#    define LPPS_AUTO_CALIB_COOLDOWN_MS 3000
#endif

#ifndef LPPS_AUTO_CALIB_MAX_X
#    define LPPS_AUTO_CALIB_MAX_X 16
#endif

#ifndef LPPS_AUTO_CALIB_MAX_Y
#    define LPPS_AUTO_CALIB_MAX_Y 16
#endif

typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} lpps_raw_data_t;

bool lpps_read_raw(lpps_raw_data_t *raw);
void lpps_reset_state(void);
void lpps_request_calibration(void);
bool lpps_calibration_task(void);
bool lpps_calibration_pending(void);
#ifdef CONSOLE_ENABLE
typedef struct {
    uint16_t runtime_cpi;
    bool     calibration_pending;
    int16_t  bias_x_q8;
    int16_t  bias_y_q8;
    uint16_t zero_streak;
    uint16_t since_last_calibration_ms;
} lpps_debug_state_t;

void lpps_get_debug_state(lpps_debug_state_t *state);

extern volatile uint32_t lpps_debug_get_report_count;
extern volatile uint32_t lpps_debug_read_ok_count;
extern volatile uint32_t lpps_debug_nonzero_count;
extern volatile int8_t   lpps_debug_max_abs_x;
extern volatile int8_t   lpps_debug_max_abs_y;
extern volatile int8_t   lpps_debug_max_abs_z;
#endif
