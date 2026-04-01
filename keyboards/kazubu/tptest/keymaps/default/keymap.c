#include QMK_KEYBOARD_H
#include "drivers/pointing/lpps_i2c.h"

void lpps_request_calibration(void);
bool lpps_calibration_task(void);

#ifdef CONSOLE_ENABLE
static uint16_t last_log = 0;
static uint32_t last_report = 0;
static uint32_t last_ok = 0;
static uint32_t last_nonzero = 0;
#endif

enum custom_keycodes {
	LPPS_CALIB = SAFE_RANGE,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [0] = LAYOUT(
				LPPS_CALIB
    ),

    [1] = LAYOUT(
				LPPS_CALIB
    ),

    [2] = LAYOUT(
				LPPS_CALIB
    )

};

void keyboard_post_init_user(void) {
#ifdef CONSOLE_ENABLE
	debug_enable = true;
	debug_mouse = true;
#endif
}

void housekeeping_task_user(void) {
#ifdef CONSOLE_ENABLE
	if (debug_enable && debug_mouse && timer_elapsed(last_log) >= 1000) {
		uint32_t now_report   = lpps_debug_get_report_count;
		uint32_t now_ok       = lpps_debug_read_ok_count;
		uint32_t now_nonzero  = lpps_debug_nonzero_count;
		uprintf("LPPS dbg: report/s=%lu ok/s=%lu nonzero/s=%lu max=(%d,%d,%d)\n",
				(unsigned long)(now_report - last_report),
				(unsigned long)(now_ok - last_ok),
				(unsigned long)(now_nonzero - last_nonzero),
				lpps_debug_max_abs_x,
				lpps_debug_max_abs_y,
				lpps_debug_max_abs_z);

		last_report  = now_report;
		last_ok      = now_ok;
		last_nonzero = now_nonzero;
		last_log = timer_read();

		lpps_debug_max_abs_x = 0;
		lpps_debug_max_abs_y = 0;
		lpps_debug_max_abs_z = 0;
	}
#endif
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
	switch (keycode) {
		case LPPS_CALIB:
			if (record->event.pressed) {
				lpps_request_calibration();
			}
			return false;
		default:
			return true;
	}
}
