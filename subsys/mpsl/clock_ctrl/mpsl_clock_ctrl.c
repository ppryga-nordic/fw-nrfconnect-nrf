/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
// #include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <mpsl_clock.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mpsl_pm_utils, CONFIG_MPSL_LOG_LEVEL);

#define LFCLOCK_TIMEOUT_MS 500

struct clock_onoff_state {
	struct onoff_client cli;
	struct k_sem sem;
};

static atomic_val_t m_hfclk_refcnt;
/* The variable is provided to on-off manager when requesting LFCLK without wait.
 * It must be global to do not corrupt the stack.
 */
static struct onoff_client m_lfclk_cli;

static void m_clock_ready_cb(struct onoff_manager *mgr, struct onoff_client *cli, uint32_t state,
			     int res)
{
	struct clock_onoff_state *clk_state = CONTAINER_OF(cli, struct clock_onoff_state, cli);

	k_sem_give(&clk_state->sem);
}

static int m_clock_ready_blocking_wait(struct onoff_manager *mgr, uint32_t timeout)
{

	struct clock_onoff_state state;
	int err;

	k_sem_init(&state.sem, 0, 1);
	sys_notify_init_callback(&state.cli.notify, m_clock_ready_cb);
	err = onoff_request(mgr, &state.cli);
	if (err < 0) {
		return err;
	}

	return k_sem_take(&state.sem, K_MSEC(timeout));
}

static void m_lfclk_wait(void)
{
	struct onoff_manager *mgr;
	static bool done;
	int err;

	if (done) {
		return;
	}

	done = true;

	mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_LF);
	err = m_clock_ready_blocking_wait(mgr, LFCLOCK_TIMEOUT_MS);
	__ASSERT_NO_MSG(err == 0);

	err = onoff_release(mgr);
	__ASSERT_NO_MSG(err != ONOFF_STATE_ON);
}

static void m_lfclk_calibration_start(void)
{
	z_nrf_clock_calibration_force_start();
}

static bool m_lfclk_calibration_is_enabled(void)
{
	return z_nrf_clock_calibration_is_in_progress();
}

static void m_lfclk_init(void)
{
	struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_LF);

	sys_notify_init_spinwait(&m_lfclk_cli.notify);

	return onoff_request(mgr, &m_lfclk_cli);
}

static void m_lfclk_uninit(void)
{
	struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_LF);

	/* In case there is other ongoing request, cancell it. */
	(void)onoff_cancel(mgr, &m_lfclk_cli);

	return onoff_release(mgr);
}

static void hfclk_request(void)
{
	/* The z_nrf_clock_bt_ctlr_hf_request doesn't count references to HFCLK,
	 * it is caller responsibility handle requests and releases counting.
	 */
	if (atomic_inc(&m_hfclk_refcnt) > 0) {
		return;
	}

	z_nrf_clock_bt_ctlr_hf_request();
}

static void m_hfclk_release(void)
{
	/* The z_nrf_clock_bt_ctlr_hf_request doesn't count references to HFCLK,
	 * it is caller responsibility to do not release the clock if there is
	 * other request pending.
	 */
	if (m_hfclk_refcnt < 1) {
		return;
	}

	if (atomic_dec(&m_hfclk_refcnt) > 1) {
		return;
	}

	z_nrf_clock_bt_ctlr_hf_release();
}

static bool hfclk_is_running(void)
{
	/* As of now assume the HFCLK is runnig after the request was put */
	return ((m_hfclk_refcnt > 0) ? true : false);
}
