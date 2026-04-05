/*
 * task_lora.c — Sole owner of the SX1262 LoRa device.
 *
 * Architecture
 * ────────────
 * A single thread runs a tight loop:
 *   1. Drain lora_cmd_q (non-blocking).
 *   2. If RX is enabled, call lora_recv with a 500 ms timeout.
 *   3. On a successful receive, enqueue an event to lora_evt_q.
 *
 * This gives a worst-case command response latency of ~500 ms, which is
 * acceptable for LoRa data rates.
 *
 * lora_dev_mutex serialises all lora_* API calls between this task and
 * the interactive shell commands below.  Do not use "lora" shell
 * commands concurrently with SMP LoRa RX — the mutex will resolve the
 * conflict but the shell recv will block until the current 500 ms
 * receive cycle finishes.
 *
 * Shell commands (USART2, 115200 baud):
 *   lora status
 *   lora config <freq_hz> <sf:6-12> <bw:125|250|500> <cr:5-8> <preamble> <power_dbm>
 *   lora send <text>
 *   lora recv [timeout_ms]
 *   lora calibrate
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include "lora_ipc.h"

/* Forward-declare the LoRaMac-node HAL function to avoid pulling in the
 * full sx126x.h header (which has loramac-node board-level dependencies). */
extern void SX126xCalibrateImage(uint32_t freq);

LOG_MODULE_REGISTER(lora, LOG_LEVEL_DBG);

#define LORA_NODE DT_NODELABEL(lora)
static const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);

static struct lora_modem_config lora_cfg = {
	.frequency    = 868000000,
	.bandwidth    = BW_125_KHZ,
	.datarate     = SF_10,
	.preamble_len = 8,
	.coding_rate  = CR_4_5,
	.tx_power     = 14,
	.tx           = true,
};

static atomic_t rx_active;

/*
 * Held whenever a lora_* API call is in progress.  Prevents concurrent
 * access between the task thread and shell command handlers.
 */
static K_MUTEX_DEFINE(lora_dev_mutex);

/* ── Message queues ──────────────────────────────────────────────── */

K_MSGQ_DEFINE(lora_cmd_q, sizeof(struct lora_cmd_msg), 2, 4);
K_MSGQ_DEFINE(lora_evt_q, sizeof(struct lora_evt_msg), 2, 4);

/* ── Status accessor ─────────────────────────────────────────────── */

void lora_get_current_status(struct lora_status *out)
{
	k_mutex_lock(&lora_dev_mutex, K_FOREVER);
	out->frequency    = lora_cfg.frequency;
	out->datarate     = (uint8_t)lora_cfg.datarate;
	out->bandwidth    = (uint8_t)lora_cfg.bandwidth;
	out->coding_rate  = (uint8_t)lora_cfg.coding_rate;
	out->preamble_len = lora_cfg.preamble_len;
	out->tx_power     = lora_cfg.tx_power;
	out->rx_active    = atomic_get(&rx_active) != 0;
	k_mutex_unlock(&lora_dev_mutex);
}

/* ── Helpers ─────────────────────────────────────────────────────── */

static const char *bw_str(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_125_KHZ: return "125";
	case BW_250_KHZ: return "250";
	case BW_500_KHZ: return "500";
	default:         return "?";
	}
}

/* ── Command processing (called from the task thread only) ───────── */

static bool rx_configured; /* true when modem is already set up for RX */

static void process_cmd(struct lora_cmd_msg *cmd)
{
	int ret;

	switch (cmd->type) {
	case LORA_CMD_TX:
		k_mutex_lock(&lora_dev_mutex, K_FOREVER);
		lora_cfg.tx = true;
		ret = lora_config(lora_dev, &lora_cfg);
		if (ret == 0) {
			SX126xCalibrateImage(lora_cfg.frequency);
			k_msleep(100);
			lora_config(lora_dev, &lora_cfg);
			k_msleep(100);

			ret = lora_send(lora_dev, cmd->tx_data, cmd->tx_len);
		}
		k_mutex_unlock(&lora_dev_mutex);

		if (ret == 0) {
			LOG_INF("TX: %u byte(s)", cmd->tx_len);
		} else {
			LOG_ERR("TX failed: %d", ret);
		}

		cmd->tx_sync->rc         = ret;
		cmd->tx_sync->bytes_sent = (ret == 0) ? cmd->tx_len : 0;
		k_sem_give(&cmd->tx_sync->done);
		rx_configured = false; /* modem was switched to TX; re-arm RX next cycle */
		break;

	case LORA_CMD_RX_ENABLE:
		atomic_set(&rx_active, 1);
		rx_configured = false;
		LOG_INF("LoRa RX enabled");
		break;

	case LORA_CMD_RX_DISABLE:
		atomic_set(&rx_active, 0);
		LOG_INF("LoRa RX disabled");
		break;
	}
}

/* ── Main task loop ──────────────────────────────────────────────── */

static void task_lora_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa device not ready");
		return;
	}

	k_mutex_lock(&lora_dev_mutex, K_FOREVER);
	int ret = lora_config(lora_dev, &lora_cfg);
	k_mutex_unlock(&lora_dev_mutex);

	if (ret) {
		LOG_ERR("lora_config failed: %d", ret);
		return;
	}

	LOG_INF("SX1262 ready — %u Hz  SF%u  BW%s kHz  %d dBm",
		lora_cfg.frequency, lora_cfg.datarate,
		bw_str(lora_cfg.bandwidth), lora_cfg.tx_power);

	static uint8_t rx_buf[LORA_MAX_PAYLOAD];

	while (1) {
		/* Always drain the command queue before touching the radio */
		struct lora_cmd_msg cmd;

		while (k_msgq_get(&lora_cmd_q, &cmd, K_NO_WAIT) == 0) {
			process_cmd(&cmd);
		}

		if (!atomic_get(&rx_active)) {
			k_sleep(K_MSEC(10));
			continue;
		}

		/* Configure for RX on first entry or after a TX */
		if (!rx_configured) {
			k_mutex_lock(&lora_dev_mutex, K_FOREVER);
			lora_cfg.tx = false;
			ret = lora_config(lora_dev, &lora_cfg);
			k_mutex_unlock(&lora_dev_mutex);

			if (ret != 0) {
				LOG_ERR("lora_config (RX) failed: %d", ret);
				k_sleep(K_MSEC(100));
				continue;
			}
			rx_configured = true;
			LOG_INF("LoRa RX active — %u Hz SF%u",
				lora_cfg.frequency, lora_cfg.datarate);
		}

		int16_t rssi;
		int8_t  snr;

		/*
		 * 500 ms timeout: short enough to check the command queue
		 * regularly, long enough not to hammer the driver.
		 */
		k_mutex_lock(&lora_dev_mutex, K_FOREVER);
		ret = lora_recv(lora_dev, rx_buf, sizeof(rx_buf),
				K_MSEC(500), &rssi, &snr);
		k_mutex_unlock(&lora_dev_mutex);

		if (ret < 0) {
			if (ret != -EAGAIN) {
				LOG_ERR("lora_recv: %d", ret);
			}
			continue;
		}

		LOG_INF("RX: %d byte(s)  RSSI=%d dBm  SNR=%d dB", ret, rssi, snr);

		struct lora_evt_msg evt = {
			.type = LORA_EVT_RX,
			.len  = ret,
			.rssi = rssi,
			.snr  = snr,
		};
		memcpy(evt.data, rx_buf, ret);

		if (k_msgq_put(&lora_evt_q, &evt, K_NO_WAIT) != 0) {
			LOG_WRN("RX event queue full — packet dropped");
		}
	}
}

K_THREAD_DEFINE(task_lora_tid, 2048,
		task_lora_entry, NULL, NULL, NULL,
		7, 0, 500 /* ms startup delay so shell is up first */);

/* ── Shell commands ──────────────────────────────────────────────── */

static int cmd_lora_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	shell_print(sh, "Device ready : %s", device_is_ready(lora_dev) ? "yes" : "no");
	shell_print(sh, "RX active    : %s", atomic_get(&rx_active) ? "yes" : "no");
	shell_print(sh, "Frequency    : %u Hz", lora_cfg.frequency);
	shell_print(sh, "Bandwidth    : %s kHz", bw_str(lora_cfg.bandwidth));
	shell_print(sh, "Spreading    : SF%u", lora_cfg.datarate);
	shell_print(sh, "Coding rate  : CR 4/%u", lora_cfg.coding_rate + 4);
	shell_print(sh, "Preamble     : %u symbols", lora_cfg.preamble_len);
	shell_print(sh, "TX power     : %d dBm", lora_cfg.tx_power);
	return 0;
}

/*
 * lora config <freq_hz> <sf> <bw_khz> <cr_denom> <preamble> <power_dbm>
 * Example: lora config 868000000 10 125 5 8 14
 */
static int cmd_lora_config(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 7) {
		shell_error(sh, "Usage: lora config <freq_hz> <sf:6-12>"
			    " <bw:125|250|500> <cr:5-8> <preamble> <power_dbm>");
		return -EINVAL;
	}

	uint32_t freq  = (uint32_t)strtoul(argv[1], NULL, 10);
	int      sf    = (int)strtoul(argv[2], NULL, 10);
	int      bw    = (int)strtoul(argv[3], NULL, 10);
	int      cr    = (int)strtoul(argv[4], NULL, 10);
	int      pream = (int)strtoul(argv[5], NULL, 10);
	int      pwr   = (int)strtol (argv[6], NULL, 10);

	if (sf < 6 || sf > 12) {
		shell_error(sh, "SF must be 6..12"); return -EINVAL;
	}
	if (bw != 125 && bw != 250 && bw != 500) {
		shell_error(sh, "BW must be 125, 250, or 500"); return -EINVAL;
	}
	if (cr < 5 || cr > 8) {
		shell_error(sh, "CR denominator must be 5..8"); return -EINVAL;
	}

	k_mutex_lock(&lora_dev_mutex, K_FOREVER);
	lora_cfg.frequency    = freq;
	lora_cfg.datarate     = (enum lora_datarate)sf;
	lora_cfg.bandwidth    = (bw == 125) ? BW_125_KHZ
			      : (bw == 250) ? BW_250_KHZ : BW_500_KHZ;
	lora_cfg.coding_rate  = (enum lora_coding_rate)(cr - 4);
	lora_cfg.preamble_len = (uint16_t)pream;
	lora_cfg.tx_power     = (int8_t)pwr;

	int ret = lora_config(lora_dev, &lora_cfg);
	k_mutex_unlock(&lora_dev_mutex);

	if (ret) {
		shell_error(sh, "lora_config failed: %d", ret);
		return ret;
	}
	rx_configured = false; /* force re-arm on next RX cycle */
	shell_print(sh, "Config applied: %u Hz  SF%u  BW%s kHz  CR 4/%u  preamble=%u  %d dBm",
		    lora_cfg.frequency, lora_cfg.datarate,
		    bw_str(lora_cfg.bandwidth), cr, pream, pwr);
	return 0;
}

/* lora send <text> */
static int cmd_lora_send(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: lora send <text>");
		return -EINVAL;
	}

	k_mutex_lock(&lora_dev_mutex, K_FOREVER);
	lora_cfg.tx = true;
	int ret = lora_config(lora_dev, &lora_cfg);
	if (ret == 0) {
		size_t len = strlen(argv[1]);

		SX126xCalibrateImage(lora_cfg.frequency);
		k_msleep(100);
		lora_config(lora_dev, &lora_cfg);
		k_msleep(100);

		ret = lora_send(lora_dev, (uint8_t *)argv[1], len);
		if (ret == 0) {
			shell_print(sh, "Sent %u byte(s): \"%s\"", (unsigned)len, argv[1]);
		} else {
			shell_error(sh, "lora_send failed: %d", ret);
		}
	} else {
		shell_error(sh, "lora_config failed: %d", ret);
	}
	k_mutex_unlock(&lora_dev_mutex);
	rx_configured = false;
	return ret;
}

/* lora calibrate — run image calibration for the current frequency */
static int cmd_lora_calibrate(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	k_mutex_lock(&lora_dev_mutex, K_FOREVER);
	lora_cfg.tx = true;
	int ret = lora_config(lora_dev, &lora_cfg);
	k_mutex_unlock(&lora_dev_mutex);

	if (ret) {
		shell_error(sh, "lora_config failed: %d", ret);
		return ret;
	}

	SX126xCalibrateImage(lora_cfg.frequency);
	shell_print(sh, "Image calibration done for %u Hz", lora_cfg.frequency);
	rx_configured = false;
	return 0;
}

/* lora recv [timeout_ms] — blocks until packet or timeout */
static int cmd_lora_recv(const struct shell *sh, size_t argc, char **argv)
{
	int timeout_ms = (argc >= 2) ? (int)strtoul(argv[1], NULL, 10) : 5000;

	if (atomic_get(&rx_active)) {
		shell_warn(sh, "SMP RX is active; result may be unreliable");
	}

	/* Configure for RX (mutex held only for the config call) */
	k_mutex_lock(&lora_dev_mutex, K_FOREVER);
	lora_cfg.tx = false;
	int ret = lora_config(lora_dev, &lora_cfg);
	k_mutex_unlock(&lora_dev_mutex);

	if (ret) {
		shell_error(sh, "lora_config failed: %d", ret);
		return ret;
	}

	static uint8_t buf[256];
	int16_t rssi;
	int8_t  snr;

	shell_print(sh, "Listening for %d ms...", timeout_ms);

	/*
	 * Mutex is held for the full receive duration to prevent the task
	 * thread from reconfiguring the modem underneath us.
	 */
	k_mutex_lock(&lora_dev_mutex, K_FOREVER);
	ret = lora_recv(lora_dev, buf, sizeof(buf) - 1,
			K_MSEC(timeout_ms), &rssi, &snr);
	k_mutex_unlock(&lora_dev_mutex);

	rx_configured = false;

	if (ret < 0) {
		shell_error(sh, "lora_recv: %d%s", ret,
			    ret == -EAGAIN ? " (timeout)" : "");
		return ret;
	}

	buf[ret] = '\0';
	shell_print(sh, "Received %d byte(s): \"%s\"  RSSI=%d dBm  SNR=%d dB",
		    ret, buf, rssi, snr);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(lora_sub,
	SHELL_CMD(status,    NULL,
		  "Show device state and current modem config",
		  cmd_lora_status),
	SHELL_CMD(config,    NULL,
		  "Set modem params: config <freq_hz> <sf> <bw> <cr> <preamble> <power>",
		  cmd_lora_config),
	SHELL_CMD(calibrate, NULL,
		  "Run image calibration for the current frequency",
		  cmd_lora_calibrate),
	SHELL_CMD(send,      NULL,
		  "Transmit a packet: send <text>",
		  cmd_lora_send),
	SHELL_CMD(recv,      NULL,
		  "Receive a packet (blocking): recv [timeout_ms]",
		  cmd_lora_recv),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(lora, &lora_sub, "LoRa transceiver (SX1262)", NULL);
