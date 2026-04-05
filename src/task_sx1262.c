/*
 * task_sx1262.c — SX1262 LoRa transceiver via Zephyr LoRa API
 *
 * DT node  : lora: lora@0 on spi2 (compatible = "semtech,sx1262")
 * Shell cmds:
 *   lora status
 *   lora config <freq_hz> <sf:6-12> <bw:125|250|500> <cr:5-8> <preamble> <power_dbm>
 *   lora send <text>
 *   lora recv [timeout_ms]
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

/* Forward-declare the LoRaMac-node HAL function to avoid pulling in the full
 * sx126x.h header (which has loramac-node board-level dependencies). */
extern void SX126xCalibrateImage(uint32_t freq);

LOG_MODULE_REGISTER(sx1262, LOG_LEVEL_DBG);

#define LORA_NODE DT_NODELABEL(lora)

static const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);

/* Working modem config — modified by "lora config" command */
static struct lora_modem_config lora_cfg = {
	.frequency    = 868000000,   /* 868 MHz */
	.bandwidth    = BW_125_KHZ,
	.datarate     = SF_10,
	.preamble_len = 8,
	.coding_rate  = CR_4_5,
	.tx_power     = 14,          /* dBm */
	.tx           = true,
};

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

/* Apply lora_cfg with the requested tx/rx direction. */
static int apply_config(bool tx, const struct shell *sh)
{
	lora_cfg.tx = tx;
	int ret = lora_config(lora_dev, &lora_cfg);
	if (ret && sh) {
		shell_error(sh, "lora_config failed: %d", ret);
	}
	return ret;
}

/* ── Startup thread ──────────────────────────────────────────────── */

static void task_sx1262_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa device %s not ready", lora_dev->name);
		return;
	}

	int ret = lora_config(lora_dev, &lora_cfg);
	if (ret) {
		LOG_ERR("lora_config failed: %d", ret);
		return;
	}

	LOG_INF("SX1262 ready — %u Hz  SF%u  BW%s kHz  %d dBm",
		lora_cfg.frequency, lora_cfg.datarate,
		bw_str(lora_cfg.bandwidth), lora_cfg.tx_power);

	while (1) {
		k_sleep(K_SECONDS(10));
	}
}

K_THREAD_DEFINE(task_sx1262, 1536,
		task_sx1262_entry, NULL, NULL, NULL,
		7, 0, 500 /* ms delay so shell is up first */);

/* ── Shell commands ──────────────────────────────────────────────── */

static int cmd_lora_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Device ready : %s", device_is_ready(lora_dev) ? "yes" : "no");
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

	lora_cfg.frequency    = freq;
	lora_cfg.datarate     = (enum lora_datarate)sf; /* SF_6=6 .. SF_12=12 */
	lora_cfg.bandwidth    = (bw == 125) ? BW_125_KHZ
			      : (bw == 250) ? BW_250_KHZ : BW_500_KHZ;
	lora_cfg.coding_rate  = (enum lora_coding_rate)(cr - 4); /* CR_4_5=1..CR_4_8=4 */
	lora_cfg.preamble_len = (uint16_t)pream;
	lora_cfg.tx_power     = (int8_t)pwr;

	int ret = lora_config(lora_dev, &lora_cfg);
	if (ret) {
		shell_error(sh, "lora_config failed: %d", ret);
		return ret;
	}
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

	int ret = apply_config(true, sh);
	if (ret) {
		return ret;
	}

	size_t len = strlen(argv[1]);
	ret = lora_send(lora_dev, (uint8_t *)argv[1], len);
	if (ret) {
		shell_error(sh, "lora_send failed: %d", ret);
		return ret;
	}
	shell_print(sh, "Sent %u byte(s): \"%s\"", (unsigned)len, argv[1]);
	return 0;
}

/* lora calibrate — run image calibration for the current frequency */
static int cmd_lora_calibrate(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Wake modem and apply current config so the chip is in standby */
	int ret = apply_config(true, sh);
	if (ret) {
		return ret;
	}

	SX126xCalibrateImage(lora_cfg.frequency);
	shell_print(sh, "Image calibration done for %u Hz", lora_cfg.frequency);
	return 0;
}

/* lora recv [timeout_ms]  — blocks until packet or timeout */
static int cmd_lora_recv(const struct shell *sh, size_t argc, char **argv)
{
	int timeout_ms = (argc >= 2) ? (int)strtoul(argv[1], NULL, 10) : 5000;

	int ret = apply_config(false, sh);
	if (ret) {
		return ret;
	}

	static uint8_t buf[256];
	int16_t rssi;
	int8_t  snr;

	shell_print(sh, "Listening for %d ms...", timeout_ms);
	ret = lora_recv(lora_dev, buf, sizeof(buf) - 1, K_MSEC(timeout_ms), &rssi, &snr);
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
	SHELL_CMD(status, NULL,
		  "Show device state and current modem config",
		  cmd_lora_status),
	SHELL_CMD(config, NULL,
		  "Set modem params: config <freq_hz> <sf> <bw> <cr> <preamble> <power>",
		  cmd_lora_config),
	SHELL_CMD(calibrate, NULL,
		  "Run image calibration for the current frequency",
		  cmd_lora_calibrate),
	SHELL_CMD(send, NULL,
		  "Transmit a packet: send <text>",
		  cmd_lora_send),
	SHELL_CMD(recv, NULL,
		  "Receive a packet (blocking): recv [timeout_ms]",
		  cmd_lora_recv),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(lora, &lora_sub, "LoRa transceiver (SX1262)", NULL);
