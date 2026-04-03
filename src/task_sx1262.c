/*
 * task_sx1262.c — SX1262 LoRa transceiver task + shell commands
 *
 * Hardware assumptions (adjust to schematic):
 *   SPI1  PA4=NSS  PA5=SCK  PA6=MISO  PA7=MOSI
 *   PB0   NRESET  (active-low)
 *   PB1   BUSY    (active-high, high = chip is processing)
 *
 * SX1262 SPI protocol (mode 0, MSB-first):
 *   ReadRegister  0x1D addr_hi addr_lo NOP    → status NOP NOP data
 *   WriteRegister 0x0D addr_hi addr_lo data…
 *   GetStatus     0xC0 NOP              → NOP  status
 *
 * Status byte: bits[6:4] = ChipMode, bits[3:1] = CmdStatus
 *   ChipMode: 2=STDBY_RC 3=STDBY_XOSC 4=FS 5=RX 6=TX
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sx1262, LOG_LEVEL_DBG);

/* ── Device-tree handles ─────────────────────────────────────────── */

#define SX1262_NODE DT_NODELABEL(sx1262)
#define USER_NODE   DT_PATH(zephyr_user)

static const struct spi_dt_spec sx1262_spi = SPI_DT_SPEC_GET(
	SX1262_NODE,
	SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	0);

static const struct gpio_dt_spec sx1262_busy =
	GPIO_DT_SPEC_GET(USER_NODE, sx1262_busy_gpios);
static const struct gpio_dt_spec sx1262_reset =
	GPIO_DT_SPEC_GET(USER_NODE, sx1262_reset_gpios);
static const struct gpio_dt_spec sx1262_dio1 =
	GPIO_DT_SPEC_GET(USER_NODE, sx1262_dio1_gpios);

/* ── SX1262 opcodes ──────────────────────────────────────────────── */

#define CMD_GET_STATUS      0xC0u
#define CMD_READ_REGISTER   0x1Du
#define CMD_WRITE_REGISTER  0x0Du

/* ── Low-level helpers ───────────────────────────────────────────── */

static int sx1262_wait_busy(void)
{
	/* BUSY high → chip is processing; wait up to 100 ms */
	for (int i = 0; i < 100; i++) {
		if (!gpio_pin_get_dt(&sx1262_busy)) {
			return 0;
		}
		k_msleep(1);
	}
	/* Warn but proceed — lets SPI fire even with wrong BUSY pin so we
	 * can distinguish a pin-assignment problem from a bus problem. */
	LOG_WRN("BUSY timeout — pin may be misconfigured, attempting SPI anyway");
	return 0;
}

static int sx1262_get_status(uint8_t *out_status)
{
	uint8_t tx[2] = {CMD_GET_STATUS, 0x00};
	uint8_t rx[2] = {0};
	struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	sx1262_wait_busy();
	int ret = spi_transceive_dt(&sx1262_spi, &tx_set, &rx_set);
	if (ret == 0 && out_status) {
		*out_status = rx[1];
	}
	return ret;
}

static int sx1262_read_reg(uint16_t addr, uint8_t *data)
{
	/* TX: opcode addr_hi addr_lo NOP(status) NOP(data)  */
	uint8_t tx[5] = {CMD_READ_REGISTER, addr >> 8, addr & 0xFF, 0x00, 0x00};
	uint8_t rx[5] = {0};
	struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	sx1262_wait_busy();
	int ret = spi_transceive_dt(&sx1262_spi, &tx_set, &rx_set);
	if (ret == 0 && data) {
		*data = rx[4];
	}
	return ret;
}

static int sx1262_write_reg(uint16_t addr, uint8_t data)
{
	uint8_t tx[4] = {CMD_WRITE_REGISTER, addr >> 8, addr & 0xFF, data};
	struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

	sx1262_wait_busy();
	return spi_write_dt(&sx1262_spi, &tx_set);
}

static void sx1262_hw_reset(void)
{
	gpio_pin_set_dt(&sx1262_reset, 1); /* assert NRESET low */
	k_usleep(300);
	gpio_pin_set_dt(&sx1262_reset, 0); /* release */
	/* chip holds BUSY high for ~3 ms while booting */
	k_msleep(5);
	sx1262_wait_busy();
}

/* ── Chip identification ─────────────────────────────────────────── */

/*
 * SX1262 has no explicit chip-ID register.  Identification is done by:
 *   1. Verifying GetStatus returns a valid ChipMode (2 = STDBY_RC).
 *   2. Reading the LoRa sync-word registers 0x0740/0x0741 whose reset
 *      values are 0x14 / 0x24 (private-network LoRa sync word).
 */
static void sx1262_identify(void)
{
	uint8_t status;
	if (sx1262_get_status(&status)) {
		LOG_ERR("SX1262: GetStatus SPI error");
		return;
	}
	uint8_t chip_mode  = (status >> 4) & 0x7u;
	uint8_t cmd_status = (status >> 1) & 0x7u;

	static const char *const mode_str[] = {
		"?", "?", "STDBY_RC", "STDBY_XOSC", "FS", "RX", "TX", "?"
	};
	LOG_INF("SX1262 GetStatus=0x%02X  ChipMode=%s(%u)  CmdStatus=%u",
		status, mode_str[chip_mode & 0x7], chip_mode, cmd_status);

	uint8_t sw_msb = 0, sw_lsb = 0;
	if (!sx1262_read_reg(0x0740, &sw_msb) &&
	    !sx1262_read_reg(0x0741, &sw_lsb)) {
		LOG_INF("SX1262 LoRa sync word: 0x%02X%02X  (expect 0x1424 after reset)",
			sw_msb, sw_lsb);
		if (sw_msb == 0x14 && sw_lsb == 0x24) {
			LOG_INF("SX1262: chip identified OK");
		} else {
			LOG_WRN("SX1262: unexpected sync word — check wiring / chip variant");
		}
	}
}

/* ── Task thread ─────────────────────────────────────────────────── */

static void task_sx1262_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!gpio_is_ready_dt(&sx1262_reset) ||
	    !gpio_is_ready_dt(&sx1262_busy)  ||
	    !gpio_is_ready_dt(&sx1262_dio1)) {
		LOG_ERR("SX1262: GPIO device not ready");
		return;
	}
	gpio_pin_configure_dt(&sx1262_busy,  GPIO_INPUT);
	gpio_pin_configure_dt(&sx1262_reset, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&sx1262_dio1,  GPIO_INPUT);

	if (!spi_is_ready_dt(&sx1262_spi)) {
		LOG_ERR("SX1262: SPI bus not ready");
		return;
	}

	sx1262_hw_reset();
	sx1262_identify();

	/* Thread is now idle; register operations are driven by shell. */
	while (1) {
		k_sleep(K_SECONDS(10));
	}
}

K_THREAD_DEFINE(task_sx1262, 2048,
		task_sx1262_entry, NULL, NULL, NULL,
		5, 0, 500 /* ms delay so shell is up first */);

/* ── Shell commands ──────────────────────────────────────────────── */

static int cmd_sx1262_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint8_t status;
	int ret = sx1262_get_status(&status);
	if (ret) {
		shell_error(sh, "SPI error: %d", ret);
		return ret;
	}

	uint8_t chip_mode  = (status >> 4) & 0x7u;
	uint8_t cmd_status = (status >> 1) & 0x7u;
	static const char *const mode_str[] = {
		"?", "?", "STDBY_RC", "STDBY_XOSC", "FS", "RX", "TX", "?"
	};
	shell_print(sh, "Status=0x%02X  ChipMode=%s(%u)  CmdStatus=%u",
		    status, mode_str[chip_mode & 0x7], chip_mode, cmd_status);
	return 0;
}

static int cmd_sx1262_read(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: sx1262 read <addr_hex> [count]");
		return -EINVAL;
	}

	uint16_t addr  = (uint16_t)strtoul(argv[1], NULL, 16);
	int      count = (argc >= 3) ? (int)strtoul(argv[2], NULL, 0) : 1;

	if (count < 1 || count > 64) {
		shell_error(sh, "count must be 1..64");
		return -EINVAL;
	}

	for (int i = 0; i < count; i++) {
		uint8_t data;
		int ret = sx1262_read_reg(addr + i, &data);
		if (ret) {
			shell_error(sh, "SPI error at 0x%04X: %d", addr + i, ret);
			return ret;
		}
		shell_print(sh, "Reg[0x%04X] = 0x%02X", addr + i, data);
	}
	return 0;
}

static int cmd_sx1262_write(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: sx1262 write <addr_hex> <data_hex>");
		return -EINVAL;
	}

	uint16_t addr = (uint16_t)strtoul(argv[1], NULL, 16);
	uint8_t  data = (uint8_t) strtoul(argv[2], NULL, 16);

	int ret = sx1262_write_reg(addr, data);
	if (ret) {
		shell_error(sh, "SPI error: %d", ret);
		return ret;
	}
	shell_print(sh, "Reg[0x%04X] <- 0x%02X  OK", addr, data);
	return 0;
}

static int cmd_sx1262_reset_cmd(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	sx1262_hw_reset();
	shell_print(sh, "SX1262 hardware reset done");
	return 0;
}

static int cmd_sx1262_id(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint8_t status;
	if (sx1262_get_status(&status)) {
		shell_error(sh, "SPI error reading status");
		return -EIO;
	}

	uint8_t sw_msb = 0, sw_lsb = 0;
	int ret = sx1262_read_reg(0x0740, &sw_msb) ||
		  sx1262_read_reg(0x0741, &sw_lsb);

	shell_print(sh, "Status     : 0x%02X (ChipMode=%u CmdStatus=%u)",
		    status, (status >> 4) & 0x7u, (status >> 1) & 0x7u);
	if (!ret) {
		shell_print(sh, "SyncWord   : 0x%02X%02X  (%s)",
			    sw_msb, sw_lsb,
			    (sw_msb == 0x14 && sw_lsb == 0x24)
				? "OK — private LoRa network default"
				: "unexpected value");
	}
	return 0;
}

static int cmd_sx1262_gpio(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int busy  = gpio_pin_get_dt(&sx1262_busy);
	int reset = gpio_pin_get_dt(&sx1262_reset);
	int dio1  = gpio_pin_get_dt(&sx1262_dio1);

	shell_print(sh, "BUSY  PB%-2d : %d  %s", sx1262_busy.pin,  busy,
		    busy  ? "(HIGH — chip busy / pin stuck?)" : "(LOW  — ready)");
	shell_print(sh, "RESET PB%-2d : %d  %s", sx1262_reset.pin, reset,
		    reset ? "(asserted — chip in reset!)" : "(deasserted — normal)");
	shell_print(sh, "DIO1  PA%-2d : %d", sx1262_dio1.pin, dio1);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sx1262_sub,
	SHELL_CMD(id,     NULL, "Identify chip (status + sync-word check)", cmd_sx1262_id),
	SHELL_CMD(status, NULL, "Read chip status byte",                    cmd_sx1262_status),
	SHELL_CMD(read,   NULL, "Read register(s): read <addr> [count]",   cmd_sx1262_read),
	SHELL_CMD(write,  NULL, "Write register:   write <addr> <data>",   cmd_sx1262_write),
	SHELL_CMD(reset,  NULL, "Hardware reset",                           cmd_sx1262_reset_cmd),
	SHELL_CMD(gpio,   NULL, "Show raw BUSY/RESET pin states",           cmd_sx1262_gpio),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sx1262, &sx1262_sub, "SX1262 LoRa transceiver", NULL);
