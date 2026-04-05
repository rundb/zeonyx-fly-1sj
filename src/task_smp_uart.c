/*
 * task_smp_uart.c — MCUMgr SMP coprocessor interface over USART1
 *
 * Physical layer : USART1, PA9 = TX, PA10 = RX, 115200 baud
 * Protocol       : MCUMgr SMP with UART serial framing (base64 + CRC16)
 *
 * This task is responsible only for the SMP transport layer and the
 * host communication protocol.  All LoRa operations are delegated to
 * task_lora via two message queues:
 *
 *   lora_cmd_q  (this task → task_lora)  TX / RX_ENABLE / RX_DISABLE
 *   lora_evt_q  (task_lora → this task)  RX packets (forwarded as
 *                                         unsolicited SMP notifications)
 *
 * The smp_evt_thread blocks on lora_evt_q and converts each incoming
 * RX event into an SMP write-response frame sent over USART1.
 *
 * Management group: LORA_SMP_GRP  (ID 64, first application-defined group)
 *
 *   CMD   Direction   Operation
 *   ───   ─────────   ─────────────────────────────────────────────────
 *    0    READ        get_version  → {version:"1.0.0", board:"1sj_module"}
 *    1    READ        get_status   → {freq, sf, bw_khz, cr, preamble,
 *                                     tx_power, rx_active}
 *    2    WRITE ←{}   rx_enable    → {rc:0}
 *    3    WRITE ←{}   rx_disable   → {rc:0}
 *    4    WRITE ←{msg:<text|bytes>}
 *                     tx_msg       → {rc:0, bytes_sent:N}
 *    5-7  —           reserved
 *    8    notification (device→host only)
 *                     rx_notify    ← {data:<bstr>, rssi:<int>,
 *                                     snr:<int>, len:<int>}
 *
 * Shell commands (USART2, 115200 baud):
 *   smp status          — show SMP interface state
 *   smp rx [on|off]     — enable / disable LoRa RX via the SMP path
 *   smp send <text>     — transmit a LoRa packet via the SMP task
 *   smp reset           — reset sequence counter
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/base64.h>
#include <zephyr/sys/crc.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zcbor_common.h>
#include <zcbor_encode.h>
#include <zcbor_decode.h>
#include <mgmt/mcumgr/util/zcbor_bulk.h>
#include "lora_ipc.h"

LOG_MODULE_REGISTER(smp_uart, LOG_LEVEL_INF);

/* ── Constants ───────────────────────────────────────────────────── */

#define LORA_SMP_GRP            64

#define LORA_CMD_GET_VERSION    0
#define LORA_CMD_GET_STATUS     1
#define LORA_CMD_RX_ENABLE      2
#define LORA_CMD_RX_DISABLE     3
#define LORA_CMD_TX_MSG         4
#define LORA_CMD_RX_NOTIFY      8   /* device→host only, no handler */

#define FW_VERSION_STR  "1.0.0"
#define BOARD_STR       "1sj_module"

/*
 * MCUMgr UART serial framing splits the raw packet into chunks of at
 * most SERIAL_CHUNK_MAX bytes before base64 encoding.
 * 93 raw bytes → 124 base64 chars; adding the 2-byte frame header and
 * '\n' keeps the line ≤ 127 bytes (the protocol limit).
 */
#define SERIAL_CHUNK_MAX  93

/* ── Device handles ──────────────────────────────────────────────── */

static const struct device *usart1_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));

/* ── State ───────────────────────────────────────────────────────── */

static atomic_t   notify_seq;            /* rolling notification seq# */
static K_MUTEX_DEFINE(usart1_tx_mutex); /* serialises all USART1 TX */

/* ── BW helper ───────────────────────────────────────────────────── */

static uint16_t bw_to_khz(uint8_t bw)
{
	switch ((enum lora_signal_bandwidth)bw) {
	case BW_125_KHZ: return 125;
	case BW_250_KHZ: return 250;
	case BW_500_KHZ: return 500;
	default:         return 0;
	}
}

/* ── MCUMgr command handlers ─────────────────────────────────────── */

static int lora_cmd_get_version(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;

	bool ok = zcbor_tstr_put_lit(zse, "version") &&
		  zcbor_tstr_put_lit(zse, FW_VERSION_STR) &&
		  zcbor_tstr_put_lit(zse, "board") &&
		  zcbor_tstr_put_lit(zse, BOARD_STR);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
}

static int lora_cmd_get_status(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	struct lora_status st;

	lora_get_current_status(&st);

	bool ok =
		zcbor_tstr_put_lit(zse, "freq")      &&
		zcbor_uint32_put(zse, st.frequency)  &&
		zcbor_tstr_put_lit(zse, "sf")         &&
		zcbor_uint32_put(zse, (uint32_t)st.datarate) &&
		zcbor_tstr_put_lit(zse, "bw_khz")     &&
		zcbor_uint32_put(zse, bw_to_khz(st.bandwidth)) &&
		zcbor_tstr_put_lit(zse, "cr")         &&
		zcbor_uint32_put(zse, (uint32_t)st.coding_rate + 4) &&
		zcbor_tstr_put_lit(zse, "preamble")   &&
		zcbor_uint32_put(zse, st.preamble_len) &&
		zcbor_tstr_put_lit(zse, "tx_power")   &&
		zcbor_int32_put(zse, st.tx_power)     &&
		zcbor_tstr_put_lit(zse, "rx_active")  &&
		zcbor_bool_put(zse, st.rx_active);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
}

static int lora_cmd_rx_enable(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;

	struct lora_cmd_msg cmd = { .type = LORA_CMD_RX_ENABLE };

	if (k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100)) != 0) {
		LOG_WRN("SMP: lora_cmd_q full, rx_enable dropped");
	} else {
		LOG_INF("SMP: LoRa RX enable queued");
	}

	bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0);
	return ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
}

static int lora_cmd_rx_disable(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;

	struct lora_cmd_msg cmd = { .type = LORA_CMD_RX_DISABLE };

	if (k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100)) != 0) {
		LOG_WRN("SMP: lora_cmd_q full, rx_disable dropped");
	} else {
		LOG_INF("SMP: LoRa RX disable queued");
	}

	bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0);
	return ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
}

static int lora_cmd_tx_msg(struct smp_streamer *ctxt)
{
	zcbor_state_t *zsd = ctxt->reader->zs;
	zcbor_state_t *zse = ctxt->writer->zs;
	struct zcbor_string msg = { 0 };
	size_t decoded = 0;

	struct zcbor_map_decode_key_val tx_decode[] = {
		ZCBOR_MAP_DECODE_KEY_DECODER("msg", zcbor_tstr_decode, &msg),
	};

	if (zcbor_map_decode_bulk(zsd, tx_decode, ARRAY_SIZE(tx_decode),
				  &decoded) != 0 || decoded == 0 || msg.len == 0) {
		bool err_ok = zcbor_tstr_put_lit(zse, "rc") &&
			      zcbor_int32_put(zse, MGMT_ERR_EINVAL);
		return err_ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
	}

	if (msg.len > LORA_MAX_PAYLOAD) {
		bool err_ok = zcbor_tstr_put_lit(zse, "rc") &&
			      zcbor_int32_put(zse, MGMT_ERR_EINVAL);
		return err_ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
	}

	/*
	 * Build the TX command with a sync block on our stack.
	 * We block on sync.done until task_lora completes the transmission
	 * and fills rc/bytes_sent.  The stack frame (and sync) remain alive
	 * for the entire wait, so the pointer in cmd.tx_sync is valid.
	 */
	struct lora_tx_sync sync;

	k_sem_init(&sync.done, 0, 1);

	struct lora_cmd_msg cmd = {
		.type    = LORA_CMD_TX,
		.tx_len  = (uint16_t)msg.len,
		.tx_sync = &sync,
	};
	memcpy(cmd.tx_data, msg.value, msg.len);

	if (k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100)) != 0) {
		LOG_ERR("SMP TX: command queue full");
		bool err_ok = zcbor_tstr_put_lit(zse, "rc") &&
			      zcbor_int32_put(zse, MGMT_ERR_EBUSY);
		return err_ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
	}

	/* Wait for task_lora to finish transmission (up to 10 s) */
	k_sem_take(&sync.done, K_MSEC(10000));

	if (sync.rc == 0) {
		LOG_INF("SMP TX: %u byte(s)", (unsigned)msg.len);
	} else {
		LOG_ERR("SMP TX failed: %d", sync.rc);
	}

	bool ok = zcbor_tstr_put_lit(zse, "rc") &&
		  zcbor_int32_put(zse, sync.rc) &&
		  zcbor_tstr_put_lit(zse, "bytes_sent") &&
		  zcbor_uint32_put(zse, sync.bytes_sent);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_ENOMEM;
}

/* ── MCUMgr group registration ───────────────────────────────────── */

static const struct mgmt_handler lora_mgmt_handlers[] = {
	[LORA_CMD_GET_VERSION] = { .mh_read  = lora_cmd_get_version },
	[LORA_CMD_GET_STATUS]  = { .mh_read  = lora_cmd_get_status  },
	[LORA_CMD_RX_ENABLE]   = { .mh_write = lora_cmd_rx_enable   },
	[LORA_CMD_RX_DISABLE]  = { .mh_write = lora_cmd_rx_disable  },
	[LORA_CMD_TX_MSG]      = { .mh_write = lora_cmd_tx_msg      },
	[5] = { .mh_read = NULL, .mh_write = NULL },
	[6] = { .mh_read = NULL, .mh_write = NULL },
	[7] = { .mh_read = NULL, .mh_write = NULL },
	/* CMD 8 (LORA_CMD_RX_NOTIFY) is device→host only; no handler entry. */
};

static struct mgmt_group lora_mgmt_group = {
	.mg_handlers       = lora_mgmt_handlers,
	.mg_handlers_count = ARRAY_SIZE(lora_mgmt_handlers),
	.mg_group_id       = LORA_SMP_GRP,
};

/* ── Unsolicited notification sender ─────────────────────────────── */

/*
 * Encodes a complete MCUMgr UART serial frame and sends it to the host.
 *
 * Wire format (raw bytes before base64 chunking):
 *   [uint16_be(pkt_len)][smp_header 8B][cbor_payload][uint16_be(crc16)]
 *
 * Each chunk of ≤93 raw bytes is base64-encoded and emitted as:
 *   first chunk  : 0x06 0x09 <base64> 0x0A
 *   continuation : 0x04 0x14 <base64> 0x0A
 */
static void smp_send_notification(uint8_t cmd_id,
				   const uint8_t *cbor, size_t cbor_len)
{
	uint8_t hdr[8];

	hdr[0] = MGMT_OP_WRITE_RSP;
	hdr[1] = 0;
	sys_put_be16((uint16_t)cbor_len, &hdr[2]);
	sys_put_be16(LORA_SMP_GRP, &hdr[4]);
	hdr[6] = (uint8_t)atomic_inc(&notify_seq);
	hdr[7] = cmd_id;

	uint16_t crc = crc16_itu_t(0, hdr, sizeof(hdr));
	crc = crc16_itu_t(crc, cbor, cbor_len);

	static uint8_t frame_raw[280];
	size_t pkt_len = sizeof(hdr) + cbor_len;

	if (pkt_len + 4 > sizeof(frame_raw)) {
		LOG_ERR("SMP notify: payload too large (%zu bytes)", pkt_len);
		return;
	}

	size_t fi = 0;

	sys_put_be16((uint16_t)pkt_len, frame_raw + fi);  fi += 2;
	memcpy(frame_raw + fi, hdr, sizeof(hdr));          fi += sizeof(hdr);
	memcpy(frame_raw + fi, cbor, cbor_len);            fi += cbor_len;
	sys_put_be16(crc, frame_raw + fi);                 fi += 2;

	k_mutex_lock(&usart1_tx_mutex, K_FOREVER);

	bool first = true;
	size_t offset = 0;

	while (offset < fi) {
		size_t chunk = MIN(SERIAL_CHUNK_MAX, fi - offset);
		uint8_t b64[128];
		size_t b64_len = 0;

		if (base64_encode(b64, sizeof(b64), &b64_len,
				  frame_raw + offset, chunk) != 0) {
			LOG_ERR("SMP notify: base64 error");
			break;
		}

		uart_poll_out(usart1_dev, first ? 0x06 : 0x04);
		uart_poll_out(usart1_dev, first ? 0x09 : 0x14);
		first = false;

		for (size_t i = 0; i < b64_len; i++) {
			uart_poll_out(usart1_dev, b64[i]);
		}
		uart_poll_out(usart1_dev, '\n');

		offset += chunk;
	}

	k_mutex_unlock(&usart1_tx_mutex);
}

/* Encodes a LoRa RX event and sends it as an SMP notification (CMD 8). */
static void send_rx_notification(const struct lora_evt_msg *evt)
{
	uint8_t cbor_buf[256];
	zcbor_state_t zse[3];

	zcbor_new_encode_state(zse, ARRAY_SIZE(zse),
			       cbor_buf, sizeof(cbor_buf), 1);

	struct zcbor_string bstr = {
		.value = evt->data,
		.len   = (size_t)evt->len,
	};

	bool ok = zcbor_map_start_encode(zse, 10) &&
		  zcbor_tstr_put_lit(zse, "data") && zcbor_bstr_encode(zse, &bstr) &&
		  zcbor_tstr_put_lit(zse, "rssi") && zcbor_int32_put(zse, evt->rssi) &&
		  zcbor_tstr_put_lit(zse, "snr")  && zcbor_int32_put(zse, evt->snr)  &&
		  zcbor_tstr_put_lit(zse, "len")  && zcbor_uint32_put(zse, (uint32_t)evt->len) &&
		  zcbor_map_end_encode(zse, 10);

	if (!ok) {
		LOG_ERR("SMP notify: CBOR encode failed");
		return;
	}

	size_t cbor_len = (size_t)(zse->payload_mut - cbor_buf);
	smp_send_notification(LORA_CMD_RX_NOTIFY, cbor_buf, cbor_len);
}

/* ── LoRa event consumer thread ──────────────────────────────────── */

/*
 * Blocks on lora_evt_q.  Each RX event received from task_lora is
 * forwarded to the host as an unsolicited SMP notification frame.
 */
static void smp_evt_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	struct lora_evt_msg evt;

	while (1) {
		k_msgq_get(&lora_evt_q, &evt, K_FOREVER);

		if (evt.type == LORA_EVT_RX) {
			LOG_INF("SMP evt: RX %d byte(s)  RSSI=%d dBm  SNR=%d dB",
				evt.len, evt.rssi, evt.snr);
			send_rx_notification(&evt);
		}
	}
}

K_THREAD_DEFINE(smp_evt_tid, 1536,
		smp_evt_thread, NULL, NULL, NULL,
		8, 0, 1000 /* ms startup delay */);

/* ── Module init ─────────────────────────────────────────────────── */

static int task_smp_uart_init(void)
{
	if (!device_is_ready(usart1_dev)) {
		LOG_ERR("USART1 not ready");
		return -ENODEV;
	}

	mgmt_register_group(&lora_mgmt_group);
	LOG_INF("SMP UART interface ready on USART1 (115200 baud, group %d)",
		LORA_SMP_GRP);
	return 0;
}

SYS_INIT(task_smp_uart_init, APPLICATION, 90);

/* ── Shell commands ──────────────────────────────────────────────── */

static int cmd_smp_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	struct lora_status st;

	lora_get_current_status(&st);

	shell_print(sh, "USART1 ready  : %s",
		    device_is_ready(usart1_dev) ? "yes" : "no");
	shell_print(sh, "LoRa RX active: %s", st.rx_active ? "yes" : "no");
	shell_print(sh, "Notify seq    : %u", (unsigned)atomic_get(&notify_seq));
	shell_print(sh, "Frequency     : %u Hz", st.frequency);
	shell_print(sh, "SF / BW / CR  : SF%u  %u kHz  CR 4/%u",
		    st.datarate,
		    bw_to_khz(st.bandwidth),
		    st.coding_rate + 4);
	shell_print(sh, "TX power      : %d dBm", st.tx_power);
	return 0;
}

/* smp rx [on|off] */
static int cmd_smp_rx(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		struct lora_status st;

		lora_get_current_status(&st);
		shell_print(sh, "LoRa RX (SMP path): %s", st.rx_active ? "on" : "off");
		return 0;
	}

	struct lora_cmd_msg cmd = { 0 };

	if (strcmp(argv[1], "on") == 0) {
		cmd.type = LORA_CMD_RX_ENABLE;
		if (k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100)) == 0) {
			shell_print(sh, "SMP LoRa RX enable queued");
		} else {
			shell_error(sh, "Command queue full");
			return -EBUSY;
		}
	} else if (strcmp(argv[1], "off") == 0) {
		cmd.type = LORA_CMD_RX_DISABLE;
		if (k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100)) == 0) {
			shell_print(sh, "SMP LoRa RX disable queued");
		} else {
			shell_error(sh, "Command queue full");
			return -EBUSY;
		}
	} else {
		shell_error(sh, "Usage: smp rx [on|off]");
		return -EINVAL;
	}
	return 0;
}

/* smp send <text> — quick TX test from the interactive shell */
static int cmd_smp_send(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: smp send <text>");
		return -EINVAL;
	}

	size_t len = strlen(argv[1]);

	if (len > LORA_MAX_PAYLOAD) {
		shell_error(sh, "Message too long (%zu > %d bytes)", len, LORA_MAX_PAYLOAD);
		return -EINVAL;
	}

	struct lora_tx_sync sync;

	k_sem_init(&sync.done, 0, 1);

	struct lora_cmd_msg cmd = {
		.type    = LORA_CMD_TX,
		.tx_len  = (uint16_t)len,
		.tx_sync = &sync,
	};
	memcpy(cmd.tx_data, argv[1], len);

	if (k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100)) != 0) {
		shell_error(sh, "Command queue full");
		return -EBUSY;
	}

	k_sem_take(&sync.done, K_MSEC(10000));

	if (sync.rc == 0) {
		shell_print(sh, "Sent %u byte(s): \"%s\"", (unsigned)len, argv[1]);
	} else {
		shell_error(sh, "lora_send failed: %d", sync.rc);
	}
	return sync.rc;
}

/* smp reset — zero the sequence counter */
static int cmd_smp_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);

	atomic_set(&notify_seq, 0);
	shell_print(sh, "SMP interface reset");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(smp_sub,
	SHELL_CMD(status, NULL,
		  "Show SMP interface state and LoRa config",
		  cmd_smp_status),
	SHELL_CMD(rx, NULL,
		  "Enable/disable LoRa RX: rx [on|off]",
		  cmd_smp_rx),
	SHELL_CMD(send, NULL,
		  "Transmit a LoRa packet via the SMP path: send <text>",
		  cmd_smp_send),
	SHELL_CMD(reset, NULL,
		  "Reset notification sequence counter",
		  cmd_smp_reset),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(smp, &smp_sub, "SMP coprocessor interface (USART1)", NULL);
