/*
 * task_smp_uart.c — MCUMgr/SMP command observer and LoRa IPC thread.
 *
 * USART1 (PA9=TX, PA10=RX) is driven by the Zephyr uart_mcumgr console
 * driver when CONFIG_MCUMGR_TRANSPORT_UART=y.  That driver installs its own
 * UART IRQ callback, so this task does NOT install a competing one.
 *
 * Instead we use the MCUMgr notification-hook API:
 *   MGMT_EVT_OP_CMD_RECV fires after the transport has decoded and framed
 *   each incoming SMP packet, giving us the command group, id, and opcode.
 *
 * NOTE on payload size: the standard mgmt_evt_op_cmd_arg struct does NOT
 * carry nh_len (SMP header payload length).  To log that field one would need
 * a custom uart_mcumgr receive hook that parses the raw decoded net_buf before
 * dispatch (see uart_mcumgr_register() in uart_mcumgr.h).  For now we log the
 * fields that the callback API exposes.
 *
 * Required prj.conf additions (already set):
 *   CONFIG_MCUMGR_MGMT_NOTIFICATION_HOOKS=y
 *   CONFIG_MCUMGR_SMP_COMMAND_STATUS_HOOKS=y
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include "lora_ipc.h"

LOG_MODULE_REGISTER(smp_handler, LOG_LEVEL_DBG);

/* USART1 — the SMP coprocessor interface (zephyr,uart-mcumgr in DTS) */
#define SMP_UART_NODE  DT_CHOSEN(zephyr_uart_mcumgr)
static const struct device *smp_uart = DEVICE_DT_GET(SMP_UART_NODE);

/* ── MCUMgr command-receive notification callback ────────────────── */
/*
 * Called from the SMP server work queue after each fully-decoded SMP frame.
 * group  — MGMT_GROUP_ID_OS(0), _IMAGE(1), _FS(8), _SHELL(9), …
 * id     — command index within the group
 * op     — MGMT_OP_READ(0)/MGMT_OP_WRITE(2) (from mgmt_defines.h)
 */
static enum mgmt_cb_return on_smp_cmd_recv(uint32_t event,
					   enum mgmt_cb_return prev_status,
					   int32_t *rc, uint16_t *group,
					   bool *abort_more, void *data,
					   size_t data_size)
{
	ARG_UNUSED(event);
	ARG_UNUSED(prev_status);
	ARG_UNUSED(rc);
	ARG_UNUSED(group);
	ARG_UNUSED(abort_more);

	if (data_size < sizeof(struct mgmt_evt_op_cmd_arg)) {
		return MGMT_CB_OK;
	}

	const struct mgmt_evt_op_cmd_arg *arg = data;

	LOG_DBG("SMP recv: group=0x%04x id=%u op=%u",
		arg->group, arg->id, arg->op);

	return MGMT_CB_OK;
}

static struct mgmt_callback smp_cmd_recv_cb = {
	.callback = on_smp_cmd_recv,
	.event_id = MGMT_EVT_OP_CMD_RECV,
};

/* ── Task thread ──────────────────────────────────────────────────── */

static void task_smp_uart_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(smp_uart)) {
		LOG_ERR("SMP UART %s not ready", smp_uart->name);
		return;
	}
	LOG_INF("SMP UART ready: %s", smp_uart->name);

	/* Register management callback now that the device is confirmed ready */
	mgmt_callback_register(&smp_cmd_recv_cb);
	LOG_INF("SMP CMD_RECV hook registered");

	/*
	 * Diagnostic: poll raw bytes from USART1.
	 *
	 * If CONFIG_MCUMGR_TRANSPORT_UART=y initialised correctly,
	 * uart_irq_rx_enable() has been called and the uart_mcumgr ISR
	 * consumes all arriving bytes — uart_poll_in returns -1 immediately
	 * and the loop body never executes.
	 *
	 * If the MCUMgr init failed (smp_transport_init error, net_buf
	 * exhaustion, etc.) the IRQ is NOT armed and uart_poll_in WILL
	 * return bytes here.  Seeing "raw: 0x..." in the log means the
	 * init chain broke and the UART never entered interrupt-driven mode.
	 *
	 * Correct sender framing (SMP-over-UART serial protocol):
	 *   [0x06 0x09] [len_BE16] [base64(smp_hdr+cbor)] [crc16_BE16] [0x0A]
	 * Raw binary SMP frames are silently discarded by the serial parser.
	 */
	unsigned char byte;
	uint32_t raw_count = 0;

	while (uart_poll_in(smp_uart, &byte) == 0) {
		LOG_WRN("raw UART byte (IRQ not armed?): 0x%02x", byte);
		raw_count++;
	}
	if (raw_count == 0) {
		LOG_DBG("uart_poll_in: no bytes — IRQ mode is active (expected)");
	}

	/*
	 * Main loop: drain LoRa events produced by task_lora, and poll
	 * the raw UART as a diagnostic.  If the IRQ handler is active the
	 * uart_poll_in call is a no-op; if it returns bytes the MCUMgr
	 * UART transport never armed its interrupt.
	 */
	while (1) {
		/* ── Raw-byte diagnostic ────────────────────────────────── */
		while (uart_poll_in(smp_uart, &byte) == 0) {
			LOG_WRN("raw rx 0x%02x — MCUMgr IRQ not armed!", byte);
		}

		/* ── LoRa event forwarding ──────────────────────────────── */
		struct lora_evt_msg evt;
		int ret = k_msgq_get(&lora_evt_q, &evt, K_MSEC(100));

		if (ret == 0) {
			LOG_DBG("LoRa evt: type=%d len=%d rssi=%d snr=%d",
				evt.type, evt.len, evt.rssi, evt.snr);
			/* TODO: encode evt as SMP notification and send via
			 * uart_mcumgr_send() */
		}
	}
}

K_THREAD_DEFINE(task_smp_uart_tid, 1024,
		task_smp_uart_entry, NULL, NULL, NULL,
		8, 0, 600 /* ms — start after MCUMgr transport is up */);
