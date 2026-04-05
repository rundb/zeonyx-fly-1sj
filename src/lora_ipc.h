/*
 * lora_ipc.h — LoRa inter-task message types and queue declarations.
 *
 * task_lora.c owns the LoRa device and defines both queues.
 * task_smp_uart.c is the sole producer of lora_cmd_q and sole consumer
 * of lora_evt_q.
 *
 *   lora_cmd_q  (SMP → LoRa)  LORA_CMD_TX / RX_ENABLE / RX_DISABLE
 *   lora_evt_q  (LoRa → SMP)  LORA_EVT_RX
 */
#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>
#include <stdbool.h>
#include <stdint.h>

#define LORA_MAX_PAYLOAD 255

/* ── Commands: SMP → LoRa task ───────────────────────────────────── */

enum lora_cmd_type {
	LORA_CMD_TX,
	LORA_CMD_RX_ENABLE,
	LORA_CMD_RX_DISABLE,
};

/*
 * Synchronisation block for TX commands.  Allocated on the sender's
 * stack; the LoRa task writes rc/bytes_sent then gives the semaphore.
 * The sender must remain blocked on done until the semaphore is given.
 */
struct lora_tx_sync {
	struct k_sem done;
	int          rc;
	uint16_t     bytes_sent;
};

struct lora_cmd_msg {
	enum lora_cmd_type   type;
	uint8_t              tx_data[LORA_MAX_PAYLOAD]; /* LORA_CMD_TX only */
	uint16_t             tx_len;                    /* LORA_CMD_TX only */
	struct lora_tx_sync *tx_sync;                   /* LORA_CMD_TX only */
};

/* ── Events: LoRa task → SMP ─────────────────────────────────────── */

enum lora_evt_type {
	LORA_EVT_RX,
};

struct lora_evt_msg {
	enum lora_evt_type type;
	uint8_t  data[LORA_MAX_PAYLOAD];
	int      len;
	int16_t  rssi;
	int8_t   snr;
};

/* ── Status snapshot ─────────────────────────────────────────────── */

struct lora_status {
	uint32_t frequency;
	uint8_t  datarate;
	uint8_t  bandwidth;
	uint8_t  coding_rate;
	uint16_t preamble_len;
	int8_t   tx_power;
	bool     rx_active;
};

/* ── Queues defined in task_lora.c ───────────────────────────────── */

extern struct k_msgq lora_cmd_q;
extern struct k_msgq lora_evt_q;

/* Returns a consistent snapshot of the current modem config and RX state. */
void lora_get_current_status(struct lora_status *out);
