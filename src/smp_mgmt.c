/*
 * smp_mgmt.c — MCUmgr custom group 64 (LORA_SMP_GRP) handlers.
 *
 * The SMP framework opens an outer CBOR map before calling each handler and
 * closes it after.  Handlers must NOT call zcbor_map_start/end_encode — they
 * just emit key-value pairs directly into the already-open map.
 *
 * Command map:
 *   0  READ   get_version  → {version:<str>, board:<str>}
 *   1  READ   get_status   → {freq, sf, bw_khz, cr, preamble, tx_power, rx_active}
 *   2  WRITE  rx_enable    → {rc:<int>}
 *   3  WRITE  rx_disable   → {rc:<int>}
 *   4  WRITE  tx_msg  ←{msg:<str>}  → {rc:<int>, bytes_sent:<uint>}
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>

#include <zcbor_common.h>
#include <zcbor_decode.h>
#include <zcbor_encode.h>

#include "lora_ipc.h"

LOG_MODULE_REGISTER(smp_mgmt, LOG_LEVEL_DBG);

#define LORA_SMP_GRP  64  /* MGMT_GROUP_ID_PERUSER */

/* ── Helpers ─────────────────────────────────────────────────────── */

static uint16_t bw_enum_to_khz(uint8_t bw)
{
	switch (bw) {
	case BW_125_KHZ: return 125;
	case BW_250_KHZ: return 250;
	case BW_500_KHZ: return 500;
	default:         return 0;
	}
}

/* ── Command 0: get_version (READ) ───────────────────────────────── */

static int cmd_get_version(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;

	/* Framework already opened the outer map; just emit key-value pairs. */
	bool ok = zcbor_tstr_put_lit(zse, "version") &&
		  zcbor_tstr_put_lit(zse, "1.0.0") &&
		  zcbor_tstr_put_lit(zse, "board") &&
		  zcbor_tstr_put_lit(zse, "1sj_module");

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

/* ── Command 1: get_status (READ) ────────────────────────────────── */

static int cmd_get_status(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	struct lora_status st;

	lora_get_current_status(&st);

	bool ok = zcbor_tstr_put_lit(zse, "freq") &&
		  zcbor_uint32_put(zse, st.frequency) &&
		  zcbor_tstr_put_lit(zse, "sf") &&
		  zcbor_uint32_put(zse, st.datarate) &&
		  zcbor_tstr_put_lit(zse, "bw_khz") &&
		  zcbor_uint32_put(zse, bw_enum_to_khz(st.bandwidth)) &&
		  zcbor_tstr_put_lit(zse, "cr") &&
		  zcbor_uint32_put(zse, (uint32_t)(st.coding_rate + 4)) &&
		  zcbor_tstr_put_lit(zse, "preamble") &&
		  zcbor_uint32_put(zse, st.preamble_len) &&
		  zcbor_tstr_put_lit(zse, "tx_power") &&
		  zcbor_int32_put(zse, st.tx_power) &&
		  zcbor_tstr_put_lit(zse, "rx_active") &&
		  zcbor_bool_put(zse, st.rx_active);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

/* ── Command 2: rx_enable (WRITE) ────────────────────────────────── */

static int cmd_rx_enable(struct smp_streamer *ctxt)
{
	struct lora_cmd_msg cmd = { .type = LORA_CMD_RX_ENABLE };
	int ret = k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100));

	if (ret != 0) {
		LOG_WRN("rx_enable: lora_cmd_q full (%d)", ret);
	}

	zcbor_state_t *zse = ctxt->writer->zs;
	bool ok = zcbor_tstr_put_lit(zse, "rc") &&
		  zcbor_int32_put(zse, ret);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

/* ── Command 3: rx_disable (WRITE) ──────────────────────────────── */

static int cmd_rx_disable(struct smp_streamer *ctxt)
{
	struct lora_cmd_msg cmd = { .type = LORA_CMD_RX_DISABLE };
	int ret = k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(100));

	if (ret != 0) {
		LOG_WRN("rx_disable: lora_cmd_q full (%d)", ret);
	}

	zcbor_state_t *zse = ctxt->writer->zs;
	bool ok = zcbor_tstr_put_lit(zse, "rc") &&
		  zcbor_int32_put(zse, ret);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

/* ── Command 4: tx_msg (WRITE) ←{msg:<str>} → {rc, bytes_sent} ───── */

static int cmd_tx_msg(struct smp_streamer *ctxt)
{
	zcbor_state_t *zsd = ctxt->reader->zs;
	struct zcbor_string key = {0}, msg_str = {0};
	bool ok;

	/* Decode request {msg: <tstr>}. */
	ok = zcbor_map_start_decode(zsd) &&
	     zcbor_tstr_decode(zsd, &key) &&
	     (key.len == 3 && memcmp(key.value, "msg", 3) == 0) &&
	     zcbor_tstr_decode(zsd, &msg_str);

	if (!ok || msg_str.len == 0) {
		LOG_ERR("tx_msg: failed to decode {msg}");
		return MGMT_ERR_EINVAL;
	}

	zcbor_map_end_decode(zsd);

	uint16_t len = (uint16_t)MIN(msg_str.len, LORA_MAX_PAYLOAD);

	/*
	 * lora_tx_sync lives on this stack frame.  We block on sync.done,
	 * so the frame stays alive until the LoRa task signals completion.
	 */
	struct lora_tx_sync sync;

	k_sem_init(&sync.done, 0, 1);
	sync.rc         = 0;
	sync.bytes_sent = 0;

	struct lora_cmd_msg cmd = {
		.type    = LORA_CMD_TX,
		.tx_len  = len,
		.tx_sync = &sync,
	};
	memcpy(cmd.tx_data, msg_str.value, len);

	int ret = k_msgq_put(&lora_cmd_q, &cmd, K_MSEC(200));

	if (ret != 0) {
		LOG_ERR("tx_msg: lora_cmd_q full (%d)", ret);
	} else {
		/* Block until the LoRa task completes the TX. */
		k_sem_take(&sync.done, K_SECONDS(30));
		ret = sync.rc;
	}

	zcbor_state_t *zse = ctxt->writer->zs;
	bool enc_ok = zcbor_tstr_put_lit(zse, "rc") &&
		      zcbor_int32_put(zse, ret) &&
		      zcbor_tstr_put_lit(zse, "bytes_sent") &&
		      zcbor_uint32_put(zse, (uint32_t)sync.bytes_sent);

	return enc_ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

/* ── Handler table ───────────────────────────────────────────────── */

static const struct mgmt_handler lora_smp_handlers[] = {
	[0] = { .mh_read  = cmd_get_version, .mh_write = NULL          },
	[1] = { .mh_read  = cmd_get_status,  .mh_write = NULL          },
	[2] = { .mh_read  = NULL,            .mh_write = cmd_rx_enable  },
	[3] = { .mh_read  = NULL,            .mh_write = cmd_rx_disable },
	[4] = { .mh_read  = NULL,            .mh_write = cmd_tx_msg     },
};

static struct mgmt_group lora_smp_group = {
	.mg_handlers       = lora_smp_handlers,
	.mg_handlers_count = ARRAY_SIZE(lora_smp_handlers),
	.mg_group_id       = LORA_SMP_GRP,
};

int my_mgmt_init(void)
{
	mgmt_register_group(&lora_smp_group);
	LOG_INF("registered MCUmgr group id=%u (%zu commands)",
		lora_smp_group.mg_group_id,
		lora_smp_group.mg_handlers_count);
	return 0;
}
