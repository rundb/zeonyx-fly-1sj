#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>

#include <zcbor_common.h>
#include <zcbor_decode.h>
#include <zcbor_encode.h>

LOG_MODULE_REGISTER(my_smp_mgmt, LOG_LEVEL_DBG);

/* Group 64 = MGMT_GROUP_ID_PERUSER */
#define MY_MGMT_GROUP_ID   MGMT_GROUP_ID_PERUSER
#define MY_MGMT_CMD_PING   0

/* READ handler for group 64, command 0 */
static int my_ping_read(struct smp_streamer *ctxt)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	bool ok;

	LOG_INF("my_ping_read() hit");

	/* Optional: decode request body here from ctxt->reader->zs */

	/* Encode CBOR response map */
	ok = zcbor_map_start_encode(zse, 2) &&
	     zcbor_tstr_put_lit(zse, "rc") &&
	     zcbor_int32_put(zse, 0) &&
	     zcbor_tstr_put_lit(zse, "msg") &&
	     zcbor_tstr_put_lit(zse, "hello from custom mgmt group") &&
	     zcbor_map_end_encode(zse, 2);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static const struct mgmt_handler my_mgmt_handlers[] = {
	[MY_MGMT_CMD_PING] = {
		.mh_read = my_ping_read,
		.mh_write = NULL,
	},
};

static struct mgmt_group my_mgmt_group = {
	.mg_handlers = my_mgmt_handlers,
	.mg_handlers_count = ARRAY_SIZE(my_mgmt_handlers),
	.mg_group_id = MY_MGMT_GROUP_ID,
};

int my_mgmt_init(void)
{
	mgmt_register_group(&my_mgmt_group);
	LOG_INF("registered MCUmgr group id=%u", my_mgmt_group.mg_group_id);
	return 0;
}

// SYS_INIT(my_mgmt_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);