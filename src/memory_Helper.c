#include "memory_Helper.h"

void initializeBtStack()
{
	wiced_bt_cfg_settings_t bt_cfg_settings = {
			.max_number_of_buffer_pools = 100};

	wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
			{
					/*  { buf_size, buf_count } */
					{64, 16},	 /* Small Buffer Pool */
					{360, 8},	 /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
					{1056, 4}, /* Large Buffer Pool  (used for HCI ACL messages) */
					{2048, 4}, /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
			};

	wiced_bt_stack_init(NULL, &bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

UINT8 *allocateBufferUint8(UINT16 length)
{
	UINT8 *buffer = (UINT8 *)wiced_bt_get_buffer((length) * sizeof(UINT8));
	if (buffer == NULL)
	{
		WICED_BT_TRACE("MEMORY ALLOCATION FAILED! PROBABLY NOT ENOUGH RAM AVAILABLE!\n\r");
		return NULL;
	}
	return buffer;
}
