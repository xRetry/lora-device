#if !defined(MEMORY_HELPER_H_)
#define MEMORY_HELPER_H_

#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_pspi.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_timer.h"
#include "wiced_hal_rand.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_memory.h"
#include "bt_types.h"

#include <stdbool.h>
#include <stdint.h>

/// @brief Initializes the public buffer pool, so we can allocate memory with allocateBuffer()
void initializeBtStack();

/// @brief Allocates memory for a dynamicaly sized buffer
/// @param length size in bytes
/// @return the pointer to the allocated memory; NULL if not succesful
UINT8 *allocateBufferUint8(UINT16 length);


#endif // MEMORY_HELPER_H_
