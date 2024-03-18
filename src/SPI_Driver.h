

/*
Simple SPI library for basic read and write operations. Uses dynamic memory!
It is assumed the wiring and configuration in the device manager is correct, since there is no way
to check for a successful initilization :(
*/

#if !defined(SPI_DRIVER_H_)
#define SPI_DRIVER_H_

/*****************************    Includes   *****************************/
#include "memory_Helper.h"
#include "wiced_hal_pspi.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_memory.h"
#include "bt_types.h"

#include <stdbool.h>

/*****************************    Definitions   *****************************/

/*****************************    Data types   *****************************/

/// @brief Custom data type for SPI configuration. Keep in mind that you have to set the same pins in the device manager!
typedef struct spi_config
{
  wiced_bt_gpio_numbers_t MOSI, MISO, CLK, CS;
  spi_interface_t spi_interface; //SPI1 or SPI2
  UINT8 device_role;      // SPI_MASTER or SPI_SLAVE
  UINT16 input_pin_config; // INPUT_PIN_PULL_UP, INPUT_PIN_PULL_DOWN or INPUT_PIN_FLOATING
  UINT32 clock_speed;     // in Hz; 93750-12000000 for Master; 0 for Slave
  SPI_ENDIAN endian;  //SPI_MSB_FIRST or SPI_LSB_FIRST
  SPI_SS_POLARITY cs_polarity; //SPI_SS_ACTIVE_LOW or SPI_SS_ACTIVE_HIGH
  SPI_MODE spi_mode; // SPI mode 0-3, see wiced_hal_pspi.h for definition
} spi_config_t;

/**********************************************************************/
/*                                                                    */
/*                           FUNCTION SIGNATURES                      */
/*                                                                    */
/**********************************************************************/

/// @brief Initializes the SPI communication
/// @param config strcut with pin and interface config
/// @return true when done
bool initSpi(spi_config_t config);

/// @brief Write 1 byte of data to a specified address
/// @param config SPI config struct
/// @param address_to_write specify address to write to
/// @param data to write
/// @param SPI_WRITE_MASK (optional) mask for the register adress to allow for writing
void spiWrite(spi_config_t config, UINT8 address_to_write, UINT8 data, UINT8 SPI_WRITE_MASK);

/// @brief Send data of dynamic length
/// @param config SPI config struct
/// @param address_to_write specify address to write to
/// @param data to write
/// @param length number of bytes to write
/// @param SPI_WRITE_MASK (optional) mask for the register adress to allow for writing
void spiBurstWrite(spi_config_t config, UINT8 address_to_write, UINT8 *data, UINT8 length, UINT8 SPI_WRITE_MASK);

/// @brief Read 1 byte of data from a specific address
/// @param config SPI config struct
/// @param address_to_read specify address to read from
/// @param read_buffer pointer to read buffer
/// @param SPI_READ_MASK (optional) mask for the register adress to allow for reading
void spiRead(spi_config_t config, UINT8 address_to_read, UINT8 *read_buffer, UINT8 SPI_READ_MASK);

/// @brief Read data of dynamic length
/// @param config SPI config struct
/// @param address_to_read specify address to read from
/// @param length number of bytes to read
/// @param SPI_READ_MASK (optional) mask for the register adress to allow for reading
/// @return pointer to read data, NULL if error occured
UINT8 *spiBurstRead(spi_config_t config, UINT8 address_to_read, UINT8 length, UINT8 SPI_READ_MASK);

/// @brief Write data of static size with no adress byte infront
/// @param config SPI config struct
/// @param data to write
/// @param length size in bytes 
void spiWriteNoAdressByte(spi_config_t config, UINT8 *data, UINT16 length);

/// @brief Send data of dynamic length with no address byte infront
/// @param config SPI config struct
/// @param data to write
/// @param length number of bytes to write
void spiBurstWriteNoAdressByte(spi_config_t config, UINT8 *data, UINT8 length);

/// @brief Read and write data of dynamic length
/// @param config SPI config struct
/// @param data_in data to write
/// @param data_out where to store read data
/// @param length number of bytes to read/write
void spiReadWriteNoAdressByte(spi_config_t config, UINT8 *data_in, UINT8 *data_out, UINT16 length);


void printArrayHexa(UINT8 arr[], UINT16 size);

#endif // SPI_DRIVER_H_
