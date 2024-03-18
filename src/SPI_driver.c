
#include "SPI_Driver.h"

// initialize SPI interface
bool initSpi(spi_config_t config)
{
  wiced_hal_pspi_reset(config.spi_interface);
  UINT32 shifted_config = SPI_PIN_CONFIG(config.CLK, config.CS, config.MOSI, config.MISO);
  wiced_hal_pspi_init(config.spi_interface, config.device_role, config.input_pin_config, shifted_config, config.clock_speed, config.endian, config.cs_polarity, config.spi_mode, config.CS);

  UINT32 polarity = GPIO_PIN_OUTPUT_HIGH;
  if (config.cs_polarity != SPI_SS_ACTIVE_HIGH)
    polarity = GPIO_PIN_OUTPUT_LOW;

  wiced_hal_gpio_init();
  wiced_hal_gpio_configure_pin(config.CS, GPIO_INPUT_DISABLE, polarity);
  return true;
}

void spiRead(spi_config_t config, UINT8 address_to_read, UINT8 *read_buffer, UINT8 SPI_READ_MASK)
{
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
  UINT8 adress[2];
  UINT8 temp[2];
  adress[0] = address_to_read & ~SPI_READ_MASK;
  wiced_hal_pspi_exchange_data(config.spi_interface, sizeof(adress), adress, temp);
  *read_buffer = temp[1];
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

UINT8 *spiBurstRead(spi_config_t config, UINT8 address_to_read, UINT8 length, UINT8 SPI_READ_MASK)
{
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
  length += 1; //+1 because we need 1 byte for the register address
  UINT8 *buffer = allocateBufferUint8(length);
  UINT8 *address_helper = allocateBufferUint8(length);
  if (buffer == NULL || address_helper == NULL)
  {
    WICED_BT_TRACE("SPI Reading is not possible due to Allocation problems!\n\r");
    return NULL;
  }
  address_helper[0] = address_to_read & ~SPI_READ_MASK;
  for (UINT8 i = 1; i < length; i++)
    address_helper[i] = 0;
  wiced_hal_pspi_exchange_data(config.spi_interface, length, address_helper, buffer);
  wiced_bt_free_buffer(address_helper);
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
  return buffer;
}

void spiWrite(spi_config_t config, UINT8 address_to_write, UINT8 data, UINT8 SPI_WRITE_MASK)
{
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
  UINT8 sendBuffer[2] = {address_to_write | SPI_WRITE_MASK, data};
  wiced_hal_pspi_tx_data(config.spi_interface, sizeof(sendBuffer), sendBuffer);
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiBurstWrite(spi_config_t config, UINT8 address_to_write, UINT8 *data, UINT8 length, UINT8 SPI_WRITE_MASK)
{
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
  length += 1; //+1 because we need 1 byte for the register address
  UINT8 *buffer = allocateBufferUint8(length);
  if (buffer == NULL)
  {
    WICED_BT_TRACE("SPI Writing is not possible due to Allocation problems!\n\r");
    return;
  }
  buffer[0] = address_to_write | SPI_WRITE_MASK;
  for (UINT8 i = 1; i < length; i++)
    buffer[i] = data[i - 1];
  wiced_hal_pspi_tx_data(config.spi_interface, length, buffer);
  wiced_bt_free_buffer(buffer);
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiWriteNoAdressByte(spi_config_t config, UINT8 *data, UINT16 length)
{
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
  wiced_hal_pspi_tx_data(config.spi_interface, length, data);
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiReadWriteNoAdressByte(spi_config_t config, UINT8 *data_in, UINT8 *data_out, UINT16 length)
{
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
  wiced_hal_pspi_exchange_data(config.spi_interface, (UINT32)length, data_in, data_out);
  wiced_hal_gpio_set_pin_output(config.CS, ((config.cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void printArrayHexa(UINT8 arr[], UINT16 size)
{
  UINT16 i;
  WICED_BT_TRACE("Sending %u bytes: [", size);
  for (i = 0; i < size; i++)
  {
    WICED_BT_TRACE("0x%x", arr[i]);
    if (i < size - 1)
    {
      WICED_BT_TRACE(", ");
    }
  }
  WICED_BT_TRACE("]\r\n");
}
