
#include "spi_driver.h"


// initialize SPI interface
bool initSpi(spi_config_t* config) {
    cyhal_spi_init(
        &config->spi_interface,
        config->MOSI,
        config->MISO,
        config->CLK,
        config->CS,
        NULL,
        8,
        CYHAL_SPI_MODE_00_MSB,
        false
    );

    cyhal_spi_set_frequency(
        &config->spi_interface, 
        config->clock_speed
    );

    return true;
}

void spiRead(spi_config_t* config, uint8_t address_to_read, uint8_t *read_buffer, uint8_t SPI_READ_MASK) {
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
    uint8_t adress[2];
    uint8_t temp[2];
    adress[0] = address_to_read & ~SPI_READ_MASK;
    cyhal_spi_transfer(&config->spi_interface, adress, sizeof(adress), temp, sizeof(temp), 0);
    *read_buffer = temp[1];
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiBurstRead(spi_config_t* config, uint8_t address_to_read, uint8_t* buffer, uint8_t length, uint8_t SPI_READ_MASK) {
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));

    length += 1; //+1 because we need 1 byte for the register address
    
    uint8_t address_helper[length];
    address_helper[0] = address_to_read & ~SPI_READ_MASK;
    for (uint8_t i = 1; i < length; i++) {
        address_helper[i] = 0;
    }
    cyhal_spi_transfer(&config->spi_interface, address_helper, length, buffer, length, 0);
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiWrite(spi_config_t* config, uint8_t address_to_write, uint8_t data, uint8_t SPI_WRITE_MASK) {
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));
    uint8_t sendBuffer[2] = {address_to_write | SPI_WRITE_MASK, data};
    uint8_t temp[2];
    cyhal_spi_transfer(&config->spi_interface, sendBuffer, sizeof(sendBuffer), temp, sizeof(temp), 0);
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiBurstWrite(spi_config_t* config, uint8_t address_to_write, uint8_t *data, uint8_t length, uint8_t SPI_WRITE_MASK) {
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));

    length += 1; //+1 because we need 1 byte for the register address
    uint8_t rx_buf[length];
    uint8_t buffer[length];
    buffer[0] = address_to_write | SPI_WRITE_MASK;
    for (uint8_t i = 1; i < length; i++) {
        buffer[i] = data[i - 1];
    }

    cyhal_spi_transfer(&config->spi_interface, buffer, length, rx_buf, length, 0);

    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiWriteNoAdressByte(spi_config_t* config, uint8_t *data, uint16_t length) {
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));

    uint8_t rx_buf[length];
    cyhal_spi_transfer(&config->spi_interface, data, length, rx_buf, length, 0);

    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void spiReadWriteNoAdressByte(spi_config_t* config, uint8_t *data_in, uint8_t *data_out, uint16_t length) {
    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH));

    cyhal_spi_transfer(&config->spi_interface, data_in, length, data_out, length, 0);

    cyhal_gpio_write(config->CS, ((config->cs_polarity == SPI_SS_ACTIVE_HIGH) ? GPIO_PIN_OUTPUT_HIGH : GPIO_PIN_OUTPUT_LOW));
}

void printArrayHexa(uint8_t arr[], uint16_t size) {
    uint16_t i;
    printf("Sending %u bytes: [", size);
    for (i = 0; i < size; i++) {
        printf("0x%x", arr[i]);
        if (i < size - 1) {
            printf(", ");
        }
    }
    printf("]\r\n");
}
