#include "cycfg_pins.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_clock.h"
#include "cyhal_clock_impl.h"
#include "cyhal_gpio.h"
#include "cyhal_hw_types.h"
#include "cyhal_lptimer.h"
#include "cyhal_spi.h"
#include <stdint.h>
#include "cyhal_system.h"
#include "sht3x.h"
#include "rfm95.h"
#include "cy_sysclk.h"

/* SPI baud rate in Hz */
#define SPI_FREQ_HZ 1000000
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY 1000UL
/* SPI transfer bits per frame */
#define BITS_PER_FRAME  16

/*****************************    Write Mask   *****************************/
#define RH_SPI_WRITE_MASK 0x80

/*****************************    Registers for LORA Board   *****************************/
#define RH_RF95_REG_00_FIFO 0x00
#define RH_RF95_REG_01_OP_MODE 0x01
#define RH_RF95_REG_02_RESERVED 0x02
#define RH_RF95_REG_03_RESERVED 0x03
#define RH_RF95_REG_04_RESERVED 0x04
#define RH_RF95_REG_05_RESERVED 0x05
#define RH_RF95_REG_06_FRF_MSB 0x06
#define RH_RF95_REG_07_FRF_MID 0x07
#define RH_RF95_REG_08_FRF_LSB 0x08
#define RH_RF95_REG_09_PA_CONFIG 0x09
#define RH_RF95_REG_0A_PA_RAMP 0x0a
#define RH_RF95_REG_0B_OCP 0x0b
#define RH_RF95_REG_0C_LNA 0x0c
#define RH_RF95_REG_0D_FIFO_ADDR_PTR 0x0d
#define RH_RF95_REG_0E_FIFO_TX_BASE_ADDR 0x0e
#define RH_RF95_REG_0F_FIFO_RX_BASE_ADDR 0x0f
#define RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR 0x10
#define RH_RF95_REG_11_IRQ_FLAGS_MASK 0x11
#define RH_RF95_REG_12_IRQ_FLAGS 0x12
#define RH_RF95_REG_13_RX_NB_BYTES 0x13
#define RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB 0x14
#define RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB 0x15
#define RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB 0x16
#define RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB 0x17
#define RH_RF95_REG_18_MODEM_STAT 0x18
#define RH_RF95_REG_19_PKT_SNR_VALUE 0x19
#define RH_RF95_REG_1A_PKT_RSSI_VALUE 0x1a
#define RH_RF95_REG_1B_RSSI_VALUE 0x1b
#define RH_RF95_REG_1C_HOP_CHANNEL 0x1c
#define RH_RF95_REG_1D_MODEM_CONFIG1 0x1d
#define RH_RF95_REG_1E_MODEM_CONFIG2 0x1e
#define RH_RF95_REG_1F_SYMB_TIMEOUT_LSB 0x1f
#define RH_RF95_REG_20_PREAMBLE_MSB 0x20
#define RH_RF95_REG_21_PREAMBLE_LSB 0x21
#define RH_RF95_REG_22_PAYLOAD_LENGTH 0x22
#define RH_RF95_REG_23_MAX_PAYLOAD_LENGTH 0x23
#define RH_RF95_REG_24_HOP_PERIOD 0x24
#define RH_RF95_REG_25_FIFO_RX_BYTE_ADDR 0x25
#define RH_RF95_REG_26_MODEM_CONFIG3 0x26

#define RH_RF95_REG_40_DIO_MAPPING1 0x40
#define RH_RF95_REG_41_DIO_MAPPING2 0x41
#define RH_RF95_REG_42_VERSION 0x42

#define RH_RF95_REG_4B_TCXO 0x4b
#define RH_RF95_REG_4D_PA_DAC 0x4d
#define RH_RF95_REG_5B_FORMER_TEMP 0x5b
#define RH_RF95_REG_61_AGC_REF 0x61
#define RH_RF95_REG_62_AGC_THRESH1 0x62
#define RH_RF95_REG_63_AGC_THRESH2 0x63
#define RH_RF95_REG_64_AGC_THRESH3 0x64
/*****************************    Operating modes   *****************************/
#define RH_RF95_LONG_RANGE_MODE 0x80
#define RH_RF95_ACCESS_SHARED_REG 0x40
#define RH_RF95_MODE 0x07
#define RH_RF95_MODE_SLEEP 0x00
#define RH_RF95_MODE_STDBY 0x01
#define RH_RF95_MODE_FSTX 0x02
#define RH_RF95_MODE_TX 0x03
#define RH_RF95_MODE_FSRX 0x04
#define RH_RF95_MODE_RXCONTINUOUS 0x05
#define RH_RF95_MODE_RXSINGLE 0x06
#define RH_RF95_MODE_CAD 0x07

// RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_RX_TIMEOUT 0x80
#define RH_RF95_RX_DONE 0x40
#define RH_RF95_PAYLOAD_CRC_ERROR 0x20
#define RH_RF95_VALID_HEADER 0x10
#define RH_RF95_TX_DONE 0x08
#define RH_RF95_CAD_DONE 0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL 0x02
#define RH_RF95_CAD_DETECTED 0x01

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_SELECT 0x80
#define RH_RF95_MAX_POWER 0x70
#define RH_RF95_OUTPUT_POWER 0x0f
#define RH_RF95_MAX_PAYLOAD_LEN 0xff
#define RH_RF95_HEADER_LEN 4
#define RH_RF95_MAX_MESSAGE_LEN (RH_RF95_MAX_PAYLOAD_LEN - RH_RF95_HEADER_LEN)

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE 0x04
#define RH_RF95_PA_DAC_ENABLE 0x07

// RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_PLL_TIMEOUT 0x80
#define RH_RF95_RX_PAYLOAD_CRC_IS_ON 0x40
#define RH_RF95_FHSS_PRESENT_CHANNEL 0x3f

// This is the address that indicates a broadcast
#define RH_BROADCAST_ADDRESS 0xff

// The crystal oscillator frequency of the module
#define RH_RF95_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
#define RH_RF95_FSTEP (RH_RF95_FXOSC / 524288)

#define RH_SPI_WRITE_MASK 0x80

cyhal_lptimer_t lptimer;
cyhal_lptimer_info_t lptimer_info;

void handle_error(uint32_t status) {
    if (status != CY_RSLT_SUCCESS) {
        printf("Error\r\n");
        CY_ASSERT(0);
    }
}

uint32_t get_tick() {
    return cyhal_lptimer_read(&lptimer);
}

void sleep_until_tick(unsigned int tick_count) {
    unsigned int init_val = tick_count - get_tick() - 5;
    
    (void)Cy_SysClk_StartClkMeasurementCounters(
        (cy_en_meas_clks_t) CY_SYSCLK_MEAS_CLK_LFCLK,
        init_val, 
        CY_SYSCLK_MEAS_CLK_IMO
    );

    /* Wait for counter to reach 0 */
    while (!Cy_SysClk_ClkMeasurementCountersDone()){};
}

unsigned char random_int(unsigned char in) {
    return 136;
}

int main(void) {
    //cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    cy_rslt_t result;
    uint32_t cmd_send = CYBSP_LED_STATE_OFF;
    cyhal_spi_t mSPI;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    //result = cyhal_gpio_init(CYBSP_SPI_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    //handle_error(result);

    /* Initialize retarget-io for uart logs */
    result = cy_retarget_io_init(
        CYBSP_DEBUG_UART_TX, 
        CYBSP_DEBUG_UART_RX,
        CY_RETARGET_IO_BAUDRATE
    );
    /* Retarget-io init failed. Stop program execution */
    handle_error(result);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("Configuring SPI master...");
    result = cyhal_spi_init(
        &mSPI,
        CYBSP_SPI_MOSI,
        CYBSP_SPI_MISO,
        CYBSP_SPI_CLK,
        CYBSP_SPI_CS,
        NULL,
        BITS_PER_FRAME,
        CYHAL_SPI_MODE_00_MSB,
        false
    );
    handle_error(result);
    printf("done\r\n");

    printf("Set SPI frequency...");
    result = cyhal_spi_set_frequency(
        &mSPI, 
        SPI_FREQ_HZ
    );
    handle_error(result);
    printf("done\r\n");

    /* Enable interrupts */
    __enable_irq();

    result = cyhal_lptimer_init(&lptimer);
    handle_error(result);

    cyhal_lptimer_get_info(&lptimer, &lptimer_info);

    printf("Initializing RFM95...\r\n");
    
    rfm95_handle_t rfm_handle = {
        .spi_handle = &mSPI,
        .get_precision_tick = get_tick,
        .random_int = random_int, // TODO(marco): Maybe not needed?
        .precision_sleep_until = sleep_until_tick,
        .precision_tick_frequency = lptimer_info.frequency_hz,
        .nss_pin = CYBSP_SPI_CS,
        .nss_port = NULL,
        .nrst_pin = NULL,
        .nss_port = NULL,
        .device_address = {
            0x00, 0x00, 0x00, 0x00
        },
		.application_session_key = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        },
		.network_session_key = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        },
        .get_battery_level = NULL,
        .precision_tick_drift_ns_per_s = 0,
        .receive_mode = RFM95_RECEIVE_MODE_NONE,
        .config = NULL, // can be NULL
        .on_after_interrupts_configured = NULL, // can be NULL
    };

     // Initialise RFM95 module.
    if (!rfm95_init(&rfm_handle)) {
        printf("error\r\n");
    } else {
        printf("done\r\n");
    }

    printf("Initializing SHT31...");
    if (sensirion_i2c_init()) {
        printf("done\r\n");
    } else {
        printf("error\r\n");
    }

    printf("Probing SHT31...\r\n");
    while (sht3x_probe(SHT3X_I2C_ADDR_DFLT) != STATUS_OK) {
        printf("SHT31 sensor probing failed\r\n");
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
    }
    printf("SHT31 sensor probing successful\r\n");


    for (;;) {
        int32_t temperature, humidity;
        int8_t ret = sht3x_measure_blocking_read(
            SHT3X_I2C_ADDR_DFLT,
            &temperature, 
            &humidity
        );
        if (ret == STATUS_OK) {
            printf("measured temperature: %0.2f degreeCelsius, "
                   "measured humidity: %0.2f percentRH\r\n",
                   temperature / 1000.0f, humidity / 1000.0f);
        } else {
            printf("error reading measurement\r\n");
        }

        uint8_t data_packet[] = {
            0x01, 0x02, 0x03, 0x4
        };

        if (!rfm95_send_receive_cycle(&rfm_handle, data_packet, sizeof(data_packet))) {
            printf("RFM95 send failed\r\n");
        } else {
            printf("RFM95 send success\r\n");
        }
    
        ///* Send the command packet to the slave */
        //result = cyhal_spi_send(&mSPI, cmd_send);

        //handle_error(result);

        //int tick = get_tick();
        //sleep_until_tick(tick+1000);
        //printf("target: %d, actual %d\r\n", tick+1000, get_tick());
        //printf("diff: %d\r\n", tick+1000-get_tick());

        /* Give delay between commands */
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
    }
}

