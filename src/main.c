#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_spi.h"

/* SPI baud rate in Hz */
#define SPI_FREQ_HZ 1000
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

void handle_error(uint32_t status) {
    if (status != CY_RSLT_SUCCESS) {
        CY_ASSERT(0);
        printf("Error\r\n");
    }
}

int main(void) {
    cy_rslt_t result;
    uint32_t cmd_send = CYBSP_LED_STATE_OFF;
    cyhal_spi_t mSPI;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

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

    printf("*************** "
           "HAL: SPI Master "
           "*************** \r\n\n");

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

    printf("Send SX1276 config...");
    uint8_t msg[] = {
        RH_RF95_REG_01_OP_MODE | RH_SPI_WRITE_MASK,
        RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE
    };
    result = cyhal_spi_send(&mSPI, *msg);
    handle_error(result);
    printf("done\r\n");
    printf("sent %x\r\n", (uint16_t) *msg);

    printf("Check SX1276 config...");
    uint8_t tx_msg[2];
    tx_msg[0] = RH_RF95_REG_01_OP_MODE & ~RH_SPI_WRITE_MASK;
    uint8_t rx_msg[2];
    int tx_length = sizeof(tx_msg), rx_length = sizeof(rx_msg);
    printf("pre %x\r\n", (uint16_t) *tx_msg);
    result = cyhal_spi_transfer(
        &mSPI, 
        &tx_msg,
        tx_length, 
        &rx_msg, 
        rx_length,
        0xFF
    );
    handle_error(result);

    printf("sent %x (%d)\r\n", tx_msg[0], tx_length);
    printf("recv %x (%d)\r\n", rx_msg[1], rx_length);


    for (;;) {
        /* Toggle the slave LED state */
        //cmd_send = (cmd_send == CYBSP_LED_STATE_OFF) ?
        //             CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF;

        ///* Send the command packet to the slave */
        //result = cyhal_spi_send(&mSPI, cmd_send);

        //handle_error(result);

        printf("loop\n");
        /* Give delay between commands */
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
    }
}

