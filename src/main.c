#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_lptimer.h"
#include "cyhal_spi.h"
#include "cyhal_system.h"
#include "sht3x.h"
#include "rfm95.h"
#include "cy_sysclk.h"
#include <stdlib.h>

/* SPI baud rate in Hz */
#define SPI_FREQ_HZ                 (1000000)

#define CMD_TO_CMD_DELAY            (1000UL)
/* SPI transfer bits per frame */
#define BITS_PER_FRAME              (8)

#define SPI_READ_MASK			    (0x80)

#define GPIO_INTERRUPT_PRIORITY     (7u)

cyhal_lptimer_t lptimer;
cyhal_lptimer_info_t lptimer_info;
rfm95_handle_t rfm_handle;
cyhal_trng_t trng_obj;

void stop_on_error(int status) {
    if (status != CY_RSLT_SUCCESS) {
        printf("Error (%d)\r\n", status);
        printf("Stopping program.\r\n");
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

    // Wait for counter to reach 0
    while (!Cy_SysClk_ClkMeasurementCountersDone()){};
}

unsigned char random_int(unsigned char in) {
    return rand() % 16;
}

void handle_interrupt_DIO0(void *handler_arg, cyhal_gpio_event_t event) {
    rfm95_on_interrupt(&rfm_handle, RFM95_INTERRUPT_DIO0);
}

void handle_interrupt_DIO1(void *handler_arg, cyhal_gpio_event_t event) {
    rfm95_on_interrupt(&rfm_handle, RFM95_INTERRUPT_DIO1);
}

void handle_interrupt_DIO5(void *handler_arg, cyhal_gpio_event_t event) {
    rfm95_on_interrupt(&rfm_handle, RFM95_INTERRUPT_DIO5);
}

int main(void) {
    cyhal_spi_t mSPI;

    // Initialize the device and board peripherals
    stop_on_error(cybsp_init());

    // Initialize retarget-io for uart logs
    stop_on_error(cy_retarget_io_init(
        CYBSP_DEBUG_UART_TX, 
        CYBSP_DEBUG_UART_RX,
        CY_RETARGET_IO_BAUDRATE
    ));

    // ANSI ESC sequence for clear screen
    printf("\x1b[2J\x1b[;H");

    printf("Configuring SPI master...");
    stop_on_error(cyhal_spi_init(
        &mSPI,
        CYBSP_SPI_MOSI,
        CYBSP_SPI_MISO,
        CYBSP_SPI_CLK,
        CYBSP_SPI_CS,
        NULL,
        8,
        CYHAL_SPI_MODE_00_MSB,
        false
    ));
    printf("done\r\n");

    printf("Initializing RFM95...");

    stop_on_error(cyhal_spi_set_frequency(
        &mSPI, 
        SPI_FREQ_HZ
    ));

    // The timer is used to wait for interrupts
    stop_on_error(cyhal_lptimer_init(&lptimer));
    cyhal_lptimer_get_info(&lptimer, &lptimer_info);
    
    __enable_irq();

    /* LoraWAN module */

    rfm_handle = (rfm95_handle_t) {
        .spi_handle = &mSPI,
        .get_precision_tick = get_tick,
        .random_int = random_int,
        .precision_sleep_until = sleep_until_tick,
        .precision_tick_frequency = lptimer_info.frequency_hz,
        .nss_pin = CYBSP_SPI_CS,
        .nss_port = 0,
        .nrst_pin = 0,
        .nrst_port = 0,
        .device_address = {
            0x26, 0x0B, 0x42, 0x1C
        },
		.application_session_key = {
            0xB2, 0x84, 0x4D, 0x48, 0xC0, 0x49, 0x07, 0xF8, 0x48, 0x9F, 0xE3, 0xFD, 0xCD, 0xB6, 0x96, 0xE3
        },
		.network_session_key = {
            0x66, 0xE8, 0xFA, 0xDF, 0xB5, 0xB6, 0xA0, 0x5A, 0xE3, 0x1F, 0x83, 0xAC, 0x4C, 0x86, 0xD6, 0xD1
        },
        .get_battery_level = NULL,
        .precision_tick_drift_ns_per_s = 0,
        .receive_mode = RFM95_RECEIVE_MODE_NONE,
        .config = { 0 }, // can be NULL
        .on_after_interrupts_configured = NULL, // can be NULL
    };

    if (!rfm95_init(&rfm_handle)) {
        printf("error\r\n");
    } else {
        printf("done\r\n");
    }

    // The interrupts are used to by the LoraWAN module to inform about specific events
    
    cyhal_gpio_callback_data_t callback_data_dio0;
    stop_on_error(cyhal_gpio_init(CYBSP_D2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0));
    callback_data_dio0.callback = handle_interrupt_DIO0;
    cyhal_gpio_register_callback(CYBSP_D2, &callback_data_dio0);
    cyhal_gpio_enable_event(CYBSP_D2, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);

    cyhal_gpio_callback_data_t callback_data_dio1;
    stop_on_error(cyhal_gpio_init(CYBSP_D7, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0));
    callback_data_dio1.callback = handle_interrupt_DIO1;
    cyhal_gpio_register_callback(CYBSP_D7, &callback_data_dio1);
    cyhal_gpio_enable_event(CYBSP_D7, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);

    stop_on_error(cyhal_gpio_init(CYBSP_D8, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0));
    cyhal_gpio_callback_data_t callback_data_dio5;
    callback_data_dio5.callback = handle_interrupt_DIO5;
    cyhal_gpio_register_callback(CYBSP_D8, &callback_data_dio5);
    cyhal_gpio_enable_event(CYBSP_D8, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);

     /* Temperature sensor */

    printf("Initializing SHT31...");
    if (sensirion_i2c_init()) {
        printf("done\r\n");
    } else {
        printf("error\r\n");
    }

    for (;;) {
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);

        int32_t temperature, humidity;
        if (STATUS_OK != sht3x_measure_blocking_read(
            SHT3X_I2C_ADDR_DFLT,
            &temperature, 
            &humidity
        )) {
            printf("error reading measurements\r\n");
            continue;
        }

        printf(
            "temperature: %0.2f degreeCelsius, "
            "humidity: %0.2f percentRH\r\n",
            temperature / 1000.0f, 
            humidity / 1000.0f
        );

        // Big endian
        //uint8_t data_packet[4] = { 0 };
        //data_packet[0] = temperature >> 24;
        //data_packet[1] = temperature >> 16;
        //data_packet[2] = temperature >> 8;
        //data_packet[3] = temperature;

        uint8_t data_packet[] = {
            0x01, 0x02, 0x03, 0x04
        };

        if (!rfm95_send_receive_cycle(&rfm_handle, data_packet, sizeof(data_packet))) {
            printf("RFM95 send failed\r\n");
        } else {
            printf("RFM95 send success\r\n");
        }
    }
}

