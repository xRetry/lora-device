#include "cycfg_pins.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_gpio.h"
#include "cyhal_hw_types.h"
#include "cyhal_lptimer.h"
#include "cyhal_spi.h"
#include <stdint.h>
#include "cyhal_system.h"
#include "sht3x.h"
#include "rfm95.h"
#include "cy_sysclk.h"

///* SPI baud rate in Hz */
#define SPI_FREQ_HZ 1000000
///* Delay of 1000ms between commands */
//#define CMD_TO_CMD_DELAY 1000UL
///* SPI transfer bits per frame */
//#define BITS_PER_FRAME  16
///* SPI baud rate in Hz */
//#define SPI_FREQ_HZ                (1000000UL)
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY           (1000UL)
/* SPI transfer bits per frame */
#define BITS_PER_FRAME             (8)

#define SPI_READ_MASK			   (0x80)

#define GPIO_INTERRUPT_PRIORITY (7u)

cyhal_lptimer_t lptimer;
cyhal_lptimer_info_t lptimer_info;
cyhal_gpio_callback_data_t gpio_btn_callback_data;
rfm95_handle_t rfm_handle;


void stop_on_error(uint32_t status) {
    if (status != CY_RSLT_SUCCESS) {
        printf("Error (%d)\r\n", status);
        CY_ASSERT(0);
    }
}

void handle_error(uint32_t status) {
    stop_on_error(status);
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

void spi_test(void) {
    cy_rslt_t result;
    cyhal_spi_t mSPI;

        uint8_t i;
    uint8_t vSPI_COEFF_tx[1] = {0x10};
    uint8_t vSPI_COEFF_rx[19];
    uint8_t vSPI_CFG_tx[2] = {0x08,0x07};
    uint8_t vSPI_PT_tx[1] = {0};
    uint8_t vSPI_PT_rx[7];
    uint8_t vSPI_CFG_rx[1] = {0};
    uint8_t vSPI_Write_Null = 0;
    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize retarget-io for uart logs */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                  CY_RETARGET_IO_BAUDRATE);

    /* Retarget-io init failed. Stop program execution */
    handle_error(result);


    printf("*************** "
           "HAL: SPI Master "
           "*************** \r\n\n");

    printf("Configuring SPI master...\r\n");
    /* Init SPI master */
    result = cyhal_spi_init(&mSPI,CYBSP_SPI_MOSI,CYBSP_SPI_MISO,CYBSP_SPI_CLK,
                            CYBSP_SPI_CS,NULL,BITS_PER_FRAME,
                            CYHAL_SPI_MODE_11_MSB,false);

    vSPI_CFG_tx[0] 	= (vSPI_CFG_tx[0] & ~SPI_READ_MASK);
    vSPI_PT_tx[0] 	= (vSPI_PT_tx[0] | SPI_READ_MASK);
    vSPI_COEFF_tx[0] 	= (vSPI_COEFF_tx[0] | SPI_READ_MASK);

    printf("Array Configuration Done...\r\n");

    printf("%d \r\n",vSPI_CFG_tx[0]);
    printf("%d \r\n",vSPI_CFG_rx[0]);

    /* Set the SPI baud rate */
	result = cyhal_spi_set_frequency(&mSPI, SPI_FREQ_HZ);
    printf("Clock Frequency set to: %lu Hz\r\n", SPI_FREQ_HZ);
    cyhal_system_delay_ms(CMD_TO_CMD_DELAY);


    handle_error(result);

    /* Enable interrupts */
    __enable_irq();


    result =  cyhal_spi_transfer(&mSPI, vSPI_CFG_tx, sizeof(vSPI_CFG_tx), vSPI_CFG_rx, sizeof(vSPI_CFG_rx), vSPI_Write_Null);
	printf("Sensor Configuration Done...\r\n");

	result = cyhal_spi_transfer(&mSPI, vSPI_COEFF_tx, sizeof(vSPI_COEFF_tx), vSPI_COEFF_rx, sizeof(vSPI_COEFF_rx), vSPI_Write_Null);
	printf("*** READING COEFFICIENTS ***\r\n");
	for (i=1; i< sizeof(vSPI_COEFF_rx); i++)
			{
				printf("Received Byte #%d: %d\r\n", i, vSPI_COEFF_rx[i]);
			}
	        handle_error(result);
    for (;;)
    {
    	printf("*** Starting new Iteration ***\r\n");
		result =  cyhal_spi_transfer(&mSPI, vSPI_PT_tx, sizeof(vSPI_PT_tx), vSPI_PT_rx, sizeof(vSPI_PT_rx), vSPI_Write_Null);

		for (i=1; i< sizeof(vSPI_PT_rx); i++)
		{
			printf("Received Byte #%d: %d\r\n", i, vSPI_PT_rx[i]);
		}
        handle_error(result);

        /* Give delay between commands */
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);

    }
}

void handle_interrupt_DIO0(void *handler_arg, cyhal_gpio_event_t event) {
    printf("Interrupt DIO0\r\n");
    rfm95_on_interrupt(&rfm_handle, RFM95_INTERRUPT_DIO0);
}

void handle_interrupt_DIO1(void *handler_arg, cyhal_gpio_event_t event) {
    printf("Interrupt DIO1\r\n");
    rfm95_on_interrupt(&rfm_handle, RFM95_INTERRUPT_DIO1);
}

void handle_interrupt_DIO5(void *handler_arg, cyhal_gpio_event_t event) {
    printf("Interrupt DIO5\r\n");
    rfm95_on_interrupt(&rfm_handle, RFM95_INTERRUPT_DIO5);
}

int main(void) {
    //spi_test();
    //return 0;
    cy_rslt_t result;
    cyhal_spi_t mSPI;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    stop_on_error(result);

    /* Initialize retarget-io for uart logs */
    result = cy_retarget_io_init(
        CYBSP_DEBUG_UART_TX, 
        CYBSP_DEBUG_UART_RX,
        CY_RETARGET_IO_BAUDRATE
    );
    // Retarget-io init failed. Stop program execution
    stop_on_error(result);

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
        8,
        CYHAL_SPI_MODE_00_MSB,
        false
    );
    stop_on_error(result);
    printf("done\r\n");

    printf("Set SPI frequency...");
    result = cyhal_spi_set_frequency(
        &mSPI, 
        SPI_FREQ_HZ
    );
    stop_on_error(result);
    printf("done\r\n");

    result = cyhal_lptimer_init(&lptimer);
    stop_on_error(result);

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
        .nrst_port = NULL,
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
        .config = NULL, // can be NULL
        .on_after_interrupts_configured = NULL, // can be NULL
    };

     /* Configure GPIO interrupt */
    handle_error(cyhal_gpio_init(CYBSP_D2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0));
    gpio_btn_callback_data.callback = handle_interrupt_DIO0;
    cyhal_gpio_register_callback(CYBSP_D2, 
                                 &gpio_btn_callback_data);
    cyhal_gpio_enable_event(CYBSP_D2, CYHAL_GPIO_IRQ_RISE, 
                                 GPIO_INTERRUPT_PRIORITY, true);

     /* Configure GPIO interrupt */
    handle_error(cyhal_gpio_init(CYBSP_D7, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0));
    gpio_btn_callback_data.callback = handle_interrupt_DIO1;
    cyhal_gpio_register_callback(CYBSP_D7, 
                                 &gpio_btn_callback_data);
    cyhal_gpio_enable_event(CYBSP_D7, CYHAL_GPIO_IRQ_RISE, 
                                 GPIO_INTERRUPT_PRIORITY, true);

     /* Configure GPIO interrupt */
    handle_error(cyhal_gpio_init(CYBSP_D8, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0));
    gpio_btn_callback_data.callback = handle_interrupt_DIO5;
    cyhal_gpio_register_callback(CYBSP_D8, 
                                 &gpio_btn_callback_data);
    cyhal_gpio_enable_event(CYBSP_D8, CYHAL_GPIO_IRQ_RISE, 
                               GPIO_INTERRUPT_PRIORITY, true);

    // Enable interrupts
    __enable_irq();

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

    printf("Probing SHT31...");
    //while (sht3x_probe(SHT3X_I2C_ADDR_DFLT) != STATUS_OK) {
    //    printf("error\r\n");
    //    cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
    //}
    printf("done\r\n");

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

        //if (!rfm95_send_receive_cycle(&rfm_handle, data_packet, sizeof(data_packet))) {
        //    printf("RFM95 send failed\r\n");
        //} else {
        //    printf("RFM95 send success\r\n");
        //}
    
        //int tick = get_tick();
        //sleep_until_tick(tick+1000);
        //printf("target: %d, actual %d\r\n", tick+1000, get_tick());
        //printf("diff: %d\r\n", tick+1000-get_tick());

        cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
    }
}

