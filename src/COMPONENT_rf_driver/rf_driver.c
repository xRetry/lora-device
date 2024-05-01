#include "rf_driver.h"
#include "cyhal_system.h"
#include "cyhal_trng.h"
#include "cycfg_pins.h"
#include "cy_retarget_io.h"

#define GPIO_INTERRUPT_PRIORITY (7u)

/*****************************    Global variables definition   *****************************/
volatile RF_BoardMode _mode;

uint8_t _thisAddress;
bool _promiscuous;

volatile uint8_t _rxHeaderTo;
volatile uint8_t _rxHeaderFrom;
volatile uint8_t _rxHeaderId;
volatile uint8_t _rxHeaderFlags;

volatile uint8_t _txHeaderTo;
volatile uint8_t _txHeaderFrom;
volatile uint8_t _txHeaderId;
volatile uint8_t _txHeaderFlags;

volatile uint8_t _lastRssi;
int8_t _lastSNR;
bool _enableCRC;

volatile uint16_t _rxBad;
volatile uint16_t _rxGood;

volatile uint16_t _txBad;
volatile uint16_t _txGood;

volatile uint8_t _buf[RH_RF95_MAX_PAYLOAD_LEN];
volatile uint8_t _bufLen;

volatile bool _cad;
volatile bool _rxBufValid;
bool _usingHFport;
uint64_t _cad_timeout;
cyhal_trng_t trng_obj;
cyhal_gpio_callback_data_t callback_data;

volatile bool check = 0;


// Set up communication with the LoRa Board
bool initRFBoard() {
    if (!initSpi(&rf_spi_config))
    {
    printf("SPI Init failed!\n\r");
    return false;
    }

    // manual reset (currently does nothing as it`s not connected on the shield)
    cyhal_system_delay_ms(50);
    cyhal_gpio_write(rf_reset_pin, GPIO_PIN_OUTPUT_LOW);
    cyhal_system_delay_ms(10);
    cyhal_gpio_write(rf_reset_pin, GPIO_PIN_OUTPUT_HIGH);
    cyhal_system_delay_ms(50);

    // This is done to set the long range mode
    spiWrite(&rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE, RH_SPI_WRITE_MASK);

    cyhal_system_delay_ms(20);

    uint8_t readBuffer = 0x00;
    spiRead(&rf_spi_config, RH_RF95_REG_01_OP_MODE, &readBuffer, RH_SPI_WRITE_MASK);
    uint8_t compareVal = RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE;
    if (readBuffer != compareVal) {
        printf("Mode set failed!\n\r");
        printf("I read: 0x%x when I should have read: 0x%x\n\r", readBuffer, compareVal);
        return false;
    }

    spiWrite(&rf_spi_config, RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0x00, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0x00, RH_SPI_WRITE_MASK);

    if (!setMode(Idle)) {
        printf("Setting Board mode failed!\n\r");
        return false;
    }

    setModemConfig(Bw125Cr45Sf128); // Radio default
    setPreambleLength(8);           // Default is 8

    setFrequency(rf_center_frequency);

    setTxPower(23, false);

    if (!interruptSetup()) {
        printf("Interrupt Setup failed!\n\r");
    }

    cyhal_trng_init(&trng_obj);

    printf("Initialized successfully!\n\r");

    //mutex = wiced_rtos_create_mutex();
    //if (wiced_rtos_init_mutex(mutex) != WICED_SUCCESS)
    //printf("Mutex init failed!\n\r");
    return true;
}

void rfISR(void *userdata, uint8_t val) {
    //lock(mutex);
    uint8_t irq_flags = 0;
    uint8_t hop_channel = 0;
    printf("ISR STARTED!\n\r");
    spiRead(&rf_spi_config, RH_RF95_REG_12_IRQ_FLAGS, &irq_flags, RH_SPI_WRITE_MASK);
    spiRead(&rf_spi_config, RH_RF95_REG_1C_HOP_CHANNEL, &hop_channel, RH_SPI_WRITE_MASK);

    spiWrite(&rf_spi_config, RH_RF95_REG_12_IRQ_FLAGS, 0xff, RH_SPI_WRITE_MASK);

    if (_mode == Rx && ((irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR)) || (_enableCRC && !(hop_channel & RH_RF95_RX_PAYLOAD_CRC_IS_ON)))) {
        _rxBad++;
        clearRxBuf();
    } else if (_mode == Rx && (irq_flags & RH_RF95_RX_DONE)) {
        uint8_t len;
        spiRead(&rf_spi_config, RH_RF95_REG_13_RX_NB_BYTES, &len, RH_SPI_WRITE_MASK);
        uint8_t current_fifo_address;
        spiRead(&rf_spi_config, RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR, &current_fifo_address, RH_SPI_WRITE_MASK);
        spiWrite(&rf_spi_config, RH_RF95_REG_0D_FIFO_ADDR_PTR, current_fifo_address, RH_SPI_WRITE_MASK);

        uint8_t tempBuff[len];
        spiBurstRead(&rf_spi_config, RH_RF95_REG_00_FIFO, tempBuff, len, RH_SPI_WRITE_MASK);
        for (uint8_t i = 0; i < len; i++) {
            _buf[i] = tempBuff[i + 1];
        }
        _bufLen = len;

        uint8_t temp;
        spiRead(&rf_spi_config, RH_RF95_REG_19_PKT_SNR_VALUE, &temp, RH_SPI_WRITE_MASK);
        _lastSNR = (int8_t)temp / 4;

        spiRead(&rf_spi_config, RH_RF95_REG_1A_PKT_RSSI_VALUE, &temp, RH_SPI_WRITE_MASK);
        _lastRssi = temp;

        if (_lastSNR < 0) {
            _lastRssi = _lastRssi + _lastSNR;
        } else {
            _lastRssi = (int)_lastRssi * 16 / 15;
        }
        if (_usingHFport) {
            _lastRssi -= 157;
        } else {
            _lastRssi -= 164;
        }

        validateRxBuf();
        if (_rxBufValid) {
            setMode(Idle);
        }
    } else if (_mode == Tx && (irq_flags & RH_RF95_TX_DONE)) {
        _txGood++;
        setMode(Idle);
    } else if (_mode == Cad && (irq_flags & RH_RF95_CAD_DONE)) {
        _cad = irq_flags & RH_RF95_CAD_DETECTED;
        setMode(Idle);
    }
    // TODO(marco)
    // wiced_hal_gpio_clear_pin_interrupt_status(rf_interrupt_pin);
    //unlock(mutex);
}

void clearRxBuf() {
    _rxBufValid = false;
    _bufLen = 0;
}

void validateRxBuf() {
    if (_bufLen < 4) { return; }
    _rxHeaderTo = _buf[0];
    _rxHeaderFrom = _buf[1];
    _rxHeaderId = _buf[2];
    _rxHeaderFlags = _buf[3];

    if (_promiscuous || _rxHeaderTo == _thisAddress || _rxHeaderTo == RH_BROADCAST_ADDRESS) {
        _rxGood++;
        _rxBufValid = true;
    }
}

bool sendRF(uint8_t *data, uint8_t length) {
    // because we need 1 byte more to also send the address in spiBurstWrite
    if (length > RH_RF95_MAX_MESSAGE_LEN - 1) {
        return false;
    }

    yieldSendingPackage();
    setMode(Idle);

    if (!waitCAD()) {
    printf("CAD wait failed!\n\r");
        return false;
    }

    spiWrite(&rf_spi_config, RH_RF95_REG_0D_FIFO_ADDR_PTR, 0, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderTo, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderFrom, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderId, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderFlags, RH_SPI_WRITE_MASK);

    spiBurstWrite(&rf_spi_config, RH_RF95_REG_00_FIFO, data, length, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_22_PAYLOAD_LENGTH, length + RH_RF95_HEADER_LEN, RH_SPI_WRITE_MASK);

    setMode(Tx); // When Tx is done, ISR should be executed and the mode is set to idle again

    return true;
}

bool receiveRF(uint8_t *buff, uint8_t *length) {
    //lock(mutex);
    if (buff && length) {
        if (*length > _bufLen - RH_RF95_HEADER_LEN) {
            *length = _bufLen - RH_RF95_HEADER_LEN;
        }

        for (uint8_t i = 0; i < *length; i++) {
            buff[i] = _buf[i + RH_RF95_HEADER_LEN];
        }
    } else {
        printf("Pointers to buff and/or length ar NULL!\n\r");
    }
    clearRxBuf();
    //unlock(mutex);
    return true;
}

// if a new message is available
bool available() {
    if (_mode == Rx) {
        return false;
    }
    setMode(Rx);
    return _rxBufValid;
}

bool waitCAD() {
    if (!_cad_timeout) {
        return true;
    }

    uint16_t passed_milliseconds = 0;
    while (isChannelActive()) {
        // TODO(marco): What is going on here? `passed_milliseconds` stays 0?
        if (passed_milliseconds > _cad_timeout) {
            return false;
        }
        
        uint32_t rand_num = cyhal_trng_generate(&trng_obj);
        cyhal_system_delay_ms(rand_num);
    }

    return true;
}

bool isChannelActive() {
    if (_mode != Cad) {
        spiWrite(&rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD, RH_SPI_WRITE_MASK);
        spiWrite(&rf_spi_config, RH_RF95_REG_40_DIO_MAPPING1, 0x80, RH_SPI_WRITE_MASK);
        _mode = Cad;
    }

    while (_mode == Cad) {
        YIELD;
    }

    return _cad;
}

// set operating mode of RF_board
bool setMode(RF_BoardMode mode) {
    switch (mode) {
        case Sleep:
            spiWrite(&rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP, RH_SPI_WRITE_MASK);
            _mode = Sleep;
            break;

        case Idle:
            spiWrite(&rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY, RH_SPI_WRITE_MASK);
            _mode = Idle;
            break;

        case Rx:
            spiWrite(&rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS, RH_SPI_WRITE_MASK);
            spiWrite(&rf_spi_config, RH_RF95_REG_40_DIO_MAPPING1, 0x00, RH_SPI_WRITE_MASK);
            _mode = Rx;
            break;

        case Tx:
            spiWrite(&rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX, RH_SPI_WRITE_MASK);
            spiWrite(&rf_spi_config, RH_RF95_REG_40_DIO_MAPPING1, 0x40, RH_SPI_WRITE_MASK);
            _mode = Tx;
            break;
        default:
            return false;
            break;
    }
    return true;
}

// set frequency (copied from arduino lib)
bool setFrequency(float centre) {
    uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    spiWrite(&rf_spi_config, RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_08_FRF_LSB, frf & 0xff, RH_SPI_WRITE_MASK);

    return true;
}

// Set one of the canned FSK Modem configs
//  Returns true if its a valid choice
bool setModemConfig(ModemConfigChoice index) {
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig))) {
        return false;
    }

    ModemConfig cfg;
    memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

bool yieldSendingPackage() {
    while (_mode == Tx) {
        printf("WAITING UNTIL TX IS DONE!\r\n");
        cyhal_system_delay_ms(1000);
    }
    return true;
}

// Sets registers from a canned modem configuration structure
void setModemRegisters(const ModemConfig *config) {
    spiWrite(&rf_spi_config, RH_RF95_REG_1D_MODEM_CONFIG1, config->reg_1d, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_1E_MODEM_CONFIG2, config->reg_1e, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_26_MODEM_CONFIG3, config->reg_26, RH_SPI_WRITE_MASK);
}

// In promiscuous mode all messages regardless of the header will be accepted!
void setPromiscuous(bool value) {
    _promiscuous = value;
}

void setThisAddress(uint8_t value) {
    _thisAddress = value;
}

void setHeaderTo(uint8_t value) {
    _txHeaderTo = value;
}

void setHeaderFrom(uint8_t value) {
    _txHeaderFrom = value;
}

void setHeaderId(uint8_t value) {
    _txHeaderFlags = value;
}

void setHeaderFlags(uint8_t set, uint8_t clear) {
    _txHeaderFlags &= ~clear;
    _txHeaderFlags |= set;
}

// Set length of preamble in bytes, 8 is default
void setPreambleLength(uint16_t bytes) {
    spiWrite(&rf_spi_config, RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8, RH_SPI_WRITE_MASK);
    spiWrite(&rf_spi_config, RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff, RH_SPI_WRITE_MASK);
}

void setTxPower(int8_t power, bool useRFO) {
    // Sigh, different behaviours depending on whether the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO) {
        if (power > 15) { power = 15; }
        if (power < 0) { power = 0; }
        // Set the MaxPower register to 0x7 => MaxPower = 10.8 + 0.6 * 7 = 15dBm
        // So Pout = Pmax - (15 - power) = 15 - 15 + power
        spiWrite(&rf_spi_config, RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | power, RH_SPI_WRITE_MASK);
        spiWrite(&rf_spi_config, RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE, RH_SPI_WRITE_MASK);
    } else {
        if (power > 20) { power = 20; }
        if (power < 2) { power = 2; }

        // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
        // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will use it
        // for 8, 19 and 20dBm
        if (power > 17) {
            spiWrite(&rf_spi_config, RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE, RH_SPI_WRITE_MASK);
            power -= 3;
        } else {
            spiWrite(&rf_spi_config, RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE, RH_SPI_WRITE_MASK);
        }

        // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
        // pin is connected, so must use PA_BOOST
        // Pout = 2 + OutputPower (+3dBm if DAC enabled)
        spiWrite(&rf_spi_config, RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power - 2), RH_SPI_WRITE_MASK);
    }
}

// Here we setup the interrupt functionality of the INT pin
bool interruptSetup() {
    // TODO(marco): Maybe 1 as init value
    cyhal_gpio_init(rf_interrupt_pin, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, 0);
    callback_data.callback = rfISR;
    cyhal_gpio_register_callback(CYBSP_D2, &callback_data);
    cyhal_gpio_enable_event(CYBSP_D2, CYHAL_GPIO_IRQ_RISE, 
                                 GPIO_INTERRUPT_PRIORITY, true);

    //wiced_hal_gpio_init();
    //wiced_hal_gpio_configure_pin(rf_interrupt_pin, (GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_LEVEL_HIGH), GPIO_PIN_OUTPUT_LOW);
    //wiced_hal_gpio_register_pin_for_interrupt(rf_interrupt_pin, rfISR, NULL);
    return true;
}
