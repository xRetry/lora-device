
#include "RF_Driver.h"

/*****************************    Global variables definition   *****************************/
volatile RF_BoardMode _mode;
wiced_mutex_t *mutex;

UINT8 _thisAddress;
bool _promiscuous;

volatile UINT8 _rxHeaderTo;
volatile UINT8 _rxHeaderFrom;
volatile UINT8 _rxHeaderId;
volatile UINT8 _rxHeaderFlags;

volatile UINT8 _txHeaderTo;
volatile UINT8 _txHeaderFrom;
volatile UINT8 _txHeaderId;
volatile UINT8 _txHeaderFlags;

volatile UINT8 _lastRssi;
int8_t _lastSNR;
bool _enableCRC;

volatile UINT16 _rxBad;
volatile UINT16 _rxGood;

volatile UINT16 _txBad;
volatile UINT16 _txGood;

volatile UINT8 _buf[RH_RF95_MAX_PAYLOAD_LEN];
volatile UINT8 _bufLen;

volatile bool _cad;
volatile bool _rxBufValid;
bool _usingHFport;
UINT64 _cad_timeout;

wiced_bt_buffer_pool_t *private_pool;

volatile bool check = 0;


// Set up communication with the LoRa Board
bool initRFBoard()
{
  if (!initSpi(rf_spi_config))
  {
    WICED_BT_TRACE("SPI Init failed!\n\r");
    return false;
  }

  // manual reset (currently does nothing as it`s not connected on the shield)
  wiced_rtos_delay_milliseconds(50, KEEP_THREAD_ACTIVE);
  wiced_hal_gpio_set_pin_output(rf_reset_pin, GPIO_PIN_OUTPUT_LOW);
  wiced_rtos_delay_milliseconds(10, KEEP_THREAD_ACTIVE);
  wiced_hal_gpio_set_pin_output(rf_reset_pin, GPIO_PIN_OUTPUT_HIGH);
  wiced_rtos_delay_milliseconds(50, KEEP_THREAD_ACTIVE);

  // This is done to set the long range mode
  spiWrite(rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE, RH_SPI_WRITE_MASK);

  wiced_rtos_delay_milliseconds(20, KEEP_THREAD_ACTIVE);

  UINT8 readBuffer = 0x00;
  spiRead(rf_spi_config, RH_RF95_REG_01_OP_MODE, &readBuffer, RH_SPI_WRITE_MASK);
  UINT8 compareVal = RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE;
  if (readBuffer != compareVal)
  {
    WICED_BT_TRACE("Mode set failed!\n\r");
    WICED_BT_TRACE("I read: 0x%x when I should have read: 0x%x\n\r", readBuffer, compareVal);
    return false;
  }

  spiWrite(rf_spi_config, RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0x00, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0x00, RH_SPI_WRITE_MASK);

  if (!setMode(Idle))
  {
    WICED_BT_TRACE("Setting Board mode failed!\n\r");
    return false;
  }

  setModemConfig(Bw125Cr45Sf128); // Radio default
  setPreambleLength(8);           // Default is 8
 
  setFrequency(rf_center_frequency);
 
  setTxPower(23, false);

  if (!interruptSetup())
    WICED_BT_TRACE("Interrupt Setup failed!\n\r");

  WICED_BT_TRACE("Initialized successfully!\n\r");

  mutex = wiced_rtos_create_mutex();
  if (wiced_rtos_init_mutex(mutex) != WICED_SUCCESS)
    WICED_BT_TRACE("Mutex init failed!\n\r");
  return true;
}

void rfISR(void *userdata, UINT8 val)
{
  lock(mutex);
  UINT8 irq_flags = 0;
  UINT8 hop_channel = 0;
  WICED_BT_TRACE("ISR STARTED!\n\r");
  spiRead(rf_spi_config, RH_RF95_REG_12_IRQ_FLAGS, &irq_flags, RH_SPI_WRITE_MASK);
  spiRead(rf_spi_config, RH_RF95_REG_1C_HOP_CHANNEL, &hop_channel, RH_SPI_WRITE_MASK);

  spiWrite(rf_spi_config, RH_RF95_REG_12_IRQ_FLAGS, 0xff, RH_SPI_WRITE_MASK);

  if (_mode == Rx && ((irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR)) || (_enableCRC && !(hop_channel & RH_RF95_RX_PAYLOAD_CRC_IS_ON))))
  {
    _rxBad++;
    clearRxBuf();
  }
  else if (_mode == Rx && (irq_flags & RH_RF95_RX_DONE))
  {
    UINT8 len;
    spiRead(rf_spi_config, RH_RF95_REG_13_RX_NB_BYTES, &len, RH_SPI_WRITE_MASK);
    UINT8 current_fifo_address;
    spiRead(rf_spi_config, RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR, &current_fifo_address, RH_SPI_WRITE_MASK);
    spiWrite(rf_spi_config, RH_RF95_REG_0D_FIFO_ADDR_PTR, current_fifo_address, RH_SPI_WRITE_MASK);
    UINT8 *tempBuff = spiBurstRead(rf_spi_config, RH_RF95_REG_00_FIFO, len, RH_SPI_WRITE_MASK);
    for (UINT8 i = 0; i < len; i++)
    {
      _buf[i] = tempBuff[i + 1];
    }
    wiced_bt_free_buffer(tempBuff);
    _bufLen = len;

    UINT8 temp;
    spiRead(rf_spi_config, RH_RF95_REG_19_PKT_SNR_VALUE, &temp, RH_SPI_WRITE_MASK);
    _lastSNR = (int8_t)temp / 4;

    spiRead(rf_spi_config, RH_RF95_REG_1A_PKT_RSSI_VALUE, &temp, RH_SPI_WRITE_MASK);
    _lastRssi = temp;

    if (_lastSNR < 0)
      _lastRssi = _lastRssi + _lastSNR;
    else
      _lastRssi = (int)_lastRssi * 16 / 15;
    if (_usingHFport)
      _lastRssi -= 157;
    else
      _lastRssi -= 164;

    validateRxBuf();
    if (_rxBufValid)
      setMode(Idle);
  }
  else if (_mode == Tx && (irq_flags & RH_RF95_TX_DONE))
  {
    _txGood++;
    setMode(Idle);
  }
  else if (_mode == Cad && (irq_flags & RH_RF95_CAD_DONE))
  {
    _cad = irq_flags & RH_RF95_CAD_DETECTED;
    setMode(Idle);
  }
  wiced_hal_gpio_clear_pin_interrupt_status(rf_interrupt_pin);
  unlock(mutex);
}

void clearRxBuf()
{
  _rxBufValid = false;
  _bufLen = 0;
}

void validateRxBuf()
{
  if (_bufLen < 4)
    return;
  _rxHeaderTo = _buf[0];
  _rxHeaderFrom = _buf[1];
  _rxHeaderId = _buf[2];
  _rxHeaderFlags = _buf[3];

  if (_promiscuous || _rxHeaderTo == _thisAddress || _rxHeaderTo == RH_BROADCAST_ADDRESS)
  {
    _rxGood++;
    _rxBufValid = true;
  }
}

bool sendRF(UINT8 *data, UINT8 length)
{
  if (length > RH_RF95_MAX_MESSAGE_LEN - 1) // because we need 1 byte more to also send the address in spiBurstWrite
    return false;

  yieldSendingPackage();
  setMode(Idle);

  if (!waitCAD())
  {
    WICED_BT_TRACE("CAD wait failed!\n\r");
    return false;
  }

  spiWrite(rf_spi_config, RH_RF95_REG_0D_FIFO_ADDR_PTR, 0, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderTo, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderFrom, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderId, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_00_FIFO, _txHeaderFlags, RH_SPI_WRITE_MASK);

  spiBurstWrite(rf_spi_config, RH_RF95_REG_00_FIFO, data, length, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_22_PAYLOAD_LENGTH, length + RH_RF95_HEADER_LEN, RH_SPI_WRITE_MASK);

  setMode(Tx); // When Tx is done, ISR should be executed and the mode is set to idle again

  return true;
}

bool receiveRF(UINT8 *buff, UINT8 *length)
{
  lock(mutex);
  if (buff && length)
  {
    if (*length > _bufLen - RH_RF95_HEADER_LEN)
      *length = _bufLen - RH_RF95_HEADER_LEN;

    for (UINT8 i = 0; i < *length; i++)
      buff[i] = _buf[i + RH_RF95_HEADER_LEN];
    
  }
  else
    WICED_BT_TRACE("Pointers to buff and/or length ar NULL!\n\r");
  clearRxBuf();
  unlock(mutex);
  return true;
}

bool available() // if a new message is available
{
  if (_mode == Rx)
  {
    return false;
  }
  setMode(Rx);
  return _rxBufValid;
}

bool waitCAD()
{
  if (!_cad_timeout)
    return true;

  UINT16 passed_milliseconds = 0;
  while (isChannelActive())
  {
    if (passed_milliseconds > _cad_timeout)
      return false;
    wiced_rtos_delay_milliseconds((UINT16)wiced_hal_rand_gen_num(), KEEP_THREAD_ACTIVE); // Sophisticated DCF function/ Algorithm
  }

  return true;
}

bool isChannelActive()
{
  if (_mode != Cad)
  {
    spiWrite(rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD, RH_SPI_WRITE_MASK);
    spiWrite(rf_spi_config, RH_RF95_REG_40_DIO_MAPPING1, 0x80, RH_SPI_WRITE_MASK);
    _mode = Cad;
  }

  while (_mode == Cad)
    YIELD;

  return _cad;
}

// set operating mode of RF_board
bool setMode(RF_BoardMode mode)
{
  switch (mode)
  {
  case Sleep:
    spiWrite(rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP, RH_SPI_WRITE_MASK);
    _mode = Sleep;
    break;

  case Idle:
    spiWrite(rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY, RH_SPI_WRITE_MASK);
    _mode = Idle;
    break;

  case Rx:
    spiWrite(rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS, RH_SPI_WRITE_MASK);
    spiWrite(rf_spi_config, RH_RF95_REG_40_DIO_MAPPING1, 0x00, RH_SPI_WRITE_MASK);
    _mode = Rx;
    break;

  case Tx:
    spiWrite(rf_spi_config, RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX, RH_SPI_WRITE_MASK);
    spiWrite(rf_spi_config, RH_RF95_REG_40_DIO_MAPPING1, 0x40, RH_SPI_WRITE_MASK);
    _mode = Tx;
    break;
  default:
    return false;
    break;
  }
  return true;
}

// set frequency (copied from arduino lib)
bool setFrequency(float centre)
{
  uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
  spiWrite(rf_spi_config, RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_08_FRF_LSB, frf & 0xff, RH_SPI_WRITE_MASK);

  return true;
}

// Set one of the canned FSK Modem configs
//  Returns true if its a valid choice
bool setModemConfig(ModemConfigChoice index)
{
  if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
    return false;

  ModemConfig cfg;
  memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(ModemConfig));
  setModemRegisters(&cfg);

  return true;
}

bool yieldSendingPackage()
{
  while (_mode == Tx)
  {
    WICED_BT_TRACE("WAITING UNTIL TX IS DONE!\r\n");
    wiced_rtos_delay_milliseconds(1000, KEEP_THREAD_ACTIVE);
  }
  return true;
}

// Sets registers from a canned modem configuration structure
void setModemRegisters(const ModemConfig *config)
{
  spiWrite(rf_spi_config, RH_RF95_REG_1D_MODEM_CONFIG1, config->reg_1d, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_1E_MODEM_CONFIG2, config->reg_1e, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_26_MODEM_CONFIG3, config->reg_26, RH_SPI_WRITE_MASK);
}

// In promiscuous mode all messages regardless of the header will be accepted!
void setPromiscuous(bool value)
{
  _promiscuous = value;
}

void setThisAddress(UINT8 value)
{
  _thisAddress = value;
}

void setHeaderTo(UINT8 value)
{
  _txHeaderTo = value;
}

void setHeaderFrom(UINT8 value)
{
  _txHeaderFrom = value;
}

void setHeaderId(UINT8 value)
{
  _txHeaderFlags = value;
}

void setHeaderFlags(UINT8 set, UINT8 clear)
{
  _txHeaderFlags &= ~clear;
  _txHeaderFlags |= set;
}

// Set length of preamble in bytes, 8 is default
void setPreambleLength(uint16_t bytes)
{
  spiWrite(rf_spi_config, RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8, RH_SPI_WRITE_MASK);
  spiWrite(rf_spi_config, RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff, RH_SPI_WRITE_MASK);
}

void setTxPower(int8_t power, bool useRFO)
{
  // Sigh, different behaviours depending on whether the module use PA_BOOST or the RFO pin
  // for the transmitter output
  if (useRFO)
  {
    if (power > 15)
      power = 15;
    if (power < 0)
      power = 0;
    // Set the MaxPower register to 0x7 => MaxPower = 10.8 + 0.6 * 7 = 15dBm
    // So Pout = Pmax - (15 - power) = 15 - 15 + power
    spiWrite(rf_spi_config, RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | power, RH_SPI_WRITE_MASK);
    spiWrite(rf_spi_config, RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE, RH_SPI_WRITE_MASK);
  }
  else
  {
    if (power > 20)
      power = 20;
    if (power < 2)
      power = 2;

    // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
    // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will use it
    // for 8, 19 and 20dBm
    if (power > 17)
    {
      spiWrite(rf_spi_config, RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE, RH_SPI_WRITE_MASK);
      power -= 3;
    }
    else
    {
      spiWrite(rf_spi_config, RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE, RH_SPI_WRITE_MASK);
    }

    // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
    // pin is connected, so must use PA_BOOST
    // Pout = 2 + OutputPower (+3dBm if DAC enabled)
    spiWrite(rf_spi_config, RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power - 2), RH_SPI_WRITE_MASK);
  }
}

// Here we setup the interrupt functionality of the INT pin
bool interruptSetup()
{
  wiced_hal_gpio_init();
  wiced_hal_gpio_configure_pin(rf_interrupt_pin, (GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_LEVEL_HIGH), GPIO_PIN_OUTPUT_LOW);
  wiced_hal_gpio_register_pin_for_interrupt(rf_interrupt_pin, rfISR, NULL);
  return true;
}
