
#if !defined(RF_DRIVER_H_)
#define RF_DRIVER_H_

/*****************************    Includes   *****************************/
//#include "wiced_bt_dev.h"
//#include "sparcommon.h"
//#include "wiced_hal_puart.h"
//#include "wiced_hal_pspi.h"
//#include "wiced_bt_trace.h"
//#include "wiced_hal_gpio.h"
//#include "wiced_timer.h"
//#include "wiced_hal_rand.h"
//#include "wiced_bt_stack.h"
//#include "wiced_rtos.h"
//#include "wiced_memory.h"
//#include "bt_types.h"
//#include "memory_Helper.h"
#include "spi_driver.h"

#include <stdbool.h>

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

#define YIELD

#define lock(mutex) wiced_rtos_lock_mutex(mutex)
#define unlock(mutex) wiced_rtos_unlock_mutex(mutex)

/**********************************************************************/
/*                                                                    */
/*                             Data types                             */
/*                                                                    */
/**********************************************************************/
typedef enum RF_BoardMode
{
	Sleep,
	Idle,
	Rx,
	Tx,
	Cad
} RF_BoardMode;

typedef enum ModemConfigChoice
{
	Bw125Cr45Sf128 = 0, ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
	Bw500Cr45Sf128,			///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
	Bw31_25Cr48Sf512,		///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	Bw125Cr48Sf4096,		///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
	Bw125Cr45Sf2048,		///< Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
} ModemConfigChoice;

typedef struct
{
	uint8_t reg_1d; ///< Value for register RH_RF95_REG_1D_MODEM_CONFIG1
	uint8_t reg_1e; ///< Value for register RH_RF95_REG_1E_MODEM_CONFIG2
	uint8_t reg_26; ///< Value for register RH_RF95_REG_26_MODEM_CONFIG3
} ModemConfig;

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
static const ModemConfig MODEM_CONFIG_TABLE[] =
		{
				//  1d,     1e,      26
				{0x72, 0x74, 0x04}, // Bw125Cr45Sf128 (the chip default), AGC enabled
				{0x92, 0x74, 0x04}, // Bw500Cr45Sf128, AGC enabled
				{0x48, 0x94, 0x04}, // Bw31_25Cr48Sf512, AGC enabled
				{0x78, 0xc4, 0x0c}, // Bw125Cr48Sf4096, AGC enabled
				{0x72, 0xb4, 0x04}, // Bw125Cr45Sf2048, AGC enabled

};

/**********************************************************************/
/*                                                                    */
/*                           GLOBAL VARIABLES                         */
/*                                                                    */
/**********************************************************************/
extern volatile RF_BoardMode _mode;
extern wiced_mutex_t *mutex;

extern UINT8 _thisAddress;
extern bool _promiscuous;

extern volatile UINT8 _rxHeaderTo;
extern volatile UINT8 _rxHeaderFrom;
extern volatile UINT8 _rxHeaderId;
extern volatile UINT8 _rxHeaderFlags;

extern volatile UINT8 _txHeaderTo;
extern volatile UINT8 _txHeaderFrom;
extern volatile UINT8 _txHeaderId;
extern volatile UINT8 _txHeaderFlags;

extern volatile UINT8 _lastRssi;
extern int8_t _lastSNR;
extern bool _enableCRC;

extern volatile UINT16 _rxBad;
extern volatile UINT16 _rxGood;

extern volatile UINT16 _txBad;
extern volatile UINT16 _txGood;

extern volatile UINT8 _buf[RH_RF95_MAX_PAYLOAD_LEN];
extern volatile UINT8 _bufLen;

extern volatile bool _cad;
extern volatile bool _rxBufValid;
extern bool _usingHFport;
extern UINT64 _cad_timeout;

extern wiced_bt_buffer_pool_t *private_pool;

extern wiced_bt_gpio_numbers_t rf_reset_pin;		 // set in main.c
extern wiced_bt_gpio_numbers_t rf_interrupt_pin; // set in main.c
extern float rf_center_frequency;								 // set in main.c
extern spi_config_t rf_spi_config;							 // set in main.c

/**********************************************************************/
/*                                                                    */
/*                           FUNCTION SIGNATURES                      */
/*                                                                    */
/**********************************************************************/
/// @brief Initalizes the hardware and software
/// default: 13dBm, Bw = 125kHz, Cr =4/5, Sf =128chips/symbol, CRC on
/// @return true if successful
bool initRFBoard();

/// @brief Checks if it is possible to send at the moment then loads mesage to transmitter and sets Rx mode
/// @param data pointer to the message to send
/// @param length size in bytes to send
/// @return true if message has been copied successfully; false if Channel is not clear
bool sendRF(UINT8 *data, UINT8 length);

/// @brief Check if a valid message is present in _buf; discards the header bytes
/// @param buff Location to copy the received message to
/// @param length Pointer to available space in buff; is set to actual number of bytes received
/// @return true, if a valid message was copied to buff
bool receiveRF(UINT8 *buff, UINT8 *length);

/// @brief In promiscuous mode all messages are accepted, regardless of the header
/// @param value true or false
void setPromiscuous(bool value);

/// @brief Set Address of this node
/// @param value
void setThisAddress(UINT8 value);

/// @brief Sets the TO header to be sent in all subsequent messages
/// @param value
void setHeaderTo(UINT8 value);

/// @brief Sets the FROM header to be sent in all subsequent messages
/// @param value
void setHeaderFrom(UINT8 value);

/// @brief Sets the ID header to be sent in all subsequent messages
/// @param value
void setHeaderId(UINT8 value);

/// @brief Sets and clears bits in the FLAGS header to be sent in all subsequent messages
/// First it clears the FLAGS  according to the clear argument, then sets the FLAGS according to the
/// set argument. The default for clear always clears the application soecific FLAGS
/// @param set Bitmask of the bits to be set. FLAGS are cleared with the clear mask before being set.
/// @param clear Bitmask of the FLAGS to clear. Defaults to RH_FLAGS_APPLICATION_SPECIFIC
///            which clears the application specific flags, resulting in new application specific flags
///            identical to the set.
void setHeaderFlags(UINT8 set, UINT8 clear);

/**********************************************************************/
/*                                                                    */
/*                           HELPER FUNCTIONS                         */
/*                                                                    */
/**********************************************************************/

/// @brief Sets the length of the preamble in bytes;
/// Caution: Set to the same value across all nodes!
/// @param bytes
void setPreambleLength(uint16_t bytes);

/// @brief Sets the transmitter power output level and configures the transmitter pin
/// @param power Tranimtter power level in dBm, Valid from +2 to +20
/// @param useRFO If true, enables the use of the RFO transmitter pins instead of the PA_BOOST pin
void setTxPower(int8_t power, bool useRFO);

/// @brief Sets all the registers required to configure the data modem in the radio#
/// @param config A ModemConfig structure containing values for the config registers
void setModemRegisters(const ModemConfig *config);

/// @brief Sets the mode to the wanted value
/// @param mode value to set
/// @return true if successful
bool setMode(RF_BoardMode mode);

/// @brief Sets the transmitter and receiver center frequency
/// @param centre Frequency in MHz 137.0 to 1020.0; check for freqeuncy range of specific radio module
/// @return true if frequency is within range
bool setFrequency(float centre);

/// @brief Select one the predefined modem configs. If the required config is not provided, use setModemRegisters().
/// @param index The config choice
/// @return true if index is a valid choice
bool setModemConfig(ModemConfigChoice index);

/// @brief Blocks until the transmitter is no longer transmitting
/// @return true if radio completed transmission
bool yieldSendingPackage();

/// @brief Blocks channel activity until activity is finished or CAD timeout occurs.
/// Using the radio's integrated CAD function (if supported) to detect channel activity
/// @return true if channel is clear, or CAD is not supported
bool waitCAD();

/// @brief Check if currrently selected radio channel is active.
/// @return true if channel is in use
bool isChannelActive();

/// @brief Checks if a new message is available from the driver
/// @return true if a new complete and error free message is available to be retreived by receiveRF()
bool available();

/// @brief Clear the local receive buffer
void clearRxBuf();

/// @brief Examine the receive buffer to determine wether the message is for this node
void validateRxBuf();

/**********************************************************************/
/*                                                                    */
/*                           UTILITY FUNCTIONS                        */
/*                                                                    */
/**********************************************************************/
/// @brief Initalizes the hardware and software
/// default: 13dBm, Bw = 125kHz, Cr =4/5, Sf =128chips/symbol, CRC on
/// @return true if successful
bool initRFBoard();

/// @brief initializes the interrupt pin
/// @return true when succeeful
bool interruptSetup();

/// @brief Interrupt service routine, should not be called manually
/// The paramters are unused and are only present because the WICED library needs this paramters
/// @param userdata set NULL
/// @param val not used
void rfISR(void *userdata, UINT8 val);

#endif // RF_DRIVER_H_
