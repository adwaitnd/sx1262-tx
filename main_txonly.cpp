/*!
 * \file      main_pingpong.cpp
 *
 * \brief     Ping implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 * - Changed to be transmit only code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 */

#include "mbed.h"
#include "sx126x-hal.h"
#include "sx126x.h"
//#include "debug.h"
 
#define SEND_PING_BEAT_US 300000
#define RX_TIMEOUT_US 200000

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1
 
/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA
 
#define RF_FREQUENCY                                    434000000 // Hz
#define TX_OUTPUT_POWER                                 10         // max 14 dBm
 
#if USE_MODEM_LORA == 1
 
    #define LORA_BANDWIDTH                              LORA_BW_125         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       LORA_SF10         // [SF7..SF12]
    #define LORA_LOWDATARATEOPTIMIZE                    0
    #define LORA_CODINGRATE                             LORA_CR_4_5         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_HEADER_TYPE                            LORA_PACKET_VARIABLE_LENGTH
    #define LORA_FHSS_ENABLED                           false  
    #define LORA_NB_SYMB_HOP                            4     
    #define LORA_IQ                                     LORA_IQ_NORMAL
    #define LORA_CRC_MODE                               LORA_CRC_OFF
 
#elif USE_MODEM_FSK == 1
 
    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               RX_BW_93800     // Hz
    #define FSK_MODULATION_SHAPPING                     MOD_SHAPING_G_BT_05
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_HEADER_TYPE                             RADIO_PACKET_VARIABLE_LENGTH
    #define FSK_CRC_MODE                                RADIO_CRC_2_BYTES_CCIT
    #define FSK_ADDR_FILTERING                          RADIO_ADDRESSCOMP_FILT_NODE;
    #define FSK_WHITENING_MODE                          RADIO_DC_FREE_OFF
    #define FSK_PREAMBLE_DETECTOR_MODE                  RADIO_PREAMBLE_DETECTOR_OFF
    #define FSK_SYNCWORD_LENGTH                         8
#else
    #error "Please define a modem in the compiler options."
#endif
 
#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     32        // Define the payload size here
 
#if( defined ( TARGET_KL25Z ) || defined ( TARGET_LPC11U6X ) )
DigitalOut led( LED2 );
#else
DigitalOut led( LED1 );
#endif

//DigitalOut led1( LED1 );
//DigitalOut led2( LED2 );

 /*
 * Callback functions prototypes
 */
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );
 
/*!
 * @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );
 
/*!
 * @brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );
 
/*!
 * @brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );
 
/*!
 * @brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t errCode );
 
/*!
 * @brief Function executed on Radio Fhss Change Channel event
 */
void OnFhssChangeChannel( uint8_t channelIndex );

typedef struct{
    RadioPacketTypes_t packetType;
    int8_t txPower;
    RadioRampTimes_t txRampTime;
    ModulationParams_t modParams;
    PacketParams_t packetParams;
    uint32_t rfFrequency;
    uint16_t irqTx;
    uint16_t irqRx;
    uint32_t txTimeout;
    uint32_t rxTimeout;
}RadioConfigurations_t;
RadioConfigurations_t radioConfiguration;
 
/*
 *  Global variables declarations
 */
typedef enum
{
    SEND_PACKET,
    WAIT_SEND_DONE,
}AppStates_t;
volatile AppStates_t State = SEND_PACKET;

typedef struct{
    bool rxDone;
    bool rxError;
    bool txDone;
    bool rxTimeout;
    bool txTimeout;
}RadioFlags_t;
RadioFlags_t radioFlags = {
    .txDone = false,
    .txTimeout = false,
};
 
/*!
 * Radio events function pointer
 */
static RadioCallbacks_t RadioEvents = {
    .txDone = &OnTxDone,
    .txTimeout = &OnTxTimeout,
    .rxDone = NULL,
    .rxPreambleDetect = NULL,
    .rxHeaderDone = NULL,
    .rxTimeout = NULL,
    .rxError = NULL,
    .cadDone = NULL,
};

/*
 *  Global variables declarations
 */
//Radio Radio( NULL );
#define MESSAGE_SIZE 4
typedef uint8_t Messages_t[MESSAGE_SIZE];
const Messages_t PingMsg = {'P', 'I', 'N', 'G'};
const Messages_t *messageToSend = &PingMsg;

SX126xHal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, A1, A2, D8, &RadioEvents );

void SetToMaster(void);
void RunTXStateMachine();
void SetConfiguration(RadioConfigurations_t *config);
void ConfigureGeneralRadio(SX126xHal *radio, RadioConfigurations_t *config);
void ConfigureRadioTx(SX126xHal *radio, RadioConfigurations_t *config);
void PrepareBuffer(SX126xHal *radio, const Messages_t *messageToSend);
bool proceed = true;
uint32_t count = 0;

// function to run
void MasterSendNextEvent(void)
{
    proceed = true;
}

Ticker masterSendNextTicker;

Serial serial(USBTX, USBRX);

int main( void ) 
{
    Radio.Reset();
    Radio.Init();
    serial.baud(115200);
    SetConfiguration(&radioConfiguration);
    ConfigureGeneralRadio(&Radio, &radioConfiguration);
    
    // initialize transmitter
    printf("-->TX mode\n");
    State = SEND_PACKET;
    messageToSend = &PingMsg;
    masterSendNextTicker.attach_us(&MasterSendNextEvent, SEND_PING_BEAT_US);
    while(true){
        RunTXStateMachine();
    }
}

// let's keep the state machine because waiting for transmission to be done
// will take multiple instances of the state machine running. We could use a
// for loop, but the state machine seems g
void RunTXStateMachine(){
    switch(State){
        case SEND_PACKET:{
            if (proceed == true) {
                // wait for timer to say it's fine to proceed with transmission
                proceed = false;
                count++;
                PrepareBuffer(&Radio, messageToSend);
                ConfigureRadioTx(&Radio, &radioConfiguration);
                Radio.SetTx(radioConfiguration.txTimeout);
                printf("%u: TX", count);
                State = WAIT_SEND_DONE;
            }
            break;
        }
        
        case WAIT_SEND_DONE:{
            if(radioFlags.txDone){
                // transmission successful, wait
                radioFlags.txDone = false;  // reset interrupted flag
                printf(" done\r\n");
                State = SEND_PACKET;
            }
            if(radioFlags.txTimeout){
                // transmission failed, try again!
                radioFlags.txTimeout = false;  // reset interrupted flag
                printf(" err\r\n");
                State = SEND_PACKET;
            }
            break;
        }        
    }
}


// Interrupt handler functions - Adwait

void SetToMaster(){
    
}
 
void OnTxDone( void )
{
    radioFlags.txDone = true;
}
 
void OnTxTimeout( void )
{
    radioFlags.txTimeout = true;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

// Other helper functions - Adwait

void SetConfiguration(RadioConfigurations_t *config){
    config->irqRx = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
    config->irqTx = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    config->rfFrequency = RF_FREQUENCY;
    config->txTimeout = 0;
    config->rxTimeout = (uint32_t)(RX_TIMEOUT_US / 15.625);
    config->txPower = TX_OUTPUT_POWER;
    config->txRampTime = RADIO_RAMP_200_US;
    #if USE_MODEM_LORA == 1
        config->packetType = PACKET_TYPE_LORA;
        config->modParams.PacketType = PACKET_TYPE_LORA;
        config->modParams.Params.LoRa.Bandwidth = LORA_BANDWIDTH;
        config->modParams.Params.LoRa.CodingRate = LORA_CODINGRATE;
        config->modParams.Params.LoRa.LowDatarateOptimize = LORA_LOWDATARATEOPTIMIZE;
        config->modParams.Params.LoRa.SpreadingFactor = LORA_SPREADING_FACTOR;
        config->packetParams.PacketType = PACKET_TYPE_LORA;
        config->packetParams.Params.LoRa.CrcMode = LORA_CRC_MODE;
        config->packetParams.Params.LoRa.HeaderType = LORA_HEADER_TYPE;
        config->packetParams.Params.LoRa.InvertIQ = LORA_IQ;
        config->packetParams.Params.LoRa.PayloadLength = BUFFER_SIZE;
        config->packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    #elif USE_MODEM_FSK == 1
        config->packetType = PACKET_TYPE_GFSK;
        config->modParams.PacketType = PACKET_TYPE_GFSK;
        config->modParams.Params.Gfsk.Bandwidth = FSK_BANDWIDTH;
        config->modParams.Params.Gfsk.BitRate = 1024000000 / FSK_DATARATE;
        config->modParams.Params.Gfsk.Fdev = FSK_FDEV * 1.048576;
        config->modParams.Params.Gfsk.ModulationShaping = FSK_MODULATION_SHAPPING;
        config->packetParams.PacketType = PACKET_TYPE_GFSK;
        config->packetParams.Params.Gfsk.AddrComp = FSK_ADDR_FILTERING;
        config->packetParams.Params.Gfsk.CrcLength = FSK_CRC_MODE;
        config->packetParams.Params.Gfsk.DcFree = FSK_WHITENING_MODE;
        config->packetParams.Params.Gfsk.HeaderType = FSK_HEADER_TYPE;
        config->packetParams.Params.Gfsk.PayloadLength = BUFFER_SIZE;
        config->packetParams.Params.Gfsk.PreambleLength = FSK_PREAMBLE_LENGTH;
        config->packetParams.Params.Gfsk.PreambleMinDetect = FSK_PREAMBLE_DETECTOR_MODE;
        config->packetParams.Params.Gfsk.SyncWordLength = FSK_SYNCWORD_LENGTH;
    #endif
}

void ConfigureGeneralRadio(SX126xHal *radio, RadioConfigurations_t *config){
    radio->SetPacketType(config->packetType);
    radio->SetPacketParams(&config->packetParams);
    radio->SetModulationParams(&config->modParams);
    radio->SetRfFrequency(config->rfFrequency);
    radio->SetTxParams(config->txPower, config->txRampTime);
    radio->SetInterruptMode();
    if(config->packetType == PACKET_TYPE_GFSK){
        uint8_t syncword[8] = {0xF0, 0x0F, 0x55, 0xAA, 0xF0, 0x0F, 0x55, 0xAA};
        radio->SetSyncWord(syncword);
    }
}

void ConfigureRadioTx(SX126xHal *radio, RadioConfigurations_t *config){
    radio->SetDioIrqParams(config->irqTx, config->irqTx, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

void PrepareBuffer(SX126xHal *radio, const Messages_t *messageToSend){
    radio->SetPayload((uint8_t*)messageToSend, MESSAGE_SIZE);
}