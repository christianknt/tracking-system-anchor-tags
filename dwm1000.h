#ifndef DWM1000_H_INCLUDED
#define DWM1000_H_INCLUDED

#define DEV_ID 0x00 //Identificador del dispositivo
#define EUI 0x01 //Identificador unico del dispositivo
#define PANADR 0x03//PAN identifier and short address
#define SYS_CFG 0x04 //System configuration bitmap
#define SYS_TIME 0x06 //System time
#define TX_FCTRL 0x08 //Transmit frame control
#define TX_BUFFER 0x09 //Transmission buffer
#define DX_TIME 0x0A //Delayed send or receive time
#define SYS_CTRL 0x0D
#define SYS_STATUS 0x0F
#define RX_FINFO 0x10 //Informacion sobre el frame recibido
#define RX_BUFFER 0x11 //Buffer de recepcion
#define RX_TIME 0x15 //Tiempo de recepcion
#define TX_TIME 0x17 //Tiempo de envio
#define TX_POWER 0x1E // TX Power control
#define CHAN_CTRL 0x1F //Registro de control del canal
#define AON_WCFG 0x2C //AON wakeup configuration register
#define AGC_CTRL 0x23 // Automatic Gain Control configuration and control
#define DRX_CONF 0x27 //Digital receiver configuration
#define RF_CONF 0x28 //Analog RF Configuration
#define TX_CAL 0x2A //Transmitter calibration block
#define FS_CTRL 0x2B //Frequency synthesiser control block
#define OTP_IF 0x2D
#define LDE_IF 0x2E
#define PMSC 0x36 //Power Management and System control
//Sub-registers
#define SHORT_ADDR 0
#define PAN_ID 2
#define AGC_TUNE1 0x04
#define AGC_TUNE2 0x0C
#define DRX_TUNE0b 0x02
#define DRX_TUNE1a 0x04
#define DRX_TUNE1b 0x06
#define DRX_TUNE2 0x08 //Offset for digital tuning register 2
#define DRX_TUNE4H 0x26
#define LDE_CFG2 0x1806
#define RF_RXCTRLH 0x0B //Analog RX Control Register
#define RF_TXCTRL 0x0C //Analog TX Control Register
#define TC_PGDELAY 0x0B //Transmitter calibration - Pulse generator delay
#define FS_PLLTUNE 0x0B //Frequency synthesiser -- PLL Tuning


// Time resolution in micro-seconds of time based registers/values.
// Each bit in a timestamp counts for a period of approx. 15.65ps
#define TIME_RES 0.000015650040064103f

#define SPEED_OF_LIGHT 300 //Speed of radio waves in vacuum[m/us]

// Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
#define DISTANCE_OF_RADIO 0.0046917639786159f


#define BEACON 0
#define DATA 1
#define ACKNOWLEDGEMENT 2
#define MACCOMAND 3

#define NO_DESTINATIONADDRESS 0
#define DESTINATION_SHORT 2
#define DESTINATION_EXTENDED 3

#define NO_SOURCEADDRESS 0
#define SOURCE_SHORT 2
#define SOURCE_EXTENDED 3

#define RANGE_MESSAGE 0x00
#define TXDLY_ENABLED 1
#define TXDLY_DISABLED 0
//Definition of some configuration parameters

//Data rate
#define DWT_BR_110K 0b00000000
#define DWT_BR_6M8 0b01000000
#define DWT_BR_850K 0b00100000

//Transmit pulse repetition frequency
#define DWT_PRF_16M 0b00000001
#define DWT_PRF_64M 0b00000010

//Preamble length selection
#define DWT_PLEN_4096 0b00001100
#define DWT_PLEN_1024 0b00001000
#define DWT_PLEN_128 0b00010100
#define DWT_PLEN_64 0b00000100



struct msgParameters{
    byte length;
    uint16_t sourceAddress;
    uint16_t destinationAddress;
};

struct rxtxParameters{
    byte length;
    uint64_t timeRxTx;
    bool rxtxOk;
};

typedef struct{
    uint8_t dataRate; //110kbps, 850kbps or 6.8Mbps
    uint8_t Prf; //Pulse repetition frequency - 16MHZ or 64 MHZ
    uint8_t txPreambleLength;
    bool nsSFD;
    uint8_t txCode;
    uint8_t rxCode;
    uint8_t rxPAC; //Preamble acquisition chunk size; 8 or 64
    uint8_t phrMode;
}dwt_config_t;

void configureDwm1000(dwt_config_t configuration_parameters);

void writeDwm1000(uint8_t dir, uint16_t subDir, uint8_t data);
byte readDwm1000(uint8_t dir, uint16_t subDir);

void loadLDE();

void getDevId();
void getEUI(); //Return the EUI value
uint16_t getAddress(byte type); //Return PAN Adress or Short Adress, depending on the argument choosed
void setAddress(byte type, uint16_t address);

void sendData(byte buffer[], int length);
rxtxParameters sendData(byte buffer[], byte length, uint16_t destinationAddress, uint16_t sourceAddress);
rxtxParameters sendData(byte buffer[], byte length, uint16_t destinationAddress, uint16_t sourceAddress, byte txDelay);
rxtxParameters receiveData(byte buffer[]); //Receive data function. It returns the length of the received buffer.

void measureICTemperature();
void measureICVoltage();

uint16_t buildFrameControl(byte frameType, byte destinationMode, byte sourceMode);
uint64_t getActualTime();
//void getActualTime(byte time[]);
#endif // DWM1000_H_INCLUDED
