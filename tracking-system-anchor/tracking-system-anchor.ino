///ARDUINO MEGA
#include <Arduino.h>
#include <SPI.h>
#include <C:\Users\Christian\Desktop\tracking-system-anchor&tags\dwm1000.h>


   //Declaracion de variables

byte buffer[127];


void setup()
{
    Serial.begin(9600);

	pinMode(50, INPUT);
    pinMode(51, OUTPUT);
    pinMode(52, OUTPUT);
    pinMode(53, OUTPUT);
    digitalWrite(SS,HIGH);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setBitOrder(MSBFIRST);
    SPI.begin();

 //Configuration

 //Modo 4
//    dwt_config_t configuration;
//    configuration.dataRate=DWT_BR_6M8; //Data rate=6.8Mbps
//    configuration.Prf=DWT_PRF_16M; //Pulse repetition frequency=16Mhz
//    configuration.txPreambleLength=DWT_PLEN_128; //Preamble Length=128
//    configuration.nsSFD=false; //Use non-standard SFD
//    configuration.rxCode=3; //Tabla pagina 204
//    configuration.txCode=3;
//    configuration.rxPAC=32;  //Tabla pagina 32
//    configureDwm1000(configuration);

//Modo 3
    dwt_config_t configuration;
    configuration.dataRate=DWT_BR_110K;
    configuration.Prf=DWT_PRF_16M;
    configuration.txPreambleLength=DWT_PLEN_1024;
    configuration.nsSFD=true;
    configuration.rxCode=3; //Tabla pagina 204
    configuration.txCode=3;
    configuration.rxPAC=64;
    configureDwm1000(configuration);

    loadLDE();

    //Configuro la direccion del Dwm1000
    setAddress(SHORT_ADDR,0x000A);
    setAddress(PAN_ID,0xDECA);

    //Imprimo algunos parametros
    //getDevId();

    //Serial.print("Device short address: ");
    //Serial.println(getAddress(SHORT_ADDR),HEX);

    //Apago recepcion y transmision
    writeDwm1000(SYS_CTRL,0x00,0b01000000);

    //Frame filtering enabled for data reception
    writeDwm1000(SYS_CFG,0x00,0b00001001);

}

void loop()
{
    int i;
    rxtxParameters parametrosRx, parametrosTx;
    uint64_t TRP, TSR, TSP, TRR, TRF, TSF;
    uint32_t diferencia, TOF_without_unit;
    float TOF;
    float distance;


    parametrosRx=receiveData(buffer); //OBTENGO TRP
    parametrosTx=sendData(buffer,RANGE_MESSAGE,0x0B, 0x0A,TXDLY_DISABLED); //OBTENGO TSR
    TRP=parametrosRx.timeRxTx;
    TSR=parametrosTx.timeRxTx;

    //recibir TSP y TRR
    parametrosRx=receiveData(buffer);
    TRF=parametrosRx.timeRxTx;

    TSP=0;
    TRR=0;
    TSF=0;
    for (i=0; i<8; i++)
    {
        TSP=(TSP<<8)+buffer[7-i+11];
    }
    for (i=0; i<8; i++)
    {
        TRR=(TRR<<8)+buffer[15-i+11];  //11 es offset por el sistema de MAC
    }
    for (i=0; i<8; i++)
    {
        TSF=(TSF<<8)+buffer[23-i+11];  //11 es offset por el sistema de MAC
    }

    diferencia=(2*TRR-TSP-2*TSR+TRP+TRF-TSF)/2;
    TOF_without_unit=diferencia;
    TOF=diferencia*TIME_RES;

//    Serial.print("Tiempo ida y vuelta [us]=");
//    Serial.println(TOF,5);


//    ///MATLAB:
    Serial.println(TOF_without_unit);

}
