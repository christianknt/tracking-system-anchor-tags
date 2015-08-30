///ARDUINO UNO
#include <Arduino.h>
#include <SPI.h>
#include <C:\Users\Christian\Desktop\tracking-system-anchor&tags\dwm1000.h>
#include <math.h>

byte buffer[127];

bool loopinfinito=true;
bool ledState=false;

void setup()
{
    Serial.begin(9600);

    pinMode(8, OUTPUT);
	pinMode(MISO, INPUT);
    pinMode(SS, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    digitalWrite(SS,HIGH);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setBitOrder(MSBFIRST);
    SPI.begin();


    //Cargo desde la memoria OTP la informacion a la RAM
    loadLDE();

    getDevId();

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

    //Configuro el Dwm1000
    setAddress(SHORT_ADDR,0x000B);
    setAddress(PAN_ID,0xDECA);

    Serial.print("Device short address: ");
    Serial.println(getAddress(SHORT_ADDR),HEX);

    //Apago recepcion y transmision
    writeDwm1000(SYS_CTRL,0x00,0b01000000);

    //Frame filtering enabled for data reception
    writeDwm1000(SYS_CFG,0x00,0b00001001);


//Configuracion del timer
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0; //Actual time

    OCR1A = 15625;            // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 1024 prescaler
    TCCR1B |= (1 << CS10);    // 1024 prescaler
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();             // enable all interrupts
}


void loop()
{
    int i=0;
    rxtxParameters parametrosTx, parametrosRx;
    uint64_t TSP=0, TRR=0, TSF=0, aux;
    byte delayBytes[5];
    uint64_t actualTime=0, sendTime=0;

    byte auxiliar;


    if (loopinfinito)
    {
        delay(1000);
        loopinfinito=false;
    }

    parametrosTx=sendData(buffer,RANGE_MESSAGE,0x000A,0x000B);
    parametrosRx=receiveData(buffer);
    //delayed transmission enabled
    actualTime=getActualTime();
    sendTime=actualTime+202307692;
                    ///202307692
    TSP=parametrosTx.timeRxTx;
    TRR=parametrosRx.timeRxTx;
    TSF=sendTime;

    for (i=0; i<5; i++)
    {
        auxiliar=sendTime&0xFF;
        writeDwm1000(DX_TIME, i, auxiliar);
        sendTime=sendTime>>8;
    }

    //TSP A ENVIAR
    aux=TSP;
    for (i=0; i<8; i++)
    {
        buffer[i]=0x000000FF&aux;
        aux=aux>>8;
    }
    //TRR A ENVIAR
    aux=TRR;
     for (i=8; i<16; i++)
    {
        buffer[i]=0x000000FF&aux;
        aux=aux>>8;
    }
    //TSF A ENVIAR
    aux=TSF;
    for (i=16; i<24; i++)
    {
        buffer[i]=0x000000FF&aux;
        aux=aux>>8;
    }
    //    Inicio transmision dentro del delay programado
    writeDwm1000(SYS_CTRL,0x00,0b00000110);

    //Envio TSP , TRR y TSF al otro módulo
    parametrosTx=sendData(buffer,24,0x000A,0x000B,TXDLY_ENABLED);
    TSF=parametrosTx.timeRxTx;

    uint32_t diferencia=TSF-TRR;
   // Serial.print("diferencia=");
    Serial.println(diferencia*TIME_RES);

    delay(50);
    //delay(500);

}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
    if (ledState==false)
    {
        digitalWrite(8,1);
        ledState=true;
    }
    else
    {
        digitalWrite(8,0);
        ledState=false;
    }
}
