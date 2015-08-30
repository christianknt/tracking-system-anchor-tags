#include <Arduino.h>
#include <SPI.h>
#include <C:\Users\Christian\Desktop\tracking-system-anchor&tags\dwm1000.h>

void configureDwm1000(dwt_config_t configuration_parameters)
{
    uint8_t auxiliar, auxiliar2;
    int i;
//Configuration of some parameters according to user manual (optimal values)

    //Configuration of AGC_TUNE2
    writeDwm1000(AGC_CTRL, AGC_TUNE2, 0x07);
    writeDwm1000(AGC_CTRL, AGC_TUNE2+1, 0xA9);
    writeDwm1000(AGC_CTRL, AGC_TUNE2+2, 0x02);
    writeDwm1000(AGC_CTRL, AGC_TUNE2+3, 0x25);

    //Configuration of NTM
    if (configuration_parameters.Prf==DWT_PRF_16M)
    {
        writeDwm1000(LDE_IF, LDE_CFG2, 0x07);
        writeDwm1000(LDE_IF, LDE_CFG2+1, 0x16);
    }
    else
    {
        if (configuration_parameters.Prf==DWT_PRF_64M)
        {
        writeDwm1000(LDE_IF, LDE_CFG2, 0x07);
        writeDwm1000(LDE_IF, LDE_CFG2+1, 0x06);
        }
    }

    //Configuration of RF_TXCTRL
    writeDwm1000(RF_CONF, RF_TXCTRL, 0xE0);
    writeDwm1000(RF_CONF, RF_TXCTRL+1, 0x3F);
    writeDwm1000(RF_CONF, RF_TXCTRL+2, 0x1E);
    writeDwm1000(RF_CONF, RF_TXCTRL+3, 0x00);

    //Configuration of TC_PGDELAY
    writeDwm1000(TX_CAL, TC_PGDELAY, 0xC0); //0xC0 for channel 5

    //Configuracion de FS_PLLTUNE
    writeDwm1000(FS_CTRL, FS_PLLTUNE, 0xBE);
    /// /// /// /// /// /// /// /// /// ////
    ///Configuration of others parameters //
    /// /// /// /// /// /// /// /// /// ////

///configuration of Preamble length & Prf
    auxiliar=0;
    auxiliar=configuration_parameters.txPreambleLength+configuration_parameters.Prf;
    writeDwm1000(TX_FCTRL,2,auxiliar); //Configuration of PRF & Preamble extension

///configuration of data rate
    writeDwm1000(TX_FCTRL,1,configuration_parameters.dataRate); //Configuration of data rate

    if (configuration_parameters.dataRate==DWT_BR_110K)
    {                              ///sp
        writeDwm1000(SYS_CFG,2,0b01000100); //receiver mode 110kbps enabled, smart power disabled
    }
    else
    {                              ///sp
        writeDwm1000(SYS_CFG,2,0b00000100); //receiver mode 110kbps disabled, smart power disabled
    }

///configuration of TX Power, AGC_TUNE1 & DRX_TUNE1a
    if (configuration_parameters.Prf==DWT_PRF_16M) //PRF=16MHz
    {
        //Configuration of TX_POWER
        writeDwm1000(TX_POWER,0,0x48); //MODO 1
        writeDwm1000(TX_POWER,1,0x28);
        writeDwm1000(TX_POWER,2,0x08);
        writeDwm1000(TX_POWER,3,0x0E);

        //Configuration of AGC_TUNE1 for PRF=16M
        writeDwm1000(AGC_CTRL,AGC_TUNE1+0,0x70);
        writeDwm1000(AGC_CTRL,AGC_TUNE1+1,0x88);
        //Configuration of DRX_TUNE1a
        writeDwm1000(DRX_CONF,DRX_TUNE1a,0x87);
        writeDwm1000(DRX_CONF,DRX_TUNE1a+1,0x00);
    }
    else //PRF=64MHz
    {
        for (i=0; i<4; i++)
        {
            writeDwm1000(TX_POWER,0x00+i,0x85);
        }
        //Configuration of AGC_TUNE1 for PRF=64M
        writeDwm1000(AGC_CTRL,AGC_TUNE1+0,0x9B);
        writeDwm1000(AGC_CTRL,AGC_TUNE1+1,0x88);
        //Configuration of DRX_TUNE1a
        writeDwm1000(DRX_CONF,DRX_TUNE1a,0x8D);
    }

    auxiliar=0;
    if (configuration_parameters.nsSFD==true)
    {
        auxiliar=0b00110010; //non-standard propietary SFR sequence
        //configuration of DRX_TUNE0b
        if (configuration_parameters.dataRate==DWT_BR_6M8)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE0b,0x02);
        }
        else
        {
            if (configuration_parameters.dataRate==DWT_BR_850K)
            {
                writeDwm1000(DRX_CONF,DRX_TUNE0b,0x06);
            }
            else
            {
                if (configuration_parameters.dataRate==DWT_BR_110K)
                {
                    writeDwm1000(DRX_CONF,DRX_TUNE0b,0x16);
                }
            }
        }
    }
    else //SFD standard
    {
        auxiliar=0b00000000; //standard SFR sequence
        //configuration of DRX_TUNE0b
        if (configuration_parameters.dataRate==DWT_BR_6M8)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE0b,0x01);
        }
        else
        {
            if (configuration_parameters.dataRate==DWT_BR_850K)
            {
                writeDwm1000(DRX_CONF,DRX_TUNE0b,0x01);
            }
            else
            {
                if (configuration_parameters.dataRate==DWT_BR_110K)
                {
                    writeDwm1000(DRX_CONF,DRX_TUNE0b,0x0A);
                }
            }
        }
    }
    ///Configuration of RXPRF & TX_PCODE
    auxiliar=auxiliar+(configuration_parameters.Prf<<2);
    auxiliar2=(configuration_parameters.txCode)&0b00000011;
    auxiliar2=auxiliar2<<6;
    auxiliar=auxiliar+auxiliar2;

    writeDwm1000(CHAN_CTRL,2,auxiliar);

    auxiliar=(configuration_parameters.rxCode)&0b00011111;
    auxiliar=auxiliar<<3;
    auxiliar2=(configuration_parameters.txCode)&0b00011100;
    auxiliar2=auxiliar2>>2;
    auxiliar=auxiliar+auxiliar2;
    writeDwm1000(CHAN_CTRL,3,auxiliar);


///Configuration of DRX_TUNE1b
{
    if (configuration_parameters.txPreambleLength==64)
    {
        writeDwm1000(DRX_CONF,DRX_TUNE1b,0x64);
        writeDwm1000(DRX_CONF,DRX_TUNE1b+1,0x00);
    }

    if ((configuration_parameters.txPreambleLength>=128)&&(configuration_parameters.txPreambleLength<=1024))
    {
        writeDwm1000(DRX_CONF,DRX_TUNE1b,0x20);
        writeDwm1000(DRX_CONF,DRX_TUNE1b+1,0x00);
    }

    if (configuration_parameters.txPreambleLength>1024)
    {
        writeDwm1000(DRX_CONF,DRX_TUNE1b,0x10);
        writeDwm1000(DRX_CONF,DRX_TUNE1b+1,0x00);
    }
}

///Configuration of DRX_TUNE2
{
    if (configuration_parameters.rxPAC==8)
    {
        writeDwm1000(DRX_CONF,DRX_TUNE2+3,0x31); //Pag. 137
        if (configuration_parameters.Prf==DWT_PRF_16M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0x2D);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x00);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x1A);
        }
        if (configuration_parameters.Prf==DWT_PRF_64M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0x6B);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x00);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x3B);
        }
    }

    if (configuration_parameters.rxPAC==16)
    {
        writeDwm1000(DRX_CONF,DRX_TUNE2+3,0x33);
        if (configuration_parameters.Prf==DWT_PRF_16M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0x52);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x00);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x1A);
        }
        if (configuration_parameters.Prf==DWT_PRF_64M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0xBE);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x00);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x3B);
        }
    }

    if (configuration_parameters.rxPAC==32)
    {
        writeDwm1000(DRX_CONF,DRX_TUNE2+3,0x35);
        if (configuration_parameters.Prf==DWT_PRF_16M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0x9A);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x00);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x1A);
        }
        if (configuration_parameters.Prf==DWT_PRF_64M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0x5E);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x01);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x3B);
        }
    }

    if (configuration_parameters.rxPAC==64)
    {
        writeDwm1000(DRX_CONF,DRX_TUNE2+3,0x37);
        if (configuration_parameters.Prf==DWT_PRF_16M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0x1D);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x01);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x1A);
        }
        if (configuration_parameters.Prf==DWT_PRF_64M)
        {
            writeDwm1000(DRX_CONF,DRX_TUNE2,0x96);
            writeDwm1000(DRX_CONF,DRX_TUNE2+1,0x02);
            writeDwm1000(DRX_CONF,DRX_TUNE2+2,0x3B);
        }
    }
}

///Configuration of DRX_TUNE4H
{
if (configuration_parameters.txPreambleLength==DWT_PLEN_64)
{
    writeDwm1000(DRX_CONF,DRX_TUNE4H,0x10);
    writeDwm1000(DRX_CONF,DRX_TUNE4H+1,0x00);
}
if ((configuration_parameters.txPreambleLength==DWT_PLEN_1024)||(configuration_parameters.txPreambleLength==DWT_PLEN_128))
    writeDwm1000(DRX_CONF,DRX_TUNE4H,0x28);
    writeDwm1000(DRX_CONF,DRX_TUNE4H+1,0x00);
}

}

void getDevId()
{
    uint16_t RIDTAG=0;
    uint8_t MODEL=0, VERSION=0;
    uint16_t aux=0;

//Lectura del DEV_ID
    aux=readDwm1000(DEV_ID,0x03);
    aux=aux<<8;
    RIDTAG=aux+readDwm1000(DEV_ID,0x02);
    MODEL=readDwm1000(DEV_ID,1); //Leo el model del dispositivo
    VERSION=readDwm1000(DEV_ID,0); //Leo la version del dispositivo

    //Imprimo los datos en pantalla por puerto serie.
    Serial.print("Device: ");
    Serial.print(RIDTAG,HEX);
    Serial.print(" Ver. ");
    Serial.print(MODEL,HEX);
    Serial.print(".");
    Serial.println(VERSION,HEX);

}

void getEUI()
{
    int i=0;
    uint64_t EUI_leido=0, aux2=0;
    int aux=0;

    Serial.print("EUI: ");
    for (i=0; i<8; i++)
    {
        aux=readDwm1000(EUI,i);
        aux2=aux<<(i*8);
        EUI_leido=EUI_leido+aux2;
        Serial.print(aux);
        Serial.print(".");
    }
    Serial.println("\n");

}


uint16_t getAddress(byte type)
{
    uint16_t address=0;
    uint16_t auxiliar=0; //Auxiliar variable

    address=readDwm1000(PANADR,type); //Read first 8 bits of address 'type'
    auxiliar=readDwm1000(PANADR,type+1);//Read last 8 bits of address 'type'
    address=address+(auxiliar<<8);
    return address;
}

void setAddress(byte type, uint16_t address)
{
    uint16_t auxiliar=0; //Auxiliar variable
    auxiliar=0x00FF&address; //Write first 8 bits of address on the 'type' register
    writeDwm1000(PANADR,type,auxiliar);
    auxiliar=(0xFF00&address)>>8; //Write last 8 bits of address on the 'type' register
    writeDwm1000(PANADR,type+1,auxiliar);
}


void writeDwm1000(uint8_t dir, uint16_t subDir, uint8_t data)
{
    int byte1=0, byte2=0, byte3=0;
    uint16_t aux;

    if ((subDir>=0)&&(subDir<=0x7F))
    {
        byte1=0b11000000+dir; //Indica escritura y que hay segundo octeto

        byte2=0b000000001111111&subDir; //Indica que no hay tercer octeto

        digitalWrite(SS,LOW);
        SPI.transfer(byte1);
        SPI.transfer(byte2);
        SPI.transfer(data);
        digitalWrite(SS,HIGH);
    }
    else
    if ((subDir>0x7F)&&(subDir<=0x7FFF))
    {
        byte1=0b11000000+dir; //Indica escritura y que hay segundo octeto

        byte2=0b0000000001111111&subDir;
        byte2=byte2+0b10000000; //Indica que hay tercer octeto

        aux=0b0111111110000000&subDir;
        aux=aux>>7;
        byte3=aux;

        digitalWrite(SS,LOW);
        SPI.transfer(byte1);
        SPI.transfer(byte2);
        SPI.transfer(byte3);
        SPI.transfer(data);
        digitalWrite(SS,HIGH);
    }
}


byte readDwm1000(uint8_t dir, uint16_t subDir)
{
    int byte1=0, byte2=0, byte3=0;
    uint16_t aux;
    byte msg=0;


    if ((subDir>=0)&&(subDir<=0x7F))
    {
        byte1=0b01000000+dir;
        byte2=0b000000001111111&subDir;

        digitalWrite(SS,LOW);
        SPI.transfer(byte1);
        SPI.transfer(byte2);
        msg=SPI.transfer(0x00);
        digitalWrite(SS,HIGH);
    }
    else
    if ((subDir>0x7F)&&(subDir<=0x7FFF))
    {
        byte1=0b01000000+dir;
        byte2=0b000000001111111&subDir;
        byte2=byte2+0b10000000; //Indica que hay proximo octeto
        aux=0b0111111110000000&subDir;
        aux=aux>>7;
        byte3=aux;


        digitalWrite(SS,LOW);
        SPI.transfer(byte1);
        SPI.transfer(byte2);
        SPI.transfer(byte3);
        msg=SPI.transfer(0x00);
        digitalWrite(SS,HIGH);
    }

    return msg;
}


void sendData(byte buffer[], int length)
{
    bool transmitido=false;
    byte auxiliar, auxiliar2;
    byte control;
    int i;

// Apago recepcion y transmision
    writeDwm1000(SYS_CTRL,0x00,0b01000000);
    /////////////////////////////////////

//    Dato a enviar
    for (i=0; i<length; i++)
    {
        writeDwm1000(TX_BUFFER,i,buffer[i]); //Copio los datos a enviar en el buffer de salida.
    }
//    Largo del dato a enviar
    writeDwm1000(TX_FCTRL,0x00,length+2);

//    Inicio transmision
    writeDwm1000(SYS_CTRL,0x00,0b00000010);

    transmitido=false;
    while (transmitido==false)
    {
        auxiliar=readDwm1000(SYS_STATUS,0x00); //Leo el bit 7 (TXFRS)
        if ((auxiliar&0b10000000)==128)
        {
            writeDwm1000(SYS_STATUS,0x00,0b11111110); //Limpio las variables informativas
            transmitido=true;
            break;
        }
        else
        {
            auxiliar=readDwm1000(SYS_STATUS,0x03);
            if ((auxiliar&0b00010000)==16)
            {
                Serial.println("Error: Transmit Buffer Error");
                transmitido=false;
                break;
            }
        }
    }

   // Apago recepcion y transmision
    writeDwm1000(SYS_CTRL,0x00,0b01000000);
    /////////////////////////////////////


}


rxtxParameters receiveData(byte buffer[])
{
    byte control;
    bool leido=false;
    bool error=false;
    byte auxiliar, auxiliar2;
    int i=0;
    byte length;

    uint64_t rxTime=0;
    rxtxParameters parametros;

// Apago recepcion y transmision
    writeDwm1000(SYS_CTRL,0x00,0b01000000);
    /////////////////////////////////////

    //Habilito la recepcion
  //  control=readDwm1000(SYS_CTRL,0x01);
  //  control=(control&0b11111110)+0x01;
    writeDwm1000(SYS_CTRL,0x01,0b00000001);

    //Espero a recibir algun dato
    leido=false;
    while (leido==false)
    {
        auxiliar=readDwm1000(SYS_STATUS,0x01);
        if ((auxiliar&0b00100000)==32) //Leo el bit 13 (RXDFR)
        {
            //Serial.println("Data received: OK");
            leido=true; //Si recibo dato, salgo del while
            break;
        }
        else
        {
            if ((auxiliar&0b00010000)==16) //RXPHE
            {
                Serial.println("Error: Receiver PHY header Error");
                leido=false;
                break; //Si recibo dato incorrecto, salgo del while
            }
            else
            {
                auxiliar2=readDwm1000(SYS_STATUS,0x02);
                if ((auxiliar2&0b00010000)==16) //RXOVRR Receiver Overrun
                {
                    Serial.println("Error: Receiver Overrun");
                    leido=false;
                    break; //Si recibo dato incorrecto, salgo del while
                }
                else
                {
                    auxiliar2=readDwm1000(SYS_STATUS,2);
                    if ((auxiliar2&0b00100000)==32)
                    {
                        Serial.println("Preamble detection timeout");
                        leido=false;
                        break; //Si recibo dato incorrecto, salgo del while
                    }
                }
            }
        }
    }

   // Apago recepcion y transmision
    writeDwm1000(SYS_CTRL,0x00,0b01000000);
    /////////////////////////////////////

    //Leo extension del dato recibido
    length=readDwm1000(RX_FINFO,0x00);
    length=length&0b01111111;

    if (length!=RANGE_MESSAGE)
    {
         for (i=0; i<length; i++)
        {
            buffer[i]=readDwm1000(RX_BUFFER,i); //Leo el byte i, del buffer de recepcion
        }
    }


//Devuelvo el tiempo de llegada
    rxTime=0;
    for (i=0; i<5; i++)
    {
        auxiliar=readDwm1000(RX_TIME,4-i);
        rxTime=(rxTime<<8);
        rxTime=rxTime+auxiliar;
    }

    parametros.timeRxTx=rxTime;
    parametros.length=length;
    return parametros;
}

void loadLDE()
{
     //Loading information from ROM to RAM memory
    writeDwm1000(PMSC,0x00,0x01);
    writeDwm1000(OTP_IF,0x07,0b10000000); //Information is loaded from OTP memory
    delayMicroseconds(200);
    writeDwm1000(PMSC,0x00,0x00);
}

uint16_t buildFrameControl(byte frameType, byte destinationMode, byte sourceMode)
{
    uint16_t FRAMECONTROL=0, auxiliar=0;
    //Bit 0,1 and 2 indicates frame type, Frame type is configured according to data type
    FRAMECONTROL=FRAMECONTROL+frameType;

    //Bit 3 (security enable) remains in zero value indicating there is no security process

    //Bit 4 indicates the sending device has more data for the recipient

    //Bit 5 -- ACK request

    //Bit 6 - PAN ID compress

    //Bit 10 & 11 - Destination Address Mode
    auxiliar=destinationMode;
    auxiliar=auxiliar<<10;
    FRAMECONTROL=FRAMECONTROL+auxiliar;

    //Bit 12 & 13 - Frame Version - 0 due to DW1000 version

    //Bit 14 & 15 - Source address mode
    auxiliar=sourceMode;
    auxiliar=auxiliar<<14;
    FRAMECONTROL=FRAMECONTROL+auxiliar;

    return FRAMECONTROL;
}

rxtxParameters sendData(byte buffer[], byte length, uint16_t destinationAddress, uint16_t sourceAddress)
{
    uint16_t frameControl=0;
    uint8_t sequenceNumber=0;
    byte bufferAux[127];
    rxtxParameters parametros;
    int i;
    byte auxiliar;

    frameControl=buildFrameControl(DATA, DESTINATION_SHORT, SOURCE_SHORT);
    bufferAux[0]=frameControl&0x00FF;
    bufferAux[1]=(frameControl&0xFF00)>>8;
    bufferAux[2]=sequenceNumber;

    //Destination PAN identifier
    bufferAux[3]=0xCA; //Low bits of PAN_ID
    bufferAux[4]=0xDE; //High bits of PAN_ID

    //Destination Short Address
    bufferAux[5]=0x00FF&destinationAddress; //Low bits of destination short address
    bufferAux[6]=(0xFF00&sourceAddress)>>8; //High bits of destination short address

    //Source PAN identifier
    bufferAux[7]=0xCA; //Low bits of PAN_ID
    bufferAux[8]=0xDE; //High bits of PAN_ID

    //Source Short Address
    bufferAux[9]=0x00FF&sourceAddress; //Low bits of source short address
    bufferAux[10]=(0xFF00&sourceAddress)>>8; //High bits of source short address

    if (length==RANGE_MESSAGE) //If the message is not ranging message..
    {
        sendData(bufferAux,11);
    }
    else
    {
        //datos
        for (int i=11; i<length+11; i++)
        {
            bufferAux[i]=buffer[i-11];
        }

//        for (int i=0; i<length+11; i++)
//        {
//            buffer[i]=bufferAux[i];
//        }
        sendData(bufferAux,length+11);
    }
    parametros.timeRxTx=0;
    for (i=0; i<5; i++)
    {
        auxiliar=readDwm1000(TX_TIME,4-i);
        parametros.timeRxTx=(parametros.timeRxTx<<8);
        parametros.timeRxTx=parametros.timeRxTx+auxiliar;
    }
    return parametros;
}


rxtxParameters sendData(byte buffer[], byte length, uint16_t destinationAddress, uint16_t sourceAddress, byte txDelay)
{
    uint16_t frameControl=0;
    uint8_t sequenceNumber=0;
    byte bufferAux[127];
    rxtxParameters parametros;
    int i;
    byte auxiliar;
    bool transmitido;

    frameControl=buildFrameControl(DATA, DESTINATION_SHORT, SOURCE_SHORT);
    bufferAux[0]=frameControl&0x00FF;
    bufferAux[1]=(frameControl&0xFF00)>>8;
    bufferAux[2]=sequenceNumber;

    //Destination PAN identifier
    bufferAux[3]=0xCA; //Low bits of PAN_ID
    bufferAux[4]=0xDE; //High bits of PAN_ID

    //Destination Short Address
    bufferAux[5]=0x00FF&destinationAddress; //Low bits of destination short address
    bufferAux[6]=(0xFF00&sourceAddress)>>8; //High bits of destination short address

    //Source PAN identifier
    bufferAux[7]=0xCA; //Low bits of PAN_ID
    bufferAux[8]=0xDE; //High bits of PAN_ID

    //Source Short Address
    bufferAux[9]=0x00FF&sourceAddress; //Low bits of source short address
    bufferAux[10]=(0xFF00&sourceAddress)>>8; //High bits of source short address



//    Dato a enviar
    for (i=0; i<10; i++)
    {
        writeDwm1000(TX_BUFFER,i,bufferAux[i]); //Copio los datos a enviar en el buffer de salida.
    }

    length=length+11;
    if (length!=11) //If it is not RANGE MESSAGE
    {
        for (i=11; i<(length+11); i++)
        {
            writeDwm1000(TX_BUFFER,i,buffer[i-11]); //Copio los datos a enviar en el buffer de salida.
        }
    }

//    Largo del dato a enviar
    writeDwm1000(TX_FCTRL,0x00,length+2);

    if (txDelay==TXDLY_DISABLED)
    {
        //    Inicio transmision
        writeDwm1000(SYS_CTRL,0x00,0b00000010);
    }

    transmitido=false;
    while (transmitido==false)
    {
        auxiliar=readDwm1000(SYS_STATUS,0x00); //Leo el bit 7 (TXFRS)
        if ((auxiliar&0b10000000)==128)
        {
            writeDwm1000(SYS_STATUS,0x00,0b11111110); //Limpio las variables informativas
            transmitido=true;
        }
        else
        {
            auxiliar=readDwm1000(SYS_STATUS,0x03);
            if ((auxiliar&0b00010000)==16)
            {
                Serial.println("Error: Transmit Buffer Error");
                writeDwm1000(SYS_STATUS,0x00,0b11111110); //Limpio las variables informativas
                transmitido=false;
                break;
            }
        }
    }

    // Apago recepcion y transmision
    writeDwm1000(SYS_CTRL,0x00,0b01000000);
    /////////////////////////////////////

    parametros.timeRxTx=0;
    for (i=0; i<5; i++)
    {
        auxiliar=readDwm1000(TX_TIME,4-i);
        parametros.timeRxTx=(parametros.timeRxTx<<8);
        parametros.timeRxTx=parametros.timeRxTx+auxiliar;
    }
    return parametros;
}


void measureICTemperature()
{

}
void measureICVoltage()
{

}

uint64_t getActualTime()
{
    uint64_t time=0;
    int i;
    for (i=0; i<5; i++)
    {
        time=time<<8;
        time=time+readDwm1000(SYS_TIME,4-i);
    }
    return time;
}
