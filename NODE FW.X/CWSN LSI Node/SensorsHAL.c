/*******************************************************************************
 * File:   SensorsHAL.c
 * Author: Enrique Cobo Jiménez - Laboratorio de Sistemas Integrados (LSI) - UPM
 *
 * File Description: Sensors and Actuators Hardware Abstraction Layer.
 * Implements an API for application and cognitive layers.
 * It's the top level of the LSI-CWSN Microchip MiWi Stack.
 *
 * Change History:
 * Rev   Date         Description
 ******************************************************************************/
/* INCLUDES *******************************************************************/
#include "Include/SensorsHAL.h"

/* END INCLUDES ***************************************************************/

/* DEFINITIONS ****************************************************************/

// Switch on ADC if included LUM
#if defined LUM
    #define PARAM1 (ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON)
    #define PARAM2 (ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON)
    #define PARAM3 (ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_12)
    #define PARAM5 (SKIP_SCAN_ALL)
    #define PARAM4 (ENABLE_AN13_ANA)
    #define PARAM6 (ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN13)
#endif

#if defined IR
    #define NBUFFER 4

    #define TCOMUN (0x11, 0xAD, 0x78, 0x81)
    #define T1a (0x04)
    #define T0 (0x00)
    #define T1b (0x1E)
    #define T2a (0x73)
    #define OnCool (0x21)
    #define OffCool (0x20)
    #define OnHeat (0x11)
    #define OffHeat (0x10)
    #define TempCool (0x1C)
    #define TempHeat (0x22)
    #define T2b (0x35)
    #define T2c (0x20)
    #define CRCOnCool (0x1F)
    #define CRCOffCool (0x1E)
    #define CRCOnHeat (0x15)
    #define CRCOffHeat (0x14)
#endif

/* END DEFINITIONS ************************************************************/

UINT8 getAccRegister (UINT8 reg);
void setAccRegister (UINT8 reg, UINT8 dato);
void protocoloAA();

/* VARIABLES ******************************************************************/

#if defined TEMP
    int isTempLowPower;
    int cntTemp;
    INT8 tempAlertMin;
    INT8 tempAlertMax;
#endif

#if defined ACC
    int accX, accY, accZ;
    UINT8 sourceINT1, sourceINT2;
#endif

#if defined PIR
    int cntPIR;
#endif

#if defined IR
    UINT8 IRParam1 [NBUFFER];
    UINT8 IRParam2 [NBUFFER];
    void (*IRProtocol[NBUFFER]) (void);
    UINT8 AATrama1 [7] = {0x11, 0xDA, 0x17, 0x18, T1a, T0, T1b};
    UINT8 AATrama2 [15] = {0x11, 0xDA, 0x17, 0x18, T0, T2a, T0, OnCool, T0, T0, TempCool, T2b, T0, T2c, CRCOnCool} ;
    int AApos;
    int nocup;
    int puntero;
    int IRcontador;
    int IRestado;
    int IRindex;
#endif

#if defined BUZZ
    int isBuzzing;
    int cntBuzzer;
    int nota2;
    int buzzerPrescaler;
    char tempConf;
#endif

/* END VARIABLES **************************************************************/


////////////////////////////////////////////////////////////////////////////////
/*****************   HAL FUNCTIONS (FOR THE SENSORS BOARD)   ******************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Function: InitSensors()
 * Input:    None
 * Output:   NO_ERROR if success. HAL error code otherwise.
 * Overview: Sensor initialization function.
 ******************************************************************************/
#if defined SENSORS
BYTE InitSensors(){

    // En un principio no se habilitan las interrupciones
    // Hay que llamar a enableIntSensors() para habilitarlas
    // (al comienzo de la aplicación con el uso de los sensores)

    //TIMER5
    WORD T5_TICK = (CLOCK_FREQ/8/8/34000);
    OpenTimer5(T5_ON | T5_IDLE_CON | T5_GATE_OFF | T5_PS_1_8 | T5_SOURCE_INT, T5_TICK);
    ConfigIntTimer5(T5_INT_OFF | T5_INT_PRIOR_7);

    #if defined BUZZ
      AD1PCFGSET = 0x0800; // Al ser multiplexado con ADCs hay que forzar
      GPIO_BUZZ_TRIS = OUTPUT_PIN;
      GPIO_BUZZ = 0;
      isBuzzing = FALSE;
    #endif

    #if defined IR
        GPIO_IR_TRIS = OUTPUT_PIN;
        GPIO_IR = 0;
    #endif

    #if defined PIR
        mCNOpen(CN_ON | CN_IDLE_CON, CN15_ENABLE, CN_PULLUP_DISABLE_ALL);
        mPORTDRead(); //Vaciar
        ConfigIntCN(CHANGE_INT_OFF | CHANGE_INT_PRI_6);
        IFS1CLR = 0x00000001;
        IPC6SET = 0x00180000;
        GPIO_PRES_TRIS = INPUT_PIN;
    #endif

    // I2C
    #if defined ACC || defined TEMP
        #define TempAddress (0x48)
        #define AccAddress (0x1C)
        OpenI2C2 (I2C_EN, (10000));
    #endif

    #if defined TEMP
        tempConf = 0x00;
        #define tempReg (0x00)
        #define tempConfReg (0x01)
    #endif

    #if defined ACC
        int registro;
        registro = getAccRegister(0x2A);
        registro = registro | (0x03);
        setAccRegister(0x2A, registro);

        sourceINT1 = 0;
        sourceINT2 = 0;

        AD1PCFGSET = 0x8008; // Puerto multiplexado con ADCs, hay que forzar entrada digital
        GPIO_INT1_TRIS = INPUT_PIN;
        GPIO_INT2_TRIS = INPUT_PIN;
    #endif

    // LUM
    #if defined LUM
        CloseADC10();
        SetChanADC10(PARAM6);
        OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);
        EnableADC10();
    #endif

    // LEDs
    GREEN_LED_TRIS = OUTPUT_PIN;
    GREEN_LED = 0;
    RED_LED_TRIS = OUTPUT_PIN;
    RED_LED = 0;

    return NO_ERROR;
}
#endif

////////////////////////////////////////////////////////////////////////////////
/********************************** IR ****************************************/
////////////////////////////////////////////////////////////////////////////////
#if defined IR

/*******************************************************************************
 * Function:    sendIR(int address, int command)
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer on.
 ******************************************************************************/
void sendIR (INT8 device, INT8 param1, INT8 param2) {

    if (nocup == NBUFFER) return; // Si no hay hueco lo borramos

    switch (device) {
        case AA:
            IRProtocol[(puntero+nocup) % NBUFFER] = protocoloAA;
            break;
        // Implementación de nuevos protocolos
    }

    IRParam1[(puntero+nocup) % NBUFFER] = param1;
    IRParam2[(puntero+nocup) % NBUFFER] = param2;
    nocup++;

}

/*******************************************************************************
 * Function:    sendIR(int address, int command)
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer on.
 ******************************************************************************/
int mandarCero() {

    if (++IRcontador < 28) {
        GPIO_IR ^= 0x0001;
        return 0;
    }
    if (++IRcontador < 120) {
        GPIO_IR = 0;
        return 0;
    }
    IRcontador = 0;
    return 1;

}

/*******************************************************************************
 * Function:    sendIR(int address, int command)
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer on.
 ******************************************************************************/
int mandarUno() {

    if (++IRcontador < 28) {
        GPIO_IR ^= 0x0001;
        return 0;
    }
    if (++IRcontador < 260) {
        GPIO_IR = 0;
        return 0;
    }
    IRcontador = 0;
    return 1;

}

/*******************************************************************************
 * Function:    sendIR(int address, int command)
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer on.
 ******************************************************************************/
void protocoloAA () {

    // Se implementa por medio de una máquina de estados

    int nextEstado;
    int aMandar;
    int final;

    nextEstado = IRestado;
    switch(IRestado) {
        case 0: // Arranque
            GPIO_IR ^= 0x0001; // Ráfaga
            if (++IRcontador == 342) {
                nextEstado = 1;
                IRcontador = 0;
                // Decodificación de parámetros a mandar
                if (IRParam1[puntero] == AA_Summer) {
                    AATrama2[10] = TempCool;
                    if (IRParam2[puntero] == AA_On) {
                        AATrama2[7] = OnCool;
                        AATrama2[14] = CRCOnCool;
                    } else {
                        AATrama2[7] = OffCool;
                        AATrama2[14] = CRCOffCool;
                    }
                } else {
                    AATrama2[10] = TempHeat;
                    if (IRParam2[puntero] == AA_On) {
                        AATrama2[7] = OnHeat;
                        AATrama2[14] = CRCOnHeat;
                    } else {
                        AATrama2[10] = OffHeat;
                        AATrama2[14] = CRCOffHeat;
                    }
                }
            }
            break;
        case 1: // Espera
            GPIO_IR = 0; // Silencio
            if (++IRcontador == 148) {
                nextEstado = 2;
                IRcontador = 0;
            }
            break;
        case 2: //Primera trama (7 bytes fijos)
            aMandar = (AATrama1[AApos] & (1 << IRindex));
            if (aMandar) final = mandarUno();
            else final = mandarCero();
            if (final && ++IRindex == 8) {
                IRindex = 0;
                IRcontador = 0;
                if (++AApos == 7) {
                    nextEstado = 3;
                    AApos = 0;
                }
            }
            break;
        case 3: //Fin primera trama
            final =  mandarCero();
            if (final) {
                nextEstado = 4;
                }
            break;
        case 4: //Parada entre tramas
            GPIO_IR = 0; // Silencio
            if (++IRcontador == 2012) {
                nextEstado = 5;
                IRcontador = 0;
            }
            break;
        case 5: // Arranque
            GPIO_IR ^= 0x0001; // Ráfaga
            if (++IRcontador == 342) {
                nextEstado = 6;
                IRcontador = 0;
            }
            break;
        case 6: // Espera
            GPIO_IR = 0; // Silencio
            if (++IRcontador == 148) {
                nextEstado = 7;
                IRcontador = 0;
            }
            break;
        case 7: //Segunda trama (15 bytes)
            aMandar = (AATrama2[AApos] & (1 << IRindex));
            if (aMandar) final = mandarUno();
            else final = mandarCero();
            if (final && ++IRindex == 8) {
                IRindex = 0;
                IRcontador = 0;
                if (++AApos == 15) {
                    nextEstado = 8;
                    AApos = 0;
                }
            }
            break;
        case 8: //Fin primera trama
            final =  mandarCero();
           if (final) {
                nextEstado = 9;
                }
            break;
        case 9: //Idle espera
            GPIO_IR = 0; // Silencio
            if (++IRcontador == 2250) {
                nextEstado = 0;
                IRcontador = 0;
                puntero = (puntero+1) % NBUFFER; //Cambio de punteros
                nocup--;
            }
            break;
    }

    IRestado = nextEstado;

}

#endif
////////////////////////////////////////////////////////////////////////////////
/********************************** BUZZER ************************************/
////////////////////////////////////////////////////////////////////////////////

#if defined BUZZ
/*******************************************************************************
 * Function:    buzzerOn()
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer on.
 ******************************************************************************/
void buzzerOn() {

    isBuzzing++;
    return ;

}

/*******************************************************************************
 * Function:    buzzerOff()
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer off.
 ******************************************************************************/
void buzzerOff() {

    if (isBuzzing)
        isBuzzing--;
    return ;

}

#endif
////////////////////////////////////////////////////////////////////////////////
/****************************** ACCELEROMETER *********************************/
////////////////////////////////////////////////////////////////////////////////

#if defined ACC
/*******************************************************************************
 * Function:    getAcc()
 * Input:       None
 * Output:      None.
 * Overview:    Gets the acceleration by using MMA8453Q, and is saved.
 ******************************************************************************/
UINT8 getAccRegister (UINT8 reg) {

    UINT8 contenido;

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (AccAddress << 1) | 0; // Escritura
    i2cData[1] = reg; //  Registro Temp. Ambiente
    i2cData[2] = (AccAddress << 1) | 1; // Lectura

    // Comunicación
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // ACC address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    RestartI2C2();
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // ACC address y leer
    IdleI2C2();

    // Leer datos
    contenido = MasterReadI2C2();
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    return contenido;
}

/*******************************************************************************
 * Function:    getAcc()
 * Input:       None
 * Output:      None.
 * Overview:    Gets the acceleration by using MMA8453Q, and is saved.
 ******************************************************************************/
void setAccRegister (UINT8 reg, UINT8 dato) {

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (AccAddress << 1) | 0; // Escritura
    i2cData[1] = reg; //  Registro a escribir
    i2cData[2] = dato; // Escritura en el registro

    // Comunicación
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // ACC address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // Escribir en registro
    IdleI2C2();
    StopI2C2();
    IdleI2C2();
}

/*******************************************************************************
 * Function:    getAcc()
 * Input:       None
 * Output:      None.
 * Overview:    Gets the acceleration by using MMA8453Q, and is saved.
 ******************************************************************************/
void setAccLowPower() {

    int registro;

    registro = getAccRegister(0x2A);
    registro = registro | (0xC0);
    setAccRegister(0x2A, registro);

    return ;
}

void getAcc() {

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (AccAddress << 1) | 0; // Escritura
    i2cData[1] = 0x00; //  Registro Temp. Ambiente
    i2cData[2] = (AccAddress << 1) | 1; // Lectura

    // Comunicacin
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // TEMP address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    RestartI2C2();
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // TEMP address y leer
    IdleI2C2();


    UINT8 registro;

    // Leer datos
    registro = MasterReadI2C2();
    AckI2C2();
    IdleI2C2();
    accX = MasterReadI2C2();
    AckI2C2();
    IdleI2C2();
    accY = MasterReadI2C2();
    AckI2C2();
    IdleI2C2();
    accZ = MasterReadI2C2();
    IdleI2C2();
    StopI2C2();
    IdleI2C2();


     return ;
 }

/*******************************************************************************
 * Function:    getAcc()
 * Input:       None
 * Output:      None.
 * Overview:    Gets the acceleration by using MMA8453Q, and is saved.
 ******************************************************************************/
void setAccInt(int interrupcion, int source) {

    UINT8 dato, registro;

    dato = (1 << source); //BSET de la fuente correspondiente
    registro = getAccRegister(0x2E);

    if (interrupcion = ACC_INT1) {
        sourceINT1 = dato;
        dato = dato | sourceINT2; //No sobreescribir el de la otra interrupción
    } else {
        sourceINT2 = dato;
        dato = dato | sourceINT1; //No sobreescribir el de la otra interrupción
    }

    setAccRegister(0x2D, dato);
    setAccRegister(0x2E, sourceINT1);

    return ;
}


/*******************************************************************************
 * Function:    getAccX()
 * Input:       None
 * Output:      Acceleration in X axis.
 * Overview:    Once the acceleration has been saved, read it.
 ******************************************************************************/
int getAccX() {

    return accX ;
}


/*******************************************************************************
 * Function:    getAccY()
 * Input:       None
 * Output:      Acceleration in Y axis.
 * Overview:    Once the acceleration has been saved, read it.
 ******************************************************************************/
int getAccY() {

    return accY ;
}


/*******************************************************************************
 * Function:    getAccZ()
 * Input:       None
 * Output:      Acceleration in Z axis.
 * Overview:    Once the acceleration has been saved, read it.
 ******************************************************************************/
int getAccZ() {

    return accZ ;
}


#endif
////////////////////////////////////////////////////////////////////////////////
/******************************* TEMPERATURE **********************************/
////////////////////////////////////////////////////////////////////////////////

#if defined TEMP
/*******************************************************************************
 * Function:    setTempConf() // PRIVATE
 * Input:       None
 * Output:      None.
 * Overview:    Modifies the configuration register.
 ******************************************************************************/
void setTempConf() {

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (TempAddress << 1) | 0; // Escritura
    i2cData[1] = tempConfReg; //  Registro Configuración
    i2cData[2] = tempConf; // Escritura del registro

    // Comunicación
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // TEMP address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // Modificar el dato
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    return ;
}

/*******************************************************************************
 * Function:    getTempRegister()
 * Input:       None
 * Output:      Ambient temperature.
 * Overview:    Gets the temperature by using MCP9800.
 ******************************************************************************/
void getTempRegister (unsigned int reg){

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (TempAddress << 1) | 0; // Escritura
    i2cData[1] = reg; //  Registro
    i2cData[2] = (TempAddress << 1) | 1; // Lectura

    // Comunicación
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // TEMP address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    RestartI2C2();
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // TEMP address y leer
    IdleI2C2();

}

/*******************************************************************************
 * Function:    getTempConf()
 * Input:       None
 * Output:      Ambient temperature.
 * Overview:    Gets the temperature by using MCP9800.
 ******************************************************************************/
unsigned int getTempConf() {

    getTempRegister(tempConfReg);

    // Leer datos
    unsigned int valor;
    valor = MasterReadI2C2();

    //Cerrar I2C
    StopI2C2();
    IdleI2C2();

    return valor;
}

/*******************************************************************************
 * Function:    getTemp()
 * Input:       None
 * Output:      Ambient temperature.
 * Overview:    Gets the temperature by using MCP9800.
 ******************************************************************************/
unsigned int getTemp (){

    if (isTempLowPower) {
        tempConf = tempConf | 0b10000000;
        setTempConf();
        cntTemp++;
        // Esperar a que se produzca la medida
        // Tienen que estar habilitadas las interrupciones
        // while (cntTemp){}
    }

    getTempRegister(tempReg);

    // Leer datos
    unsigned int temp;
    temp = (MasterReadI2C2() << 8);
    AckI2C2();
    IdleI2C2();
    temp = temp + MasterReadI2C2();
    StopI2C2();
    IdleI2C2();

    return temp;
}

/*******************************************************************************
 * Function:    setTempResolution()
 * Input:       None
 * Output:      Ambient temperature.
 * Overview:    Gets the temperature by using MCP9800.
 ******************************************************************************/
void setTempResolution (int res){

    if (res < 0 || res > 3) {
        return ;
    }

    char mascara;

    // Reseteamos los valores antiguos
    mascara = 0b10011111;
    tempConf = tempConf & mascara;

    // Aplicamos la máscara con el valor actual del registro
    // y lo guardamos de nuevo
    mascara = (res << 5); // Bits que ocupa
    tempConf = tempConf | mascara;

    setTempConf(); //Realiza la comunicación

    return ;
}

/*******************************************************************************
 * Function:    setTempResolution()
 * Input:       None
 * Output:      Ambient temperature.
 * Overview:    Gets the temperature by using MCP9800.
 ******************************************************************************/
void setTempAlert (int reg, INT8 alert){

    if (reg < 2 || reg > 3) {
        return ;
    }

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (TempAddress << 1) | 0; // Escritura
    i2cData[1] = reg; //  Registro Configuración
    i2cData[2] = alert; // Escritura del registro

    // Comunicación
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // TEMP address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // Modificar el dato
    IdleI2C2();
    MasterWriteI2C2(0x00); // Modificar el dato
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    return ;
}

/*******************************************************************************
 * Function:    setTempResolution()
 * Input:       None
 * Output:      Ambient temperature.
 * Overview:    Gets the temperature by using MCP9800.
 ******************************************************************************/
void setTempLowPower (){

    if (isTempLowPower == 0) {
        char mascara;

        // Reseteamos los valores antiguos
        mascara = 0b01111110;
        tempConf = tempConf & mascara;

        // Aplicamos la máscara con el valor actual del registro
        // y lo guardamos de nuevo
        mascara = 1; // Bits que ocupa
        tempConf = tempConf | mascara;
        setTempConf(); //Realiza la comunicación

        isTempLowPower = 1;
    }

    return ;
}

/*******************************************************************************
 * Function:    getTempAlert()
 * Input:       None
 * Output:      NO_ERROR if correct.
 * Overview:    Get 'TRUE' if alert. 'FALSE' if no alert.
 ******************************************************************************/
BOOL getTempAlert (){

    // Activo a nivel bajo
    if (TEMP_ALERT == 0) {
        return TRUE;
    }
    return FALSE;

}

#endif
////////////////////////////////////////////////////////////////////////////////
/******************************* LUMINOSITY ***********************************/
////////////////////////////////////////////////////////////////////////////////

#if defined LUM
/*******************************************************************************
 * Function:    getLum()
 * Input:       None
 * Output:      10-bits representing brightness level.
 * Overview:    Uses internal ADC.
 ******************************************************************************/
unsigned int getLum (){

    unsigned int resultado;
    int i;
    for (i = 0; i<10; i++) {
    unsigned int offset = 8* ((~ReadActiveBufferADC10() & 0x01));
    resultado = (ReadADC10(offset));}
    return resultado;
    
}

////////////////////////////////////////////////////////////////////////////////
/******************************** PRESENCE ************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Function:    getPIR()
 * Input:       None
 * Output:      NO_ERROR if correct.
 * Overview:    Get 'TRUE' if presence. 'FALSE' if no presence.
 ******************************************************************************/
BOOL getPIR (){

    if (GPIO_PRES == 1) {
        return TRUE;
    }
    return FALSE;

}

#endif
////////////////////////////////////////////////////////////////////////////////
/************************************ LEDS ************************************/
////////////////////////////////////////////////////////////////////////////////

#if defined SENSORS
/*******************************************************************************
 * Function:    LedOn(sensorLed sl)
 * Input:       Led on the Sensors Shield
 * Output:      NO_ERROR if correct.
 * Overview:    Simple function to switch on the leds on the sensors shield.
 ******************************************************************************/
BYTE LedOn (sensorLed sl){
   switch(sl){
       case GREEN:
           GREEN_LED = 1;
           break;
       case RED:
           RED_LED = 1;
           break;
       case BOTH:
           GREEN_LED = 1;
           RED_LED = 1;
           break;

       return NO_ERROR;
        }
}

/*******************************************************************************
 * Function:    LedOff(sensorLed sl)
 * Input:       Led on the Sensors Shield
 * Output:      NO_ERROR if correct.
 * Overview:    Simple function to switch off the leds on the sensors shield.
 ******************************************************************************/
BYTE LedOff (sensorLed sl){
   switch(sl){
       case GREEN:
           GREEN_LED = 0;
           break;
       case RED:
           RED_LED = 0;
           break;
       case BOTH:
           GREEN_LED = 0;
           RED_LED = 0;
           break;

       return NO_ERROR;
        }
}

/*******************************************************************************
 * Function:    LedToggle(sensorLed sl)
 * Input:       Led on the Sensors Shield
 * Output:      NO_ERROR if correct.
 * Overview:    Simple function to toggle the leds on the sensors shield.
 ******************************************************************************/
BYTE LedToggle (sensorLed sl){
   switch(sl){
       case GREEN:
           GREEN_LED = (GREEN_LED+1)%2;
           break;
       case RED:
           RED_LED = (RED_LED+1)%2;
           break;
       case BOTH:
           GREEN_LED = (GREEN_LED+1)%2;
           RED_LED = (RED_LED+1)%2;
           break;

       return NO_ERROR;
        }
}
#endif

////////////////////////////////////////////////////////////////////////////////
/************************* TIMER INTERRUPTIONS ********************************/
////////////////////////////////////////////////////////////////////////////////

#if defined SENSORS
/*******************************************************************************
 * Function:    FuncionDePrueba()
 * Input:       None
 * Output:      Returns the byte containing the status flags.
 * Overview:    Simple function to get (read) the status flags.
 ******************************************************************************/
void __ISR(_TIMER_5_VECTOR, ipl7)IntTmp(void) {

    mT5ClearIntFlag();

    if (cntTemp) cntTemp++;

    if (cntTemp == 500) cntTemp = 0;

    if (cntPIR) cntPIR++;

    if (cntPIR == 50000) cntPIR = 0;

    if (nocup) {
        IRProtocol[puntero](); //Puntero a función que implementa el protocolo
    }

    if (isBuzzing) {
        if (buzzerPrescaler++ % 5 == 0) {
            buzzerPrescaler = 1;
            cntBuzzer++;
            if (nota2) {
                if (cntBuzzer % 4 == 0)
                    GPIO_BUZZ ^= 0x0001;
            } else {
                if (cntBuzzer % 3 == 0)
                    GPIO_BUZZ ^= 0x0001;
            }

            if (cntBuzzer == 5000) {
                nota2 ^= 0x0001;
                cntBuzzer = 0;
            }
        }
    } else {
        cntBuzzer = 0;
        GPIO_BUZZ = 0;
    }

}

#endif

#if defined ACC | defined PIR
/*******************************************************************************
 * Function:    FuncionDePrueba()
 * Input:       None
 * Output:      Returns the byte containing the status flags.
 * Overview:    Simple function to get (read) the status flags.
 ******************************************************************************/
void enableIntSensors () {

    ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_7);
    mCNOpen(CN_ON | CN_IDLE_CON, CN15_ENABLE, CN_PULLUP_DISABLE_ALL);
    mPORTDRead();
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_6);
    return;
}

/*******************************************************************************
 * Function:    FuncionDePrueba()
 * Input:       None
 * Output:      Returns the byte containing the status flags.
 * Overview:    Simple function to get (read) the status flags.
 ******************************************************************************/
void disableIntSensors () {

    ConfigIntTimer5(T5_INT_OFF | T5_INT_PRIOR_7);
    ConfigIntCN(CHANGE_INT_OFF | CHANGE_INT_PRI_6);
    return;
}

/*******************************************************************************
 * Function:    FuncionDePrueba()
 * Input:       None
 * Output:      Returns the byte containing the status flags.
 * Overview:    Simple function to get (read) the status flags.
 ******************************************************************************/
void __ISR(_CHANGE_NOTICE_VECTOR, ipl6)IntCN(void) {

    mCNClearIntFlag();
    mPORTDRead(); //Vaciar

    // Sólo activamos la alarma si ha pasado el tiempo de seguridad y
    // no estaba sonando
    if (cntPIR == 0 && isBuzzing == 0) {

        cntPIR++;
        buzzerOn();
    }
}

#endif