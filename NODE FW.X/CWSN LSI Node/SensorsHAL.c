/*******************************************************************************
 * File:   SensorsHAL.c
 * Author: Enrique Cobo Jiménez - Laboratorio de Sistemas Integrados (LSI) - UPM
 *
 * File Description: Sensors and Actuators Hardware Abstraction Level.
 * Implements an API for application and cognitive layers.
 * It's the top level of the LSI-CWSN Microchip MiWi Stack.
 *
 * Change History:
 * Rev   Date         Description
 ******************************************************************************/
/* INCLUDES *******************************************************************/
#include "Include/SensorsHAL.h"            //LSI. HAL Definitions, data types, functions
//#include "Include/Compiler.h"           //General MCHP.

/* END INCLUDES ***************************************************************/

/* CONFIGURATION **************************************************************/

/* END CONFIGURATION **********************************************************/

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

/* VARIABLES ******************************************************************/
//HAL
//nodeStatus NodeStatus;
//UINT32 SleepEventCounter;
//unsigned int coreTMRvals[11];
//BYTE coreTMRptr;

UINT8 AAModo [NBUFFER];
UINT8 AAOnOff [NBUFFER];
UINT8 AACRC [NBUFFER];
UINT8 AATrama1 [7] = {0x11, 0xDA, 0x17, 0x18, T1a, T0, T1b};
UINT8 AATrama2 [15] = {0x11, 0xDA, 0x17, 0x18, T0, T2a, T0, OnCool, T0, T0, TempCool, T2b, T0, T2c, CRCOnCool} ;
int AApos;
int nocup;
int puntero;
int IRcontador;
int IRestado;
int IRindex;

int mimascara;


BOOL isBuzzing;
int cntBuzzer;
int nota2;
int sound;
char tempConf;

int accX, accY, accZ;

int isTempLowPower;
int cntTemp;
INT8 tempAlertMin;
INT8 tempAlertMax;

int isPresence;

////////////////////////////////////////////////////////////////////////////////
/****************   HAL FUNCTIONS (FOR THE APPLICATION CODE)   ****************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Function: InitSensors()
 * Input:    None
 * Output:   NO_ERROR if success. HAL error code otherwise.
 * Overview: Sensor initialization function.
 ******************************************************************************/
BYTE InitSensors(){

      //TIMER5
    #if defined BUZZ || defined TEMP || defined IR
      WORD T5_TICK = (CLOCK_FREQ/8/8/34000);
      OpenTimer5(T5_ON | T5_IDLE_CON | T5_GATE_OFF | T5_PS_1_8 | T5_SOURCE_INT, T5_TICK);
      ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_7);
    #endif

    #if defined BUZZ
      AD1PCFGSET = 0x0008; // Al ser multiplexado con ADCs hay que forzar
      GPIO_BUZZ_TRIS = OUTPUT_PIN;
      GPIO_BUZZ = 0;
      isBuzzing = FALSE;
    #endif

    #if defined IR
        GPIO_IR_TRIS = OUTPUT_PIN;
        GPIO_IR = 0;
    #endif

    // PIR
    #if defined PIR
        mCNOpen(CN_ON | CN_IDLE_CON, CN15_ENABLE, CN_PULLUP_DISABLE_ALL);
        mPORTDRead(); //Vaciar
        ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_6);
        GPIO_PRES_TRIS = INPUT_PIN;
    #endif

    // TEMP y ACC (I2C)
    #if defined ACC || defined TEMP
        #define TempAddress (0x48)
        #define AccAddress (0x1C)
        OpenI2C2 (I2C_EN, (10000));
    #endif

    #if defined TEMP
        tempConf = 0x00;
        #define tempReg (0x00)
        #define tempConfReg (0x01)
        AD1PCFGSET = 0x8000; // Al ser multiplexado con ADCs hay que forzar
        TEMP_ALERT_TRIS = INPUT_PIN;
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

////////////////////////////////////////////////////////////////////////////////
/********************************** IR ****************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Function:    sendIR(int address, int command)
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer on.
 ******************************************************************************/
void sendAA (int modo, int estado) {

    int nuevoEstado, nuevoModo, nuevoCRC;

    if (nocup == NBUFFER) return; // Si no hay hueco lo borramos

    if (modo == AA_Summer) {
        nuevoModo = TempCool;
        if (estado == AA_On) {
            nuevoEstado = OnCool;
            nuevoCRC = CRCOnCool;
        } else {
            nuevoEstado = OffCool;
            nuevoCRC = CRCOffCool;
        }
    } else {
        nuevoModo = TempHeat;
        if (estado == AA_On) {
            nuevoEstado = OnHeat;
            nuevoCRC = CRCOnHeat;
        } else {
            nuevoEstado = OffHeat;
            nuevoCRC = CRCOffHeat;
        }
    }

    AAOnOff[(puntero+nocup) % NBUFFER] = nuevoEstado;
    AAModo[(puntero+nocup) % NBUFFER] = nuevoModo;
    AACRC[(puntero+nocup) % NBUFFER] = nuevoCRC;

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
                AATrama2[7] = AAOnOff[puntero];
                AATrama2[10] = AAModo[puntero];
                AATrama2[14] = AACRC[puntero];
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

////////////////////////////////////////////////////////////////////////////////
/********************************** BUZZER ************************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Function:    buzzerOn()
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer on.
 ******************************************************************************/
void buzzerOn() {

    isBuzzing = TRUE;
    return ;

}

/*******************************************************************************
 * Function:    buzzerOff()
 * Input:       None
 * Output:      None.
 * Overview:    Switchs the buzzer off.
 ******************************************************************************/
void buzzerOff() {

    isBuzzing = FALSE;
    return ;

}

////////////////////////////////////////////////////////////////////////////////
/****************************** ACCELEROMETER *********************************/
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * Function:    getAcc()
 * Input:       None
 * Output:      None.
 * Overview:    Gets the acceleration by using MMA8453Q, and is saved.
 ******************************************************************************/
void getAcc() {

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (AccAddress << 1) | 0; // Escritura
    i2cData[1] = 0x00; //  Registro Temp. Ambiente
    i2cData[2] = (AccAddress << 1) | 1; // Lectura

    // Comunicación
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // TEMP address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    RestartI2C2();
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // TEMP address y leer
    IdleI2C2();

    // Leer datos
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

////////////////////////////////////////////////////////////////////////////////
/******************************* TEMPERATURE **********************************/
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
/******************************* LUMINOSITY ***********************************/
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
/************************************ LEDS ************************************/
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
/************************* TIMER INTERRUPTIONS ********************************/
////////////////////////////////////////////////////////////////////////////////

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

    if (nocup) {
        protocoloAA();
    }

    if (isBuzzing && (sound++ % 5 == 0)) {
        sound = 1;
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

}

/*******************************************************************************
 * Function:    FuncionDePrueba()
 * Input:       None
 * Output:      Returns the byte containing the status flags.
 * Overview:    Simple function to get (read) the status flags.
 ******************************************************************************/
void __ISR(_CHANGE_NOTICE_VECTOR, ipl6)IntCN(void) {

    mCNClearIntFlag();

    isPresence = 1;

}