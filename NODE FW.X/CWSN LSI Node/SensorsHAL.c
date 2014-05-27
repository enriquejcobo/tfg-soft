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

// Switch on I2C if included TEMP or ACC
#if defined TEMP || defined ACC
    #define I2C2
#endif

// Switch on ADC if included LUM
#if defined LUM
    #define PARAM1 (ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON)
    #define PARAM2 (ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON)
    #define PARAM3 (ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_12)
    #define PARAM5 (SKIP_SCAN_ALL)
    #define PARAM4 (ENABLE_AN13_ANA)
    #define PARAM6 (ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN13)
#endif

/* END DEFINITIONS ************************************************************/

/* VARIABLES ******************************************************************/
//HAL
//nodeStatus NodeStatus;
//UINT32 SleepEventCounter;
//unsigned int coreTMRvals[11];
//BYTE coreTMRptr;


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

    // PIR
    #if defined PIR
        GPIO_PRES_TRIS = INPUT_PIN;
    #endif

    // TEMP y ACC (I2C)
    #if defined ACC || defined TEMP
        #define TempAddress (0x48)
        #define AccAddress (0x1C)
        OpenI2C2 (I2C_EN, (10000));
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

/*******************************************************************************
 * Function:    getTemp()
 * Input:       None
 * Output:      Ambient temperature.
 * Overview:    Gets the temperature by using MCP9800.
 ******************************************************************************/
unsigned int getTemp (){

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (TempAddress << 1) | 0; // Escritura
    i2cData[1] = 0x00; //  Registro Temp. Ambiente
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

    // Datos a mandar
    char i2cData[3];
    i2cData[0] = (TempAddress << 1) | 0; // Escritura
    i2cData[1] = 0x01; //  Registro Configuración
    i2cData[2] = (res << 5); // Escritura resolución

    // Comunicación
    StartI2C2(); // Abrimos i2c
    IdleI2C2(); // wait to complete
    MasterWriteI2C2(i2cData[0]); // TEMP address y escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[1]); // Registro a escribir
    IdleI2C2();
    MasterWriteI2C2(i2cData[2]); // TEMP address y leer
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    return ;
}


/*******************************************************************************
 * Function:    getLum()
 * Input:       None
 * Output:      10-bits representing brightness level.
 * Overview:    Uses internal ADC.
 ******************************************************************************/
unsigned int getLum (){

    unsigned int offset = 8* ((~ReadActiveBufferADC10() & 0x01));
    return (ReadADC10(offset));
    
}


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

/*******************************************************************************
 * Function:    FuncionDePrueba()
 * Input:       None
 * Output:      Returns the byte containing the status flags.
 * Overview:    Simple function to get (read) the status flags.
 ******************************************************************************/
