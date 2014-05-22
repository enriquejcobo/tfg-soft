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
    //TODO: define ADC
#endif

/* END DEFINITIONS ************************************************************/

/* VARIABLES ******************************************************************/
//HAL
nodeStatus NodeStatus;
UINT32 SleepEventCounter;
//unsigned int coreTMRvals[11];
BYTE coreTMRptr;

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

    // Green led
    GREEN_LED_TRIS = OUTPUT_PIN;
    GREEN_LED = 0;

    // Red led
    RED_LED_TRIS = OUTPUT_PIN;
    RED_LED = 0;

    #if defined PIR
    GPIO_PRES_TRIS = INPUT_PIN;
    #endif

    return NO_ERROR;
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
