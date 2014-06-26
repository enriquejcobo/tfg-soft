/*******************************************************************************
 * File:   SensorsHAL.h
 * Author: Enrique Cobo Jiménez - Laboratorio de Sistemas Integrados (LSI) - UPM
 *
 * File Description: Sensors and Actuators Hardware Abstraction Level
 * Change History:
 * Rev   Date         Description
 ******************************************************************************/
/* INCLUDES *******************************************************************/
#ifndef SensorsHAL_H
#define SensorsHAL_H

#include "GenericTypeDefs.h"    //Microchip - Type Definitions
#include "HardwareProfile.h"    //Microchip - Template modified for our design
#include "NodeHAL.h"            //cNGD HAL

/* DATA TYPES AND STRUCTURES **************************************************/
typedef enum{
    GREEN, RED, BOTH
}sensorLed;


////////////////////////////////////////////////////////////////////////////////
/*********************** HAL FUNCTION PROTOTYPES ******************************/
////////////////////////////////////////////////////////////////////////////////
//Initialization
BYTE InitSensors();
#define NBUFFER 4
#define AA_On 1
#define AA_Off 0
#define AA_Summer 1
#define AA_Winter 0

//Configuration
BYTE LedOn(sensorLed sl);
BYTE LedOff(sensorLed sl);
BYTE LedToggle(sensorLed sl);

BOOL getPIR();

unsigned int getLum();

unsigned int getTemp();
unsigned int getTempConf();
BOOL getTempAlert();
void setTempResolution(int res);
void setTempLowPower();
void setTempAlert(int reg, INT8 alert);

void sendAA(int modo, int estado);

void getAcc();
int getAccX();
int getAccY();
int getAccZ();

void buzzerOn();
void buzzerOff();

//Power Management

// LEDS


//Send & Receive - Radio Communication300


//Security


//RadioFrequency


//Node's variables saving:


//Stacks Maintenance

#endif