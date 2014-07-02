/*******************************************************************************
 * File:   SensorsHAL.h
 * Author: Enrique Cobo Jiménez - Laboratorio de Sistemas Integrados (LSI) - UPM
 *
 * File Description: Sensors and Actuators Hardware Abstraction Layer
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
#if defined SENSORS
    typedef enum{
        GREEN, RED, BOTH
    } sensorLed;

////////////////////////////////////////////////////////////////////////////////
/*********************** HAL FUNCTION PROTOTYPES ******************************/
////////////////////////////////////////////////////////////////////////////////
//Initialization and general configuration
    BYTE InitSensors();

    BYTE LedOn(sensorLed sl);
    BYTE LedOff(sensorLed sl);
    BYTE LedToggle(sensorLed sl);

    void enableIntCN();
    void disableIntCN();
#endif

// Temperature sensor
#if defined TEMP
    unsigned int getTemp();
    unsigned int getTempConf();
    BOOL getTempAlert();
    void setTempResolution(int res);
    void setTempLowPower();
    void setTempAlert(int reg, INT8 alert);
#endif

// Accelerometer
#if defined ACC
    void getAcc();
    int getAccX();
    int getAccY();
    int getAccZ();
#endif

// Presence sensor
#if defined PIR
    BOOL getPIR();
#endif

// Luminosity sensor
#if defined LUM
    unsigned int getLum();
#endif

// Infrared emitter
#if defined IR
    #define NBUFFER 4
    #define AA_On 1
    #define AA_Off 0
    #define AA_Summer 1
    #define AA_Winter 0

    void sendAA(int modo, int estado);
#endif

#if defined BUZZ
    void buzzerOn();
    void buzzerOff();
#endif


#endif