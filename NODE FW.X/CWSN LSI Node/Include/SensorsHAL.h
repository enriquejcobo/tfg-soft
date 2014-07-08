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

    void enableIntSensors();
    void disableIntSensors();
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
    #define DATA_READY 0
    #define MOTION_FREEFALL 2
    #define TAP 3
    #define ORIENTATION 4
    #define TRANSIENT 5
    #define ASLEEP 7
    #define NO_INT 8

    #define ACC_INT1 0
    #define ACC_INT2 1

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
    #define AA 1
    #define AA_On 1
    #define AA_Off 0
    #define AA_Summer 1
    #define AA_Winter 0

    void sendIR(INT8 device, INT8 param1, INT8 param2);
#endif

#if defined BUZZ
    void buzzerOn();
    void buzzerOff();
#endif


#endif