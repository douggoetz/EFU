/*

EFUMain

Created by Doug Goetz starting in 12/2017 

*/

#ifndef EFULibrary_h
#define EFULibrary_h


// verify that all arduino librarys are being called from SAMD folder
#include <Arduino.h>
//#include <stdint.h>
//#include <stdbool.h>
#include "SPI.h"
#include "Wire.h"
//#include "stdio.h"
//#include "math.h"
#include <RTCZero.h>
#include <Adafruit_GPS.h>
#include <CircularBuffer.h>

// LTC2983 Temperature IC Libraries 
#include "Charge_LTC2983_configuration_constants.h"
#include "Charge_LTC2983_support_functions.h"
#include "Charge_LTC2983_table_coeffs.h"



class EFULibrary
{
  public:
    EFULibrary(int pin);
    void ManageState(); //Switch-case that uses the EFUState enum for a state machine
    void ResetConfiguration(); //Sets safe configuration for startup and watchdog reset
    void TimeManager();
    
    //Control Heater based on battery temperature
    void HeaterControl(int setT, int setTspan);

    //LTC Temperature measurement 
    void ConfigureChannels(); //Configure LTC2983 Channel settings
    void configure_global_parameters(); //configures globals for LTC chip
    float MeasureLTC2983(int channel);//returns the temperature of a given channel in degrees C.
    float MeasureLTC2983_V(int channel);//returns the temperature of a given channel in degrees C.
    void LTC_sleep(); //used to put LTC in low power sleep mode.
    void AcquireTempC();
    void AcquireRTDVoltages();

    //Fiber TX on/off
    void FiberTXOff();  
    void FiberTXOn();

    // UBLOX GPS controls
    void GPSon();
    void GPSoff();
    void sendGPSconfig();
    void AcquireGPS();

    // Generic Solar Charger functions
    void SolarOff(); //only off during reset
    void SolarOn();
    void ChargerOn();
    void ChargerOff();
    void HeaterOn();
    void HeaterOff();

    //Analog voltages
    void ReadAnalogChannels(); //read and reports Vbatt, Vsolar, Vcharge

    //debug functions:
    void printRTC();

    unsigned long counter;
    unsigned long lastcount;
    
  private:
    int _pin;

};

#endif
