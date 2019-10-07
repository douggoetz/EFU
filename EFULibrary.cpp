
#include "EFULibrary.h"

//Enumeration for state machine
enum EFUState: uint8_t {

  STARTUP,
  WARMUP,
  STANDBY,
  MEASURE,
  TELEMETRY

};


/////////// State machine Variables ///////////
uint8_t inst_state;
uint8_t measure_state;
bool TestState = false; //if 0 then run normally, if 1 then ignore GPS fix

String DataStr = "";
String TimeStr = "";
String GPSStr = "";
String TempStr = "";
//String TemperatureStr = "";
String AnalogStr = "";



//////////// Serial Port constants /////////////
#define FibSerial  Serial //TX pin for fiber optic transmitter
#define GPSSerial Serial1 //RX and TX from UBLOX

///////// Pin Definitions //////////////

#define CHIP_SELECT 5 //LTC2983 chip select
#define RESET 6 //LTC2983 reset pin

#define FiberPower 13 // fiber transmitter power

#define GPS_ExtInt 2 //Interput Pin to place UBLOX in sleep mode
#define GPS_Reset 3 //Reset pin for UBLOX
#define GPS_Power 4 //Power to SIP switch that controls 3v to UBLOX

#define SHUTDOWN 9 //solar input power switch
#define CHARGE_OFF 10 //battery charger switch
#define HEATER_ON 11 //battery heater switch

////////// Analog Channels ////////////////////
#define INPUT_V A0 //should be 3.3V
#define SOLAR_V A1 //solar charger voltage
#define BATT_V  A2 //lithium battery voltage

#define TXBUFSIZE 2000


///////////  instances of external classes ///////
RTCZero rtc;
Adafruit_GPS GPS(&GPSSerial); 
#define GPSECHO false
CircularBuffer<int,TXBUFSIZE> txbuf;



/////////// definitions of buffers, variables, and strings //////////////////////

//UBLOX config bytes
byte PowerSaveMode[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x48, 0x01, 0x62, 0x12};
byte CyclicTracking[] = {0xB5, 0x62, 0x06, 0x3B, 0x30, 0x00, 0x02, 0x06, 0x00, 0x00, 0x00, 0x10, 0x4, 0x01, 0xE8,0x03, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x02};
byte GNSSConfig[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2C, 0x4D}; 
byte SaveUBLOXConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA9};
byte GNGGL_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
byte GNGSA_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
byte GNGSV_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
byte GNVTG_off[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
byte UBX_Set_Airborne[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 
                              0x05, 0x00, // mask (only update dyn model and fix mode)
	                          0x7, // dynamic model = airborne <2g
	                          0x3, // fix mode = 2D/3D
	                          0x0, 0x0, 0x0, 0x0, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 
	                          0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 
	                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	                          0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xf9}; // 0x1d, 0xd8 for 0x2 fix mode


//RTC setup
byte seconds = 0;
byte minutes = 0;
byte hours = 0;
byte year = 0;
byte month = 0;
byte day = 0;
byte sleepalarm;
int MCounter = 0;

//GPS
float Latitude;
float Longitude;
float Altitude;
uint8_t Satellites;

//LTC Temperature
float PWD_Therm_T; //Ch 4
float Batt_Therm_T; //Ch 6
float FibPRT1_T; //Ch 16
float FibPRT2_T; //Ch 18
float OAT_T; //Ch 20 

//LTC RTD voltages
float FibPRT1_V; //Ch 16
float FibPRT2_V; //Ch 18
float OAT_V; //Ch 20 


//Analog
float val;
float Vout;
float vIn;
char inChar;
float vBatt;
float vSolar;

// Timing variables
int StartupInterval = 30000; //30s between loops to monitor/control temperature and monitor voltage 
int WarmupInterval = 1000; // 1Hz loops to get GPS lock and set RTC
int StandbyInterval = 100; // millis until next loop
int MeasureInterval = 2000; // 0.5Hz loops to get measurements of temperature, voltage, and GPS location
int TelemInterval = 1000; //1 Hz loops to send telemetry to the DIB

int StandbyPeriod = 5; //number of minutes until next warmup/measurement period
int MeasurePeriod = 1; //number of minutes to make 0.5hz measurements (to stay in MEASURE)
int TelemPeriod = 1; //number of minutes to send telemetry to the DIB
int TelemStartMin = 59; //minute to interrupt out of warmup, standby, and meausurement mode and go into telemetry mode 
int startminute;

//Moving Average Values
float vIn_Avg;
float vBatt_Avg;
float vSolar_Avg;
long Latitude_Avg;
long Longitude_Avg;
float Altitude_Avg;
float PWD_Therm_Avg;
float Batt_Therm_Avg;
float FibPRT1_Avg;
float FibPRT2_Avg;
float OAT_Avg;  
float Satellites_Avg;

//String ToAverageStr[] = {"Latitude", "Longitude", "Altitude", "Satellites", "vIn", "vBatt", "vSolar", "PWD_Therm_T", "Batt_Therm_T", "FibPRT1_T", "FibPRT1_T", "OAT_T"};
String datastr;
int CRC;
int reset_crc = 0;

//flags
bool StartupConfig = false;
bool enterstate = true;
bool RTCupdate = false;
int BattHeaterState;


//Operational thresholds
int ChargeSetTemp = 0;
int TempSpan = 1;
float ChargeVLimit = 4.2;
int IntervalDiff = 0;
int RTCDiff = 0;


EFULibrary::EFULibrary(int pin)
{
  pinMode(pin, OUTPUT);
  _pin = pin;
  delay(1000);//while (!Serial); // Wait until Serial is ready
}    

void EFULibrary::ManageState() {
  
  switch (inst_state) {
    case STARTUP:
      SerialUSB.println("Enter Startup Mode");
      if(StartupConfig == false){
        ResetConfiguration();
      }
      //turn solar charger on, monitor/control temperature until V_batt is good
      SolarOn();
      ChargerOn();
      HeaterControl(ChargeSetTemp, TempSpan);
      ReadAnalogChannels();

      if(BATT_V < 0 || BATT_V > 4.5){ //if there is a bad ADC reading reset EFU
        ResetConfiguration();
        SerialUSB.println("Bad ADC reading in startup")
        BATT_V = 0; // change to zero to restart switch-case by not entering if statment below
      }
      
      if(BATT_V>=ChargeVLimit){  
        inst_state = WARMUP; //next state to go into
        //measure_state = STANDBY; //go into stanby state if any sleep mode is needed
        measure_state = MEASURE; // go straight into measurement state after warmup
        enterstate = true;
        RTCupdate = false;
        SerialUSB.println("Exit Startup Mode");
      }
      
      break;

    case WARMUP:

      if(TestState){ //used if GPS isn't available during testing

        GPSon();
        sendGPSconfig();
        delay(100);
        enterstate = false;
        rtc.setTime(hours,minutes,seconds); 
        rtc.setDate(day, month, year);
    
        inst_state = MEASURE;
        SerialUSB.println(measure_state);
        SerialUSB.println("Exit warmup and go into measurement");
        startminute = rtc.getMinutes();
        break;

      }

      if(enterstate){ //when first going into WARMUP turn on/configure GPS
        SerialUSB.println("Enter Warmup");
        GPSon();
        sendGPSconfig();
        delay(100);
        enterstate = false; 
      }

      //Acquire NMEA strings
      AcquireGPS();
      SerialUSB.println("GPS Acquire loop");

      if(GPS.fixquality == 1 && RTCupdate == false){ //if there is a GPS fix and the RTC needs to be updated

        rtc.setTime(GPS.hour,GPS.minute,GPS.seconds); 
        rtc.setDate(GPS.day, GPS.month, GPS.year);  //set RTC
        printRTC();
        RTCupdate = true; // starts RTC update cycle
        SerialUSB.println("Set RTC");
      }

      if(RTCupdate){  // if the RTC has been updated go into MEASURE or STANDBY state
        switch (measure_state) {

        case MEASURE: //go into measure mode after warmup
          inst_state = MEASURE;
          startminute = rtc.getMinutes();
          //enterstate = true;
          break;

        case STANDBY: // go into standby mode after warmup
          inst_state = STANDBY;
          startminute = rtc.getMinutes();
          //enterstate = true;
          SerialUSB.println(measure_state);
          SerialUSB.println("Exit warmup and go into standby");
          break;

        default:
          inst_state = WARMUP; //REDO warmup mode because of old RTC set
          //enterstate = true;
          RTCupdate = false;
          break;
        }
      }
      break;

    case STANDBY:
      
      SerialUSB.println("In Standby");

      ////// Standby to be reserved for if a "Low Power" mode is needed ///////
      
      //AcquireGPS();
      //HeaterControl(ChargeSetTemp, TempSpan);  //monitor/control heater
      //ReadAnalogChannels();
      //AcquireTempC();
      //AcquireRTDVoltages();
      //FiberTXOn();
      //delay(10);
      //Send time
      //FibSerial.flush();
      //TimeStr = (String(rtc.getYear())+String(rtc.getMonth())+String(rtc.getDay())+","+String(rtc.getHours())+":"+String(rtc.getMinutes())+":"+String(rtc.getSeconds())+",");
      //GPSStr = (String(GPS.year)+String(GPS.month)+String(GPS.day)+","+String(GPS.hour)+":"+String(GPS.minute)+":"+String(GPS.seconds)+","+String(GPS.latitude_fixed)+","+String(GPS.longitude_fixed)+","+String(GPS.altitude)+",");
      //TempStr = (String(BattHeaterState)+","+String(FibPRT1_T)+","+String(FibPRT2_T)+","+String(OAT_T)+","+String(PWD_Therm_T)+","+String(Batt_Therm_T)+",");
      //AnalogStr = (String(vBatt)+","+String(vIn));

      //FibSerial.print(TimeStr+GPSStr+TempStr+AnalogStr+'\r');
      //FibSerial.println("");
      
  
      //Analog voltages
      
      // //Send RTD voltage
      // FibSerial.print(FibPRT1_V);
      // FibSerial.print(",");
      // FibSerial.print(FibPRT2_V);
      // FibSerial.print(",");
      // FibSerial.print(OAT_V);
      // FibSerial.print(",");

      // FibSerial.print(vBatt);
      // FibSerial.print(",");
      // FibSerial.print(vSolar);
      // FibSerial.print(",");
      // FibSerial.print(vIn);
      // FibSerial.println("");
      // FibSerial.flush();
      // //FiberTXOff();

      break;

    case MEASURE:

      HeaterControl(ChargeSetTemp, TempSpan);  //monitor/control heater
      AcquireGPS();
      ReadAnalogChannels();
      AcquireTempC();
      
      //to do:
      //set/control time for measurement/averaging/telemetry
      break;

    case TELEMETRY:

      GPSoff();
      FiberTXOn();
      //to do: send telemetry
      //to do: clear buffer
      break;    
    
    default:
      SerialUSB.println("Unknown State, resetting EFU");
      inst_state = STARTUP; // reset
      break;
  }

}

void EFULibrary::TimeManager(){

  counter = millis();

  switch (inst_state) {
  
  case STARTUP:

    if(counter-lastcount >= StartupInterval){
      ManageState();
      lastcount = millis();
    }
    break;
  
  case WARMUP:

    if(rtc.getMinutes()==TelemStartMin){ // Send to telemetry state if within telemetry window minute
      inst_state = TELEMETRY;
      break;
    }

    if(counter-lastcount >= WarmupInterval){
      ManageState();
      lastcount = millis();
    }  
    break;
  
  case STANDBY:
    
    if(rtc.getMinutes()==TelemStartMin){ // Send to telemetry state if within telemetry window minute
      inst_state = TELEMETRY;
      break;
    }
    
    // if(rtc.getMinutes()-startminute!=0&&((rtc.getMinutes()+60)-startminute)%StandbyPeriod == 0){ // Send to warmup state when Standby period is over
    //   inst_state = WARMUP;
    //   enterstate = true;
    //   measure_state = MEASURE;
    //   RTCupdate = false;
    //   printRTC();
    //   break;
    // }

    if(counter-lastcount >= StandbyInterval){ // Maintain temperature or other settings at Standby interval
      ManageState();
      lastcount = millis();   
    }
    break;
  
  case MEASURE:

    if(rtc.getMinutes()==TelemStartMin){ // Send to telemetry state if within telemetry window minute
      inst_state = TELEMETRY;
      break;
    }

    if(rtc.getMinutes()-startminute!=0 && ((rtc.getMinutes()+60)-startminute)%MeasurePeriod == 0){ 
      
      //to do: put averaging function here

    }

    if(counter-lastcount >= MeasureInterval){
      ManageState();
      lastcount = millis();   
    }
    break;
  
  case TELEMETRY:
    
    if(rtc.getMinutes()%TelemPeriod == 0){ // Send to warmup state when telemetry period is over
      inst_state = WARMUP; // when telemetry period is over go into warmup state
      enterstate = true; //set enterstate = true to turn on and configure GPS
      measure_state = MEASURE; //goes into measurement state after warmup
      RTCupdate = false; //used to reset RTC with GPS time
      break;
    }

    if(counter-lastcount >= TelemInterval){ // Maintain temperature at Standby interval
      ManageState();
      lastcount = millis();   
    }
    break;
  
  default:
    SerialUSB.println("Timing unknown resetting EFU");
    inst_state = STARTUP;
    break;

  }
}

void EFULibrary::ResetConfiguration(){
   
  delay(5000); // delay used to reprogram if there is a sleep mode error

  //configure comms 
  // setup native USB for debugging
  SerialUSB.begin(9600);
  FibSerial.begin(115200);
  GPSSerial.begin(9600);

  
  rtc.begin(); 
  //rtc.setTime(hours,minutes,seconds);
  delay(100);
  
  //GPS pins
	pinMode(GPS_Power, OUTPUT); //digitalwrite high to turn on
  GPSoff();
  //GPSon(); //for testing
  //delay(100);
  //sendGPSconfig(); //for testing

  //Fiber TX DIO
	pinMode(FiberPower, OUTPUT); //digitalwrite high to turn on
  //FiberTXOn();
  FiberTXOff();


	//LTC Temperature IC Pins
   pinMode(CHIP_SELECT, OUTPUT); // Configure chip select pin on Linduino
   pinMode(RESET,OUTPUT);
   digitalWrite(RESET, HIGH);
   delay(100);
   SPI.begin();
   SPI.setClockDivider(SPI_CLOCK_DIV128);

   ConfigureChannels();
   configure_global_parameters();


  //Battery Charger and Heater
  pinMode(SHUTDOWN,OUTPUT); //or __charge__
  SolarOff(); //solar off during startup
  
  pinMode(CHARGE_OFF, OUTPUT); //LED mosfet for charger
  ChargerOn();//charger on


  pinMode(HEATER_ON, OUTPUT); // LED mosfet for heater
  HeaterOff(); //heater off

  analogReference(AR_EXTERNAL);
  analogWriteResolution(10);//Full DAQ resolution for arduino zero 

  inst_state = STARTUP;
  StartupConfig = true; 
  //measure_state = STANDBY; // to use if "low power" mode is needed
  measure_state = MEASURE;
  RTCupdate = false;
    		
  counter = millis();
  lastcount = millis();

  SerialUSB.println("Reset Configuration");
}

void EFULibrary::HeaterControl(int setT, int SetTspan){

  float BattT = MeasureLTC2983(6);
  float PWDT = MeasureLTC2983(4);
  int highspan = setT+SetTspan;
  int lowspan = setT-SetTspan;
  SerialUSB.print(BattT);
  SerialUSB.print(",");
  SerialUSB.print(PWDT);
  SerialUSB.println(",");

  
  // if(BattT ==-999 || PWDT ==-999){ //if errors values then reset chip

  //   digitalWrite(RESET, HIGH);
  //   delay(100);
  //   digitalWrite(RESET, LOW);
  //   ConfigureChannels();
  //   configure_global_parameters();

  // }
  
  if(BattT >= highspan){// || PWDT >= highspan){ //if >setT then turn heater off
    SerialUSB.println("Heater OFF");
    BattHeaterState = 0;
    HeaterOff();
  }

  if(BattT <=lowspan){//} || PWDT <=lowspan){
    SerialUSB.println("Heater ON");
    BattHeaterState = 1;
    HeaterOn();
  }

}

void EFULibrary::ConfigureChannels(){
   
   uint8_t channel_number;
   uint32_t channel_assignment_data;
   
 // ----- Channel 2: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0xFA000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 1000.
  assign_channel(CHIP_SELECT, 2, channel_assignment_data);
  // ----- Channel 4: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__2 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(CHIP_SELECT, 4, channel_assignment_data);
  // ----- Channel 6: Assign Thermistor 44006 10K@25C -----
  channel_assignment_data = 
    SENSOR_TYPE__THERMISTOR_44006_10K_25C |
    THERMISTOR_RSENSE_CHANNEL__2 |
    THERMISTOR_DIFFERENTIAL |
    THERMISTOR_EXCITATION_MODE__SHARING_NO_ROTATION |
    THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
  assign_channel(CHIP_SELECT, 6, channel_assignment_data);
  // ----- Channel 14: Assign Sense Resistor -----
  channel_assignment_data = 
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x9C4000 << SENSE_RESISTOR_VALUE_LSB;		// sense resistor - value: 10000.
  assign_channel(CHIP_SELECT, 14, channel_assignment_data);
  // ----- Channel 16: Assign RTD PT-100 -----
  channel_assignment_data = 
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__14 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__50UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(CHIP_SELECT, 16, channel_assignment_data);
  // ----- Channel 18: Assign RTD PT-100 -----
  channel_assignment_data = 
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__14 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__50UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(CHIP_SELECT, 18, channel_assignment_data);
  // ----- Channel 20: Assign RTD PT-100 -----
  channel_assignment_data = 
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__14 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__50UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(CHIP_SELECT, 20, channel_assignment_data);

}

void EFULibrary::configure_global_parameters(){    

  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
    REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xFF, 0);
  

}

float EFULibrary::MeasureLTC2983(int channel){
   float temp;
    temp =  measure_channel(CHIP_SELECT, channel, TEMPERATURE);      // Ch 4: Thermistor 44006 10K@25C
   return temp;
} 

float EFULibrary::MeasureLTC2983_V(int channel){
   float temp;
    temp =  measure_channel(CHIP_SELECT, channel, VOLTAGE);
   return temp;
} 

void EFULibrary::LTC_sleep(){

	transfer_byte(CHIP_SELECT, WRITE_TO_RAM, COMMAND_STATUS_REGISTER, SLEEP_BYTE);	
} 

void EFULibrary::FiberTXOff(){
  digitalWrite(FiberPower, LOW);
}

void EFULibrary::FiberTXOn(){
  digitalWrite(FiberPower, HIGH);
}

void EFULibrary::GPSon(){
  digitalWrite(GPS_Power, HIGH);
}

void EFULibrary::GPSoff(){
  digitalWrite(GPS_Power, LOW);
}

void EFULibrary::sendGPSconfig(){
    GPSSerial.write(PowerSaveMode, sizeof(PowerSaveMode));
    GPSSerial.flush();

    GPSSerial.write(CyclicTracking,sizeof(CyclicTracking));
    //GPSSerial.write(CyclicTracking,56);
    GPSSerial.flush();

   //GPSSerial.write(GNSSConfig, 68); //command GPS to use GPS satellites only for reduce power consumption
   //GPSSerial.flush();
    
    GPSSerial.write(GNGGL_off,sizeof(GNGGL_off));
    GPSSerial.flush();
    
    GPSSerial.write(GNGSA_off,sizeof(GNGSA_off));
    GPSSerial.flush();

    GPSSerial.write(GNGSV_off, sizeof(GNGSV_off));
    GPSSerial.flush();
    
    GPSSerial.write(GNVTG_off, sizeof(GNVTG_off));
    GPSSerial.flush();

    GPSSerial.write(UBX_Set_Airborne, sizeof(UBX_Set_Airborne));
    GPSSerial.flush();

    GPSSerial.write(SaveUBLOXConfig, sizeof(SaveUBLOXConfig)); //save all GPS settings w/ battery backed ram
    GPSSerial.flush();
}

void EFULibrary::AcquireGPS(){

    //while(GPS.fix == false){
      char c = GPS.read();
      if (GPSECHO)
      if (c) SerialUSB.print(c);
      if (GPS.newNMEAreceived()) {
      SerialUSB.println("got new NMEA string");
      if (!GPS.parse(GPS.lastNMEA()))
      return;
      }   
    //} 
}  

void EFULibrary::SolarOff(){
  digitalWrite(SHUTDOWN, LOW); 
  //SolarChargeState = 0;
}

void EFULibrary::SolarOn(){
  digitalWrite(SHUTDOWN, HIGH);
  //SolarChargeState = 1; 
}

void EFULibrary::ChargerOn(){
  digitalWrite(CHARGE_OFF, HIGH);
}

void EFULibrary::ChargerOff(){
  digitalWrite(CHARGE_OFF, LOW);
}

void EFULibrary::HeaterOn(){
  digitalWrite(HEATER_ON, HIGH); 
  //BattHeaterState = 1;
}

void EFULibrary::HeaterOff(){
  digitalWrite(HEATER_ON, LOW); 
  //BattHeaterState = 0;
}

void EFULibrary::ReadAnalogChannels() {

   val = analogRead(INPUT_V); 
   Vout = val*(3.0/1023.0);
   vIn = ((Vout*(10000.0+23200.0))/23200.0);  
   SerialUSB.print(vIn);
   SerialUSB.println("");

   val = analogRead(BATT_V);    
   Vout = val*(3.0/1023.0);
   vBatt = ((Vout*(4700.0+4700.0))/4700.0); 
   SerialUSB.print(vBatt);
   SerialUSB.print(", ");

   val = analogRead(SOLAR_V);
   Vout = val*(3/1023.0);
   vSolar =((Vout*(10000.0+10000.0))/10000.0); 
   SerialUSB.print(vSolar);
   SerialUSB.print(", "); 
}

void EFULibrary::printRTC(){

  SerialUSB.print("RTC:");
  SerialUSB.print(rtc.getHours());
  SerialUSB.print(":");
  SerialUSB.print(rtc.getMinutes());
  SerialUSB.print(":");
  SerialUSB.println(rtc.getSeconds());


}
       
void EFULibrary::AcquireTempC(){

	  FibPRT1_T = MeasureLTC2983(16); 
    FibPRT2_T = MeasureLTC2983(18); 
    OAT_T = MeasureLTC2983(20); 
    PWD_Therm_T = MeasureLTC2983(6); 
    Batt_Therm_T = MeasureLTC2983(4); 

} 

void EFULibrary::AcquireRTDVoltages(){

	  FibPRT1_V = MeasureLTC2983_V(16); 
    FibPRT2_V = MeasureLTC2983_V(18); 
    OAT_V = MeasureLTC2983_V(20); 

}





