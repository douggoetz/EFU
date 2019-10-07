

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ~~~~~~~~~~~~~~~~~~~  Libraries   ~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

//// verify that all arduino librarys are being called from SAMD folder//////////
//#include <RTCZero.h> 

#include "EFULibrary.h"



////Instances 

EFULibrary EFU(13);
int testcounter = 0;
int testlastcount = 0;


void setup() {
  
  
  EFU.ResetConfiguration(); //sets DIO inputs/outputs
  testlastcount = millis();
  testcounter = millis();

}

void loop() {

///RTC 30s off
/// RTC and GPS not updated after leaving standby mode
/// Sometimes GPS and RTC not updated in warmup after reset

  //EFU.AcquireGPS();
  EFU.TimeManager();
  // if(testcounter-testlastcount>=5000){
  //   EFU.TimeManager();
  //   testlastcount = millis(); 
  // }
  // testcounter = millis();

  
      
  // }
  



 
}

