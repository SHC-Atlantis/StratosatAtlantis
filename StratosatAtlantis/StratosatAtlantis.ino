#include <SD.h>
#include <SPI.h>

#include "PDCycle.h"
#include "LED.h"


enum class FlightStage
{
  LAUNCH,
  ASCENT,
  STABILIZE,
  DESCENT,
  LANDED
}

//Declare variables
const int kMAIN_LED = 0; //Main LED pin number

FlightStage stage;
LED main_LED;

//Functions

/*
* Collect satellite data and write to "data.txt"
*/
bool collectData()
{
  File data_file = SD.open("data.txt", FILE_WRITE);

  if (data_file) 
  {
    //data_file.write() //Write to data.txt here

    data_file.close();

    Serial.println("!Data Logged!");
    return true;
  } 
  
  Serial.println("!Error Opening Log File!");
  return false;
}

//Main code

void setup() 
{
  //Initialize variables
  stage = FlightStage::Launch;
  main_LED = main_LED(kMAIN_LED, millis())

  //Initialize systems
  pinMode(main_LED, OUTPUT)
}

void loop() 
{

  collectData();

  digitalWrite(main_LED, main_LED.update(millis())); //Flash the main LED

}
