//#include <Utils.h>
// #include <SHC_BME280.h>
// #include <SHC_BNO055.h>
//#include <SD.h>
#include <math.h>

#include "PDCycle.h"
#include "LED.h"


enum class FlightStage
{
  LAUNCH,
  ASCENT,
  STABILIZE,
  DESCENT,
  LANDED
};

//Declare variables
const int kMAIN_LED = 2; //Main LED pin number
const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

FlightStage stage;
LED main_LED;
// SHC_BME280 bme;
// BNO055 accelerometer;


//Functions

/*
* Collect satellite data and write to "data.txt"
*/
bool collectData()
{
  // File data_file = SD.open("data.txt", FILE_WRITE);

  // if (data_file) 
  // {
  //   //data_file.write() //Write to data.txt here

  //   data_file.close();

  //   Serial.println("!Data Logged!");
  //   return true;
  // } 
  
  //Serial.println("!Error Opening Log File!");
  return false;
}
/*
* Gets the angle of error that the satellite must rotate to
* @param init_angle_deg: The angle that satellite is currently facing
*/
float getErrorAngle(float init_angle_deg)
{
  float target_x, target_y;

  return init_angle_deg - tan(target_y / target_x);
}

//Main code

void setup() 
{
  //Initialize variables
  stage = FlightStage::LAUNCH;
  main_LED = LED(kMAIN_LED);

  //Initialize systems
  pinMode(kMAIN_LED, OUTPUT);
  //bme.prefetchData();
  Serial.begin(9600);
}

void loop() 
{
  collectData();

  //bme.prefetchData();

  digitalWrite(kMAIN_LED, main_LED.update(millis())); //Blink the main LED

  //Serial.println(String(bme.getPressure())); //Output the pressure

}
