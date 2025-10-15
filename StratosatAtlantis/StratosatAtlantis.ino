#include <Utils.h>
#include <SHC_BME280.h>
#include <SHC_BNO055.h>
#include <SD.h>
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

//The front has the camera

//Declare variables
const int kMAIN_LED = 2; //Main LED pin number
const int kLF_SOLENOID = -1; //Left-Front Solenoid pin number
const int kLB_SOLENOID = -1; //Left-Back Solenoid pin number
const int kRF_SOLENOID = -1; //Right-Front Solenoid pin number
const int kRB_SOLENOID = -1; //Right-Back Solenoid pin number

const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

float altitude = 0;

FlightStage stage;
LED main_LED(kMAIN_LED);
SHC_BME280 bme;
BNO055 accelerometer;


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
/*
* Gets the angle of error that the satellite must rotate to
* @param init_angle_deg: The angle that satellite is currently facing
*/
float getErrorAngle(float init_angle_deg)
{
  float target_x, target_y;

  return init_angle_deg - tan(target_y / target_x);
}

/*Fires solenoids in pairs at a given rate via PD
* + -> clockwise
* - -> counterclockwise
* @param rate: the rate to fire the solenoids. 
*/
void fireSolenoidsByPD(PDCycle cycle)
{
  

}

/*Fires solenoids in pairs based upon the angular position relative to the target position via BangBang
* @param pos_deg: the current angular position
* @param tolerance_deg: how much of an angle to allow pos_deg to be off from the target
*/
void fireSolenoidsByBB(float pos_deg, float tolerance_deg = 0.5)
{
  if (pos > (GetErrorAngle(pos_deg) + tolerance)) //Rotate clockwise
  {
    digitalWrite(kLB_SOLENOID, HIGH);
    digitalWrite(kRF_SOLENOID, HIGH);

    digitalWrite(kLF_SOLENOID, LOW);
    digitalWrite(kRB_SOLENOID, LOW);
  }
  else if (pos < (GetErrorAngle(pos_deg) - tolerance)) //Rotate counter clockwise
  {
    digitalWrite(kLB_SOLENOID, LOW);
    digitalWrite(kRF_SOLENOID, LOW);

    digitalWrite(kLF_SOLENOID, HIGH);
    digitalWrite(kRB_SOLENOID, HIGH);
  }
  else //Do not rotate
  {
    digitalWrite(kLB_SOLENOID, LOW);
    digitalWrite(kRF_SOLENOID, LOW);

    digitalWrite(kLF_SOLENOID, LOW);
    digitalWrite(kRB_SOLENOID, LOW);
  }

}

//Main code

void setup() 
{
  //Initialize variables
  stage = FlightStage::LAUNCH;
  main_LED = LED(kMAIN_LED, 50, 950); // blink 1/20 sec at 1hz

  //Initialize systems
  pinMode(kMAIN_LED, OUTPUT);
  pinMode(kLF_SOLENOID, OUTPUT);
  pinMode(kLB_SOLENOID, OUTPUT);
  pinMode(kRF_SOLENOID, OUTPUT);
  pinMode(kRB_SOLENOID, OUTPUT);

  bme.prefetchData();
  accelerometer.prefetchData();

  Serial.begin(9600);
}

void loop() 
{
  digitalWrite(kMAIN_LED, main_LED.update(millis())); //Blink the main LED

  switch (stage)
  {
    case FlightStage::ASCENT:

      CollectData();

      if (altitude > kSTABILIZATION_ALTITUDE)
      {
        stage = FlightStage::STABILIZE;
      }

    break
    case FlightStage::STABILIZE:

    firePneumaticsByBB(0.0); //input current heading
    CollectData();

  }

}
