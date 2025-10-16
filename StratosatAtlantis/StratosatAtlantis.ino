#include <Utils.h>
#include <SHC_BME280.h>
#include <SHC_BNO055.h>
#include <SHC_M9N.h>
#include <string>
// #include <SD.h>
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
const int kMAIN_LED = LED_BUILTIN; //Main LED pin number
const int kCW_SOLENOID = -1; //Clockwise Solenoid pin number
const int kCCW_SOLENOID = -1; //Counter Clockwise Solenoid pin number

const float kASCENT_ALTITUDE = 500.0; //The height required to begin ascent
const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

FlightStage stage;

LED main_LED(kMAIN_LED);

SHC_BME280 bme;
BNO055 accelerometer;
M9N gps;

Timer above500m_upvelocity_timer(30000); //Timer to track upward velocity and if the satellite is above 500m.
Timer ascent_timer(30000); //Timer to remain in ascension for 30s.
Timer downvelocity_timer(30000); //Timer to track downward velocity for 30s.
Timer novelocity_timer(30000); //Timer to track absence of velocity for 30s.
Timer collection_timer(30000); //Timer to track how long it's been since the last collection in the landed phase.
Timer solenoid_timer(0); //Timer to control the solenoid's rate of fire.

//Functions

/*
* Collect satellite data and write to "data.txt"
*/
void collectData()
{
  //File data_file = SD.open("data.txt", FILE_WRITE);
  Serial1.println("4");
  String data = ""+
      String(accelerometer.getAccelerationX()) + "," + 
      String(accelerometer.getAccelerationY());
    Serial1.println(data);

  //if (data_file) 
  //{
    //data_file.write() //Write to data.txt here

    //date - time - stageOfFlight - pressure - temp - humidity - rotation - lat. - long. - altitude - packetNumber - 
    //String data = ""+
    //  String(accelerometer.getAccelerationX()) + "," + 
    //  String(accelerometer.getAccelerationY());
    //Serial1.println(data);

    //data_file.close();

    //Serial.println("!Data Logged!");
  // return true;
} 
  
  //Serial.println("!Error Opening Log File!");
  //return false;
//}
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
void fireSolenoidsByPD(PDCycle &cycle)
{
  float output = cycle.calculate(millis());

  if (solenoid_timer.isComplete())
  {
    if (output > 0)
    {
      digitalWrite(kCW_SOLENOID, HIGH);
      digitalWrite(kCCW_SOLENOID, LOW);
    }
    else if (output < 0)
    {
      digitalWrite(kCW_SOLENOID, LOW);
      digitalWrite(kCCW_SOLENOID, HIGH);
    }
    else
    {
      if (output > 0)
    {
      digitalWrite(kCW_SOLENOID, LOW);
      digitalWrite(kCCW_SOLENOID, LOW);
    }
    }
  }
  else
  {
    solenoid_timer.reset(fabs(output));
  }

}

/*Fires solenoids in pairs based upon the angular position relative to the target position via BangBang
* @param pos_deg: the current angular position
* @param tolerance_deg: how much of an angle to allow pos_deg to be off from the target
*/
void fireSolenoidsByBB(float pos_deg, float tolerance_deg = 0.5)
{
  if (pos_deg > (getErrorAngle(pos_deg) + tolerance_deg)) //Rotate clockwise
  {
    digitalWrite(kCW_SOLENOID, HIGH);
    digitalWrite(kCCW_SOLENOID, LOW);
  }
  else if (pos_deg < (getErrorAngle(pos_deg) - tolerance_deg)) //Rotate counter clockwise
  {
    digitalWrite(kCW_SOLENOID, LOW);
    digitalWrite(kCCW_SOLENOID, HIGH);
  }
  else //Do not rotate
  {
    digitalWrite(kCW_SOLENOID, LOW);
    digitalWrite(kCCW_SOLENOID, LOW);
  }

}

//Main code

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(115200);

  Serial1.println("0");

  //Initialize variables
  stage = FlightStage::LAUNCH;
  main_LED = LED(kMAIN_LED, 50, 950); // blink 1/20 sec at 1hz

  //Initialize systems


  pinMode(kMAIN_LED, OUTPUT);

  pinMode(kCW_SOLENOID, OUTPUT);
  pinMode(kCCW_SOLENOID, OUTPUT);

  //TODO: Implement error handling
  accelerometer.init();
  Serial1.println(String(bme.init()));
  Serial.println(String(gps.init()));
  Serial1.println("1");
}

void loop() 
{
  collectData();

  Serial1.println("2");
  bme.prefetchData();
  Serial1.println("2.1");
  accelerometer.prefetchData();
  Serial1.println("2.2");
  gps.prefetchData();
  Serial1.println("2.3");

  digitalWrite(kMAIN_LED, main_LED.update(millis())); //Blink the main LED
  Serial1.println("2.4");

  collectData();
  Serial1.println(":)");

  bool above500m_upvelocity = (gps.getAltitude() > 500.0) && (accelerometer.getGyroZ() > 0);
  bool first_run = true;
  bool was_above_stabilization = false;

  switch (stage)
  {
    case FlightStage::LAUNCH:
      if (!above_500m_upvelocity && first_run)
      {
        above500m_upvelocity_timer.reset();
        first_run = false;
      }
      else if (above_500m_upvelocity.isComplete())
      {
        stage = FlightStage::ASCENT;
      }
      else if (!above_500m_upvelocity)
      {
        above500m_upvelocity_timer.reset();
      }
    break;
    case FlightStage::ASCENT:
      Serial1.println("3");
      collectData();

      if (altitude > kSTABILIZATION_ALTITUDE && !was_above_stabilization)
      {
        ascent_timer.reset();
      }
      
      if (accelerometer.getGyroZ() > 2)
      {
        downvelocity_timer.reset();
      }
      else if(downvelocity_timer.isComplete())
      {
        stage = FlightStage::DESCENT;
      }

      if (ascent_timer.isComplete())
      {
        stage = FlightStage::STABILIZE;
      }
      
      was_above_stabilization = altitude > kSTABILIZATION_ALTITUDE;

    break;
    case FlightStage::STABILIZE:
    //firePneumaticsByBB(0.0); //input current heading
    collectData();

    if (accelerometer.getGyroZ() > 2)
    {
      downvelocity_timer.reset();
    }
    else if(downvelocity_timer.isComplete())
    {
      stage = FlightStage::DESCENT;
    }

    break;
    case FlightStage::DESCENT:

      collectData();

      if (fabs(accelerometer.getGyroZ()) > 2)
      {
        novelocity_timer.reset();
      }

      if (novelocity_timer.isComplete())
      {
        stage = FlightStage::LANDED;
      }

    break;
    case FlightStage::LANDED:
      
      if (collection_timer.isComplete())
      {
        collectData();

        collection_timer.reset();
      }
    
  }
  
}