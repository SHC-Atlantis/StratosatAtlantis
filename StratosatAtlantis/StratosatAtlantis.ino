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
const int kMAIN_LED_1 = 2; //Main LED pin number
const int kCW_SOLENOID = 3; //Clockwise Solenoid pin number
const int kCCW_SOLENOID = 4; //Counter Clockwise Solenoid pin number
const int kMAIN_LED_2 = 5; //Other main LED pin number

const float kASCENT_ALTITUDE = 500.0; //The height required to begin ascent
const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

FlightStage stage;

LED main_LED(kMAIN_LED_1);

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
  // Year,Month,Day,Time,Minute,Second,AccX,AccY,AccZ,OrientX,OrientY,OrientZ,GyroX,GyroY,GyroZ,Humidity,Pressure,Temperature,Altitude,Latitude,Longitude,SIV
  String data = ""+
      String(gps.getYear()) + "," +
      String(gps.getMonth()) + "," +
      String(gps.getDay()) + "," +
      String(gps.getMinute()) + "," +
      String(gps.getSecond()) + "," +
      String(accelerometer.getAccelerationX()) + "," + 
      String(accelerometer.getAccelerationY()) + "," +
      String(accelerometer.getAccelerationZ()) + "," +
      String(accelerometer.getOrientationX()) + "," +
      String(accelerometer.getOrientationY()) + "," + 
      String(accelerometer.getOrientationZ()) + "," +
      String(accelerometer.getGyroX()) + "," +
      String(accelerometer.getGyroY()) + "," +
      String(accelerometer.getGyroZ()) + "," +
      String(bme.getHumidity()) + "," +
      String(bme.getPressure()) + "," +
      String(bme.getTemperature()) + "," +
      String(gps.getAltitude()) + "," +
      String(gps.getLatitude()) + "," +
      String(gps.getLongitude())  + "," +
      String(gps.getSIV()) + ","
      ;
      
    Serial.println(data);
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

/*Fires solenoids at a given rate via PD
* + -> clockwise
* - -> counterclockwise
* @param rate: how long to wait for the next firing of the solenoids
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

/*Fires solenoids based upon the angular position relative to the target position via BangBang
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
  Serial.begin(115200);
  Serial1.begin(115200);

  //csv file header
  Serial.println("Year,Month,Day,Time,Min,Sec,AccX,AccY,AccZ,OrientX,OrientY,OrientZ,GyroX,GyroY,GyroZ,Humidity,Pressure,Temperature,Altitude,Lat,Long,SIV,");

  //Initialize variables
  stage = FlightStage::LAUNCH;
  main_LED_1 = LED(kMAIN_LED_1, 50, 950); // blink 1/20 sec at 1hz

  //Initialize systems

  pinMode(kMAIN_LED_1, OUTPUT);
  pinMode(kMAIN_LED_2, OUTPUT);

  pinMode(kCW_SOLENOID, OUTPUT);
  pinMode(kCCW_SOLENOID, OUTPUT);

  int error_amount = 0;
  int error_stored;

  error_stored = accelerometer.init() //init accelerometer
  error_amount += error_stored;
  
  Serial.println("Accelerometer: " + String(error_stored));

  error_stored = bme.init() //Init BME
  error_amount += error_stored;
  
  Serial.println("BME: " + String(error_stored));

  error_stored = gps.init() //Init GPS
  error_amount += error_stored;
  
  Serial.println("GPS: " + String(error_stored));

  if (error_amount > 0) //If any errors, scan I2C // Credit: Khaled Magdy on DeepBlueMbedded.com
  {
  byte error, address;

  int nDevices;

  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
      { Serial.print("0"); }
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
      { Serial.print("0"); }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
  { Serial.println("No I2C devices found\n"); }
  else
  { Serial.println("done\n"); }
  }
}

void loop() 
{
  collectData();

  bme.prefetchData();
  accelerometer.prefetchData();
  gps.prefetchData();

  digitalWrite(kMAIN_LED_1, main_LED.update(millis())); //Blink the main LED
  digitalWrite(kMAIN_LED_2, main_LED.update(millis()));

  collectData();
  Serial1.println("after collectData()");

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
    default:
    Serial1.println("!ERROR! stage is out of scope of the FlightStage enum!");
    break;
  }
  
}