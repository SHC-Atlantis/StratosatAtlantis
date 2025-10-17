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
const int kMAIN_LED = 2; //Main LED pin number
const int kLF_SOLENOID = -1; //Left-Front Solenoid pin number
const int kLB_SOLENOID = -1; //Left-Back Solenoid pin number
const int kRF_SOLENOID = -1; //Right-Front Solenoid pin number
const int kRB_SOLENOID = -1; //Right-Back Solenoid pin number


const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

float altitude = 0;
//unsigned long since_last_collection = 0UL;

FlightStage stage;

LED main_LED(kMAIN_LED);

SHC_BME280 bme;
BNO055 accelerometer;
M9N gps;

//Functions

/*
* Collect satellite data and write to "data.txt"
*/
void collectData()
{


  // Year,Month,Day,Hour,Minute,Second,AccX,AccY,AccZ,OrientX,OrientY,OrientZ,GyroX,GyroY,GyroZ,Humidity,Pressure,Temperature,Altitude,Latitude,Longitude,SIV
  String data = ""+
      String(gps.getYear()) + "," +
      String(gps.getMonth()) + "," +
      String(gps.getDay()) + "," +
      String(gps.getHour()) + "," +
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
      
    Serial1.println(data);
    

  //if (data_file) 
  //{
    //data_file.write() //Write to data.txt here

    //
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
void fireSolenoidsByPD(PDCycle cycle)
{
  

}

/*Fires solenoids in pairs based upon the angular position relative to the target position via BangBang
* @param pos_deg: the current angular position
* @param tolerance_deg: how much of an angle to allow pos_deg to be off from the target
*/
void fireSolenoidsByBB(float pos_deg, float tolerance_deg = 0.5)
{
  if (pos_deg > (getErrorAngle(pos_deg) + tolerance_deg)) //Rotate clockwise
  {
    digitalWrite(kLB_SOLENOID, HIGH);
    digitalWrite(kRF_SOLENOID, HIGH);

    digitalWrite(kLF_SOLENOID, LOW);
    digitalWrite(kRB_SOLENOID, LOW);
  }
  else if (pos_deg < (getErrorAngle(pos_deg) - tolerance_deg)) //Rotate counter clockwise
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
  Serial.begin(9600);
  Serial1.begin(115200);

  //csv file header
  Serial1.println("Year,Month,Day,Hour,Minute,Second,AccX,AccY,AccZ,OrientX,OrientY,OrientZ,GyroX,GyroY,GyroZ,Humidity,Pressure,Temperature,Altitude,Lat,Long,SIV");

  //Initialize variables
  stage = FlightStage::LAUNCH;
  main_LED = LED(kMAIN_LED, 50, 950); // blink 1/20 sec at 1hz

  //Initialize systems


  pinMode(kMAIN_LED, OUTPUT);

  pinMode(kLF_SOLENOID, OUTPUT);
  pinMode(kLB_SOLENOID, OUTPUT);
  pinMode(kRF_SOLENOID, OUTPUT);
  pinMode(kRB_SOLENOID, OUTPUT);

  //TODO: Implement error handling
  Serial.println(accelerometer.init());
  Serial.println(String(bme.init()));
  Serial.println(String(gps.init()));
}

void loop() 
{
  
  bme.prefetchData();
  accelerometer.prefetchData();
  gps.prefetchData();

  collectData();

  // digitalWrite(kMAIN_LED, main_LED.update(millis())); //Blink the main LED
  
  // collectData();
  return;

  switch (stage)
  {
    case FlightStage::ASCENT:
      collectData();

      if (altitude > kSTABILIZATION_ALTITUDE)
      {
        stage = FlightStage::STABILIZE;
      }

    break;
    case FlightStage::STABILIZE:
    //firePneumaticsByBB(0.0); //input current heading
    collectData();

    break;
    default:
    break;

    
  }
  
}