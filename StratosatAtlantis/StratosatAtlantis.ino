#include <Utils.h>
#include <SHC_BME280.h>
#include <SHC_BNO055.h>
#include <SHC_M9N.h>
#include <string>
// #include <SD.h>
#include <math.h>

#include "PDCycle.h"
#include "LED.h"

#include "ICP201xx.h"



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
const int kLF_SOLENOID = 4; //Left-Front Solenoid pin number
const int kLB_SOLENOID = 3; //Left-Back Solenoid pin number
const int kRF_SOLENOID = 3; //Right-Front Solenoid pin number
const int kRB_SOLENOID = 4; //Right-Back Solenoid pin number

float altitude = 20000;

int topReached = 0;


const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

float lastAlt = 0;
int count = 0;
float beginAlt = 0;
float currentAlt = 0;

float pressure_kP = 0;
float temperature_C = 0;

String FStage = "INIT";

FlightStage stage;

LED main_LED(kMAIN_LED);

SHC_BME280 bme;
BNO055 accelerometer;
M9N gps;

ICP201xx ICP(Wire,0);

Timer above500m_upvelocity_timer(30000); //Timer to track upward velocity and if the satellite is above 500m.
Timer ascent_timer(30000); //Timer to remain in ascension for 30s.
Timer downvelocity_timer(3000); //Timer to track downward velocity for 30s.
Timer novelocity_timer(30000); //Timer to track absence of velocity for 30s.
Timer collection_timer(30000); //Timer to track how long it's been since the last collection in the landed phase.
Timer solenoid_timer(0); //Timer to control the solenoid's rate of fire.
Timer downveloctiy_timer(1000); //Timer for down vel

//Functions

/*
* Collect satellite data and write to "data.txt"
*/
void collectData()
{
  
  ICP.getData(pressure_kP,temperature_C);
  // Serial.println("Stage,Year,Month,Day,Hour,Min,Sec,AccX(m/s^2),AccY(m/s^2),AccZ(m/s^2),Yaw(degrees),Roll(degrees),Pitch(degrees),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),Humidity(%rh),Pressure(mb),Temperature(C),Altitude(m),ICP-Pressure(kP),ICP-Temperature(C),Lat,Long,SIV,");
  // Stage,Year,Month,Day,Time,Minute,Second,AccX,AccY,AccZ,OrientX,OrientY,OrientZ,GyroX,GyroY,GyroZ,Humidity,Pressure,Temperature,Altitude,Pressure,Temperature,Altitude,Latitude,Longitude,SIV
  String data = ""+
      String(FStage) + "," +
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
      String(pressure_kP) + "," +
      String(temperature_C) + "," +
      String(gps.getLatitude()) + "," +
      String(gps.getLongitude())  + "," +
      String(gps.getSIV()) + ","
      ;
      
    Serial1.println(data);
    delay(50);
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
void fireSolenoidsByBB(float pos_deg, float tolerance_deg = 5)
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



bool posVelocity(){
  Serial.print("Current count ------------->");
  Serial.println(count);
  currentAlt = altitude;
  if(count < 15){ //TODO----------------------------change to 30
    if(downvelocity_timer.isComplete()){
      if((currentAlt - lastAlt) > 0){ //check if alt difference is pos
        count += 1;
      } 
      else{
        count = 0;
        return false;
      }
      lastAlt = currentAlt;
      downvelocity_timer.reset();
    }
  }
  else if(count >= 15){ //TODO----------------------------change to 30
    count = 0;
    return true;
  }
  return false;
}

bool negVelocity(int time){
  Serial.print("(neg) Current count ------------->");
  Serial.println(count);
  currentAlt = altitude;
  if(count < time){
    if(downvelocity_timer.isComplete()){
      Serial.println(currentAlt);
      Serial.println(lastAlt);
      if((currentAlt - lastAlt) < 0){ //check if alt difference is neg
        count += 1;
      } 
      else{
        count = 0;
        return false;
      }
      lastAlt = currentAlt;
      downvelocity_timer.reset();
    }
    lastAlt = currentAlt;
  }
  else if(count >= time){ 
    return true;
  }
  return false;
}

//Main code

void setup() 
{
  
  Serial.begin(9600);
  Serial1.begin(115200);

  //csv file header
  Serial1.println("Stage,Year,Month,Day,Hour,Min,Sec,AccX(m/s^2),AccY(m/s^2),AccZ(m/s^2),Yaw(degrees),Roll(degrees),Pitch(degrees),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),Humidity(%rh),Pressure(mb),Temperature(C),Altitude(m),ICP-Pressure(kP),ICP-Temperature(C),Lat,Long,SIV,");

  //Initialize variables
  stage = FlightStage::LAUNCH;
 // main_LED = LED(kMAIN_LED_1, 50, 950); // blink 1/20 sec at 1hz

  //Initialize systems

  //pinMode(kMAIN_LED_1, OUTPUT);
  //pinMode(kMAIN_LED_2, OUTPUT);

  //pinMode(kCW_SOLENOID, OUTPUT);
  //pinMode(kCCW_SOLENOID, OUTPUT);

  int error_amount = 0;
  int error_stored;

  error_stored = accelerometer.init(); //init accelerometer
  error_amount += error_stored;
  
  Serial.println("Accelerometer: " + String(error_stored));

  error_stored = bme.init(); //Init BME
  error_amount += error_stored;
  
  Serial.println("BME: " + String(error_stored));

  error_stored = gps.init(); //Init GPS
  error_amount += error_stored;
  
  Serial.println("GPS: " + String(error_stored));

  error_stored = ICP.begin(); //Init ICP
  error_amount += error_stored;
  ICP.start();

  Serial.println("ICP: ") + String(error_stored);


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

  lastAlt = gps.getAltitude();
  stage = FlightStage::DESCENT;
}

void loop() 
{
  // if(altitude < 30000 && topReached == 0){
  //   altitude += 16;
  // }
  // else if(altitude >= 30000){
  //   topReached = 1;
  // }
  
  // if(topReached == 1){
  //   altitude -= 16;
  // }

  if (altitude > 0){
    altitude -= 16;
  }
  else{
    altitude == 0;
  }
  


  bme.prefetchData();
  accelerometer.prefetchData();
  gps.prefetchData();

  //bool above_500m_upvelocity = (gps.getAltitude() > 500.0) && (negVel());
  bool first_run = true;
  bool was_above_stabilization = false;

  Serial1.println("BEFORE fire");
  fireSolenoidsByBB(0.0);
  Serial1.println("after fire");

  Serial.print("Stage: ");
  Serial.println(FStage);
  Serial.print("Alt: ");
  Serial.println(altitude);

  switch (stage)
  {
    case FlightStage::LAUNCH:
      FStage = "LAUNCH";
      collectData();

      Serial.println("pre posVel");
      if (posVelocity() == true){
        stage = FlightStage::ASCENT;
      }
    break;

    case FlightStage::ASCENT:
      FStage = "ASCENT";
      collectData();

    if (altitude < kSTABILIZATION_ALTITUDE)
    {
      Serial.println("-----------less than alt to stabilize");
      ascent_timer.reset();
      break;
    }
      
    else if(ascent_timer.isComplete() && (kSTABILIZATION_ALTITUDE < altitude))
    {
      Serial.println("----------- stabilize");
      stage = FlightStage::STABILIZE;
    }

    else if (negVelocity(10))
    {
      Serial.println("loosing alt");
      stage = FlightStage::DESCENT;
    }

    else{
      Serial.println("else");
    }
    Serial.println(ascent_timer.timeRemaining());
    break;

    case FlightStage::STABILIZE:
    FStage = "STABILIZE";
    
    collectData();

    if (negVelocity(5))
      {
        stage = FlightStage::DESCENT;
      }

    fireSolenoidsByBB(0.0); //TODO: Tune / stabilize
    break;

    case FlightStage::DESCENT:
    FStage = "DESCENT";

      collectData();

      if(novelocity_timer.isComplete() && fabs((beginAlt - altitude)) > 10)
      {
        novelocity_timer.reset();
        beginAlt = altitude;
      }
      
      else if (novelocity_timer.isComplete() && fabs((beginAlt - altitude)) < 10)
      {
        stage = FlightStage::LANDED;
      }

      
      // if (fabs(accelerometer.getGyroZ()) > 2)
      // {
      //   novelocity_timer.reset();
      // }

      // if (novelocity_timer.isComplete())
      // {
      //   stage = FlightStage::LANDED;
      // }

    break;
    case FlightStage::LANDED:
      FStage = "LANDED";
      if (collection_timer.isComplete())
      {
        collectData();

        collection_timer.reset();
      }
    default:
    break;

    
  
  }
}