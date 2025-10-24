#include <Utils.h>
#include <SHC_BME280.h>
#include <SHC_BNO055.h>
#include <SHC_M9N.h>
#include <string>
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


const int kMAIN_LED_1 = 2;
const int kMAIN_LED_2 = 5;//Main LED pin number
const int kLF_SOLENOID = 4; //Left-Front Solenoid pin number CWW
const int kLB_SOLENOID = 3; //Left-Back Solenoid pin number CC
const int kRF_SOLENOID = 3; //Right-Front Solenoid pin number CC
const int kRB_SOLENOID = 4; //Right-Back Solenoid pin number CWW
const int CCW = 4;
const int CW = 3;

float altitude = 25000;

int topReached = 0;


const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

float lastAlt = 0;
int count = 0;
float beginAlt = 0;
float currentAlt = 0;

float pressure_kP = 0;
float temperature_C = 0;

int velocityCap = 30;

String FStage = "INIT";

FlightStage stage;

//LED main_LED(kMAIN_LED);

SHC_BME280 bme;
BNO055 accelerometer;
M9N gps;

ICP201xx ICP(Wire,0);

Timer above500m_upvelocity_timer(30000); //Timer to track upward velocity and if the satellite is above 500m.
Timer ascent_timer(30000); //Timer to remain in ascension for 30s.
Timer downvelocity_timer(3000); //Timer to track downward velocity for 30s.
Timer novelocity_timer(30000); //Timer to track absence of velocity for 30s.
Timer collection_timer(30000); //Timer to track how long it's been since the last collection in the landed phase.
Timer downveloctiy_timer(1000); //Timer for down vel
Timer solenoid_timer(75); //Timer to control the solenoid's rate of fire.
Timer firing_timer(50);
Timer velocitycap_timer(500);
Timer blinky_on_timer(50);
Timer blinky_off_timer(1000);


//Functions

/*
* Collect satellite data and write to "data.txt"
*/
void collectData()
{
  
  ICP.getData(pressure_kP,temperature_C);
  // Serial.println("Stage, UNIX, Year,Month,Day,Hour,Min,Sec,AccX(m/s^2),AccY(m/s^2),AccZ(m/s^2),Yaw(degrees),Roll(degrees),Pitch(degrees),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),Humidity(%rh),Pressure(mb),Temperature(C),Altitude(m),ICP-Pressure(kP),ICP-Temperature(C),Lat,Long,SIV,");
  // Stage,Year,Month,Day,Time,Minute,Second,AccX,AccY,AccZ,OrientX,OrientY,OrientZ,GyroX,GyroY,GyroZ,Humidity,Pressure,Temperature,Altitude,Pressure,Temperature,Altitude,Latitude,Longitude,SIV
  String data = ""+
      String(FStage) + "," +
      String(gps.getUnixTime()) + "," +
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
    //delay(50);
} 
/*
* Gets the angle of error that the satellite must rotate to
* @param init_angle_deg: The angle that satellite is currently facing
*/
float getErrorAngle(float init_angle_deg)
{
  float target_x = 1; //TODO-----------------UPDATE
  float target_y = 1; //TODO-----------------UPDATE

  return init_angle_deg - atan2(target_y,target_x)/3.141592*180; //converts to deg
}

/*Fires solenoids in pairs based upon the angular position relative to the target position via BangBang
* @param pos_deg: the current angular position
* @param tolerance_deg: how much of an angle to allow pos_deg to be off from the target
*/

void fireSolenoidsByBB(float pos_deg, float tolerance_deg = 10)
{

  // delay(50); //-------------------------------------------------------------DELAY-----(don't forget)-----------------------------
  float target = 45.0;
  //Serial.print("Value: ");
  //Serial.println(getErrorAngle(pos_deg));
  Serial.print("error cal: ");
  Serial.println(errorCalc(pos_deg, target));
  Serial.print("orientation: ");
  Serial.println(accelerometer.getOrientationX());
  Serial.print("Solenoid Timer: ");
  Serial.println(solenoid_timer.timeRemaining());
  Serial.print("Firing timer: ");
  Serial.println(firing_timer.timeRemaining());
  

  
  // if(solenoid_timer.isComplete()){
    
  //   if(accelerometer.getGyroZ() > 60){
  //     fireCW();
  //     velocitycap_timer.reset();
  //   }

  //   else if(accelerometer.getGyroZ() < -60){
  //     fireCCW();
  //     velocitycap_timer.reset();
  //   }

  //   else if{
  //   }
  // }


  if (solenoid_timer.isComplete()){

    Serial.println("SOL TIMER COMPLETE");
    // Serial1.println("SOL TIMER COMPLETE");
    // if(firing_timer.isComplete()){
    //   digitalWrite(CW, LOW);
    //   digitalWrite(CCW, LOW);
    // }

    if(velocitycap_timer.isComplete()){
      stopAll();
    }

    if(firing_timer.isComplete()){
      stopAll();
    }

    if(accelerometer.getGyroZ() > 60){
      fireCW();
      velocitycap_timer.reset();
      Serial.println("over 60");
      return;
    }

    else if(accelerometer.getGyroZ() < -60){
      fireCCW();
      velocitycap_timer.reset();
      Serial.println("under -60");
      return;
    }

    

    if (errorCalc(pos_deg, target) < ((-1) * tolerance_deg)) //Rotate clockwise
    {  
      if((accelerometer.getGyroZ() < -10) || ((errorCalc(pos_deg, target)>(-20)) && (accelerometer.getGyroZ() < 0))){
        Serial.println("not CW");
        stopAll();
        return;
      
      }
      else{
        Serial.println("FiredCW");
        
        fireCW();
        firing_timer.reset();
        return;
      }

      //Serial.print("Orientation: ");
      //Serial.println(accelerometer.getOrientationX());
      Serial.println("-------------------------------CLOCKWISE");
    }
    else if (errorCalc(pos_deg, target)>(tolerance_deg)) //Rotate counter clockwise
    {
      if((accelerometer.getGyroZ() > 10) || ((errorCalc(pos_deg, target)>(20)) && (accelerometer.getGyroZ() > 0))){
        Serial.println("not CCW");
        Serial.println("not CCW");
        stopAll();
        return;
      }
      else{
        Serial.println("FiredCCW");
        Serial.println("FiredCCW");
        fireCCW();
        firing_timer.reset();
        return;
      }

      Serial.println("-------------------------------COUNTER CLOCKWISE");
    }

    
    else //Do not rotate
    {
      stopAll();


      Serial.println("-------------------------------ALIGNED");
    }
    // firing_timer.reset();
    // if (firing_timer.isComplete() && solenoid_timer.isComplete()){
    //   Serial.println("THE IF");
    //   Serial1.println("THE IF");

    //   if((errorCalc(pos_deg, target)<(tolerance_deg)) && (errorCalc(pos_deg, target) < ((-1) * tolerance_deg))){
    //     stopAll();
    //   }
    //   stopAll();
    //   //digitalWrite(CW, LOW);
    //   //digitalWrite(CCW, LOW);
    // }

    
  }
  
  else //Do not rotate
  {
    Serial.println("_________________________________________________________________TIMER NOT COMPLETE");
    Serial.println("solenoid_timer");
    Serial.println(solenoid_timer.timeRemaining());
  }
  Serial.print("Gyro: ");
  Serial.println(accelerometer.getGyroZ());
  Serial.print("Solenoid Timer: ");
  Serial.println(solenoid_timer.timeRemaining());
}


void fireCW(){ // fire CW
  digitalWrite(CW, HIGH);
  digitalWrite(CCW, LOW);
  Serial.println("CW");
}

void fireCCW(){ // fire CCW
  digitalWrite(CCW, HIGH);
  digitalWrite(CW, LOW);
  Serial.println("CCW");
}

void stopAll(){ // stop all solenoids
  digitalWrite(CW, LOW);
  digitalWrite(CCW, LOW);
  Serial.println("STOP");
}

double errorCalc(double current, double target){ // error calculations given by mentor Drew 
  return ((((int)(current - target)+ 540) % 360)-180);
}

bool posVelocity(){ // check for positive velocity every second for 30s
  currentAlt = altitude;
  if(count < 30){ 
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
  else if(count >= 30){
    count = 0;
    return true;
  }
  return false;
}

bool negVelocity(int time){ //check for negative velocity every second for 'time' seconds
  currentAlt = altitude;
  if(count < time){
    if(downvelocity_timer.isComplete()){
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

void ledBlink(){  // blink for 1/20th of a sec at 1hz
  if(blinky_off_timer.isComplete()){
      blinky_on_timer.reset(); 
      digitalWrite(kMAIN_LED_1, HIGH);
      digitalWrite(kMAIN_LED_2, HIGH);
      Serial.print("ON");
      blinky_off_timer.reset();
  }
  else if(blinky_on_timer.isComplete()){
      digitalWrite(kMAIN_LED_1, LOW);
      digitalWrite(kMAIN_LED_2, LOW);
      Serial.print("OFF");
    }
  }

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(115200);

  //csv file header
  Serial1.println("Stage,UNIX,Year,Month,Day,Hour,Min,Sec,AccX(m/s^2),AccY(m/s^2),AccZ(m/s^2),Yaw(degrees),Roll(degrees),Pitch(degrees),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),Humidity(%rh),Pressure(mb),Temperature(C),Altitude(m),ICP-Pressure(kP),ICP-Temperature(C),Lat,Long,SIV,");

  stage = FlightStage::LAUNCH;

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

  lastAlt = gps.getAltitude();
  stage = FlightStage::STABILIZE; //TODO:__________________________________________________________________________DELETE_______________________
}

void loop() 
{

  ledBlink(); //--------------------------------------------------------------------------TODO: uncomment

  // if(altitude < 30000 && topReached == 0){
  //   altitude += 16;
  // }
  // else if(altitude >= 30000){
  //   topReached = 1;
  // }
  
  // if(topReached == 1){
  //   altitude -= 16;
  // }

  // if (altitude > 0){
  //   altitude -= 16;
  // }
  // else{
  //   altitude = 0;
  // }
  
  bme.prefetchData();
  accelerometer.prefetchData();
  gps.prefetchData();

  //bool above_500m_upvelocity = (gps.getAltitude() > 500.0) && (negVel());
  //bool first_run = true; // TODO: unused, can it be deleted?
  //bool was_above_stabilization = false;

  //Serial.print("Stage: ");
  // Serial.println(FStage);
  // Serial.print("Alt: ");
  // Serial.println(altitude);

  switch (stage)
  {
    case FlightStage::LAUNCH:
      FStage = "LAUNCH";
      //collectData();
      Serial.print("Gyro: ");
      Serial.println(String(accelerometer.getGyroZ())); //--------------------------------------------------------------

      //Serial.println("pre posVel");
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

      if (negVelocity(30))
        {
          stage = FlightStage::DESCENT;
        }

      
      fireSolenoidsByBB(accelerometer.getOrientationX()); //TODO: Tune / stabilize
      if(solenoid_timer.isComplete()){
        solenoid_timer.reset();
        firing_timer.reset();
      }
    break;

    case FlightStage::DESCENT:
      FStage = "DESCENT";

      collectData();

      if(novelocity_timer.isComplete() && abs((beginAlt - altitude)) > 10)
      {
        novelocity_timer.reset();
        beginAlt = altitude;
      }
      
      else if (novelocity_timer.isComplete() && abs((beginAlt - altitude)) < 10)
      {
        stage = FlightStage::LANDED;
      }
    break;

    case FlightStage::LANDED:
      FStage = "LANDED";
      if (collection_timer.isComplete())
      {
        collectData();

        collection_timer.reset();
      }
    default:
      FStage = "DEFAULT";
      collectData();
      Serial.println("Default");
  
  }
}