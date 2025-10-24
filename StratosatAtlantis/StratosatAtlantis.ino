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

enum class SolenoidSetting
{
  CW,
  CCW,
  OFF
};

//The front has the camera

//Declare variables
const int kMAIN_LED = 2; //Main LED pin number
const int kLF_SOLENOID = 4; //Left-Front Solenoid pin number CWW
const int kLB_SOLENOID = 3; //Left-Back Solenoid pin number CC
const int kRF_SOLENOID = 3; //Right-Front Solenoid pin number CC
const int kRB_SOLENOID = 4; //Right-Back Solenoid pin number CWW
const int CCW = 4;
const int CW = 3;

float altitude = 25000.0;

int topReached = 0;

float target = -144; //Orient to -144 degrees by default

const float kSTABILIZATION_ALTITUDE = 20000.0; //The height required to begin stabilization

float lastAlt = 0;
int count = 0;
float beginAlt = 0;
float currentAlt = 0;

float pressure_kP = 0;
float temperature_C = 0;

String FStage = "INIT";

FlightStage stage;

<<<<<<< Updated upstream
LED main_LED(kMAIN_LED);
=======
SolenoidSetting fire_setting;

LED main_LED(kMAIN_LED_1);
>>>>>>> Stashed changes

SHC_BME280 bme;
BNO055 accelerometer;
M9N gps;

ICP201xx ICP(Wire,0);

Timer above500m_upvelocity_timer(30000); //Timer to track upward velocity and if the satellite is above 500m.
Timer ascent_timer(30000); //Timer to remain in ascension for 30s.
Timer downvelocity_timer(3000); //Timer to track downward velocity for 30s.
Timer novelocity_timer(30000); //Timer to track absence of velocity for 30s.
Timer collection_timer(30000); //Timer to track how long it's been since the last collection in the landed phase.
Timer solenoid_timer(1000); //Timer to control the solenoid's rate of fire.
Timer downveloctiy_timer(1000); //Timer for down velocity
Timer firing_timer(100);

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

void fireSolenoidsByBB(float pos_deg, float tolerance_deg = -10)
{
  bool rotate_cw = false;
  bool rotate_ccw = false;
  bool dont_rotate = false;

  // delay(50); //-------------------------------------------------------------DELAY-----(don't forget)-----------------------------
  float target = 45.0;
  Serial1.print("error cal: ");
  Serial1.println(errorCalc(pos_deg, target));
  Serial1.print("orientation: ");
  Serial1.println(accelerometer.getOrientationX());
  Serial1.print("Solenoid Timer: ");
  Serial1.println(solenoid_timer.timeRemaining());
  Serial1.print("Firing timer: ");
  Serial1.println(firing_timer.timeRemaining());
  if (solenoid_timer.isComplete()){

    if (errorCalc(pos_deg, target)<tolerance_deg) //Rotate clockwise
    {
      digitalWrite(CW, HIGH);
      digitalWrite(CCW, LOW);

      Serial1.println("-------------------------------CLOCKWISE");
    }
    else if (errorCalc(pos_deg, target)>fabs(tolerance_deg)) //Rotate counter clockwise
    {
      digitalWrite(CW, LOW);
      digitalWrite(CCW, HIGH);
    }

    Serial1.print("TARGET VALUE-------------------------------------------------------------------------->");
    Serial1.println(target);

    Serial.println("SOL TIMER COMPLETE");
    Serial1.println("SOL TIMER COMPLETE");
    // if(firing_timer.isComplete()){
    //   digitalWrite(CW, LOW);
    //   digitalWrite(CCW, LOW);
    // }

    if(velocitycap_timer.isComplete()){
      //stopAll();
      fire_setting = SolenoidSetting::OFF;
      dont_rotate = true;
    }

    if(firing_timer.isComplete()){
      //stopAll();
      fire_setting = SolenoidSetting::OFF;
      dont_rotate = true;
    }

    if(accelerometer.getGyroZ() > 60){
      //fireCW();
      fire_setting = SolenoidSetting::CW;
      rotate_cw = true;
      velocitycap_timer.reset();
      Serial1.println("over 60");
      return;
    }

    else if(accelerometer.getGyroZ() < -60){
      //fireCCW();
      fire_setting = SolenoidSetting::CCW;
      rotate_ccw = true;
      velocitycap_timer.reset();
      Serial1.println("under -60");
      return;
    }

    

    if (errorCalc(pos_deg, target) < ((-1) * tolerance_deg)) //Rotate clockwise
    {  
      if((accelerometer.getGyroZ() < -10) || ((errorCalc(pos_deg, target)>(-20)) && (accelerometer.getGyroZ() < 0))){
        Serial.println("not CW");
        Serial1.println("not CW");
        //stopAll();
        fire_setting = SolenoidSetting::OFF;
        dont_rotate = true;
        return;
      
      }
      else{
        Serial.println("FiredCW");
        Serial1.println("FiredCW");
        
        //fireCW();
        fire_setting = SolenoidSetting::CW;
        rotate_cw = true;
        firing_timer.reset();
        return;
      }

      //Serial.print("Orientation: ");
      //Serial.println(accelerometer.getOrientationX());
      Serial1.println("-------------------------------CLOCKWISE");
      Serial1.println("-------------------------------CLOCKWISE");
    }
    else if (errorCalc(pos_deg, target)>(tolerance_deg)) //Rotate counter clockwise
    {
      if((accelerometer.getGyroZ() > 10) || ((errorCalc(pos_deg, target)>(20)) && (accelerometer.getGyroZ() > 0))){
        Serial1.println("not CCW");
        Serial1.println("not CCW");
        //stopAll();
        fire_setting = SolenoidSetting::OFF;
        dont_rotate = true;
        return;
      }
      else{
        Serial1.println("FiredCCW");
        Serial1.println("FiredCCW");
        //fireCCW();
        fire_setting = SolenoidSetting::CCW;
        rotate_ccw = true;
        firing_timer.reset();
        return;
      }

      Serial1.println("-------------------------------COUNTER CLOCKWISE");
    }
    else //Do not rotate
    {
      digitalWrite(CW, LOW);
      digitalWrite(CCW, LOW);
      
      Serial1.println("-------------------------------ALIGNED");
    }
    // firing_timer.reset();
    if (firing_timer.isComplete()){
      digitalWrite(CW, LOW);
      digitalWrite(CCW, LOW);
      solenoid_timer.reset();
      firing_timer.reset();
    }
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

    if (rotate_cw && rotate_ccw)
    {
      Serial.println("Rotating Clockwise AND Counterclockwise");
    }
    else if (rotate_cw && dont_rotate)
    {
      Serial.println("Rotating Clockwise AND Not Rotating");
    }
    else if (rotate_ccw && dont_rotate)
    {
      Serial.println("Rotating Counterclockwise AND Not Rotating");
    }
    
  }
  else //Do not rotate
  {
    Serial1.println("_________________________________________________________________TIMER NOT COMPLETE");
    Serial1.println("solenoid_timer");
    Serial1.println(solenoid_timer.timeRemaining());
    digitalWrite(CW, LOW);
    digitalWrite(CCW, LOW);
  }
}


double errorCalc(double current, double target){
  return ((((int)(current - target)+ 540) % 360)-180);
}

bool posVelocity(){
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

bool negVelocity(int time){
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
  stage = FlightStage::STABILIZE;
}

void loop() 
{
  
  bme.prefetchData();
  accelerometer.prefetchData();
  gps.prefetchData();

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
      
      //was_above_stabilization = altitude > kSTABILIZATION_ALTITUDE;

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

      
      fireSolenoidsByBB(accelerometer.getOrientationX()); //TODO: Tune / stabilize
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

  //Control firing based off of enum setting
  switch (fire_setting)
  {
    case SolenoidSetting::CW:
      fireCW();
      break;
    case SolenoidSetting::CCW:
      fireCCW();
      break;
    case SolenoidSetting::OFF:
      stopAll();
      break;
    default:
      break;
  }
}