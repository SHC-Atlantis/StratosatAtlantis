#include <Utils.h>
#include <SHC_BME280.h>
#include <SHC_BNO055.h>
#include <SHC_M9N.h>
#include <string>
#include <math.h>

#include "PDCycle.h"
#include "LED.h"

#include "ICP201xx.h"

enum class FlightStage //determined by state machiene
{
  LAUNCH,
  ASCENT,
  STABILIZE,
  DESCENT,
  LANDED
};

//The front has the camera

const int kMAIN_LED_1 = 2;  //Left LED
const int kMAIN_LED_2 = 5;  //Right LED
const int kLF_SOLENOID = 4; //Left-Front Solenoid pin number CWW
const int kLB_SOLENOID = 3; //Left-Back Solenoid pin number CC
const int kRF_SOLENOID = 3; //Right-Front Solenoid pin number CC
const int kRB_SOLENOID = 4; //Right-Back Solenoid pin number CWW

const int CCW = 4; 
const int CW = 3;

const float kSTABILIZATION_ALTITUDE = 18000.0; //height required to begin stabilization, TODO:change before flight

//init vars
float altitude = 0; 
float lastAlt = 0;
int count = 0;
float beginAlt = 0;
float currentAlt = 0;

float pressure_kP = 0;
float temperature_C = 0;

String FStage = "INIT"; // initial flight stage, should not be printed out to SD
FlightStage stage;

SHC_BME280 bme;
BNO055 accelerometer;
M9N gps;

ICP201xx ICP(Wire,0);

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

// collect data and write to MicroSD via open log
void collectData()
{
  ICP.getData(pressure_kP,temperature_C);
  //Serial.println("Stage, UNIX, Year,Month,Day,Hour,Min,Sec,AccX(m/s^2),AccY(m/s^2),AccZ(m/s^2),Yaw(degrees),Roll(degrees),Pitch(degrees),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),Humidity(%rh),Pressure(mb),Temperature(C),Altitude(m),ICP-Pressure(kP),ICP-Temperature(C),Lat,Long,SIV,");
  //Stage,Year,Month,Day,Time,Minute,Second,AccX,AccY,AccZ,OrientX,OrientY,OrientZ,GyroX,GyroY,GyroZ,Humidity,Pressure,Temperature,Altitude,Pressure,Temperature,Altitude,Latitude,Longitude,SIV
  
  String data = ""+
      String(FStage) + "," +
      String(gps.getUnixTime()) + "," +
      String(gps.getYear()) + "," +
      String(gps.getMonth()) + "," +
      String(gps.getDay()) + "," +
      String(gps.getHour()) + "," +
      String(gps.getMinute()) + "," +
      String(gps.getSecond()) + "," +
      String(accelerometer.getAccelerationX(),3) + "," + 
      String(accelerometer.getAccelerationY(),3) + "," +
      String(accelerometer.getAccelerationZ(),3) + "," +
      String(accelerometer.getOrientationX(),3) + "," +
      String(accelerometer.getOrientationY(),3) + "," + 
      String(accelerometer.getOrientationZ(),3) + "," +
      String(accelerometer.getGyroX(),3) + "," +
      String(accelerometer.getGyroY(),3) + "," +
      String(accelerometer.getGyroZ(),3) + "," +
      String(bme.getHumidity(),3) + "," +
      String(bme.getPressure(),3) + "," +
      String(bme.getTemperature(),3) + "," +
      String(gps.getAltitude(),3) + "," +
      String(pressure_kP,3) + "," +
      String(temperature_C,3) + "," +
      String(gps.getLatitude(), 7) + "," +
      String(gps.getLongitude(), 7)  + "," +
      String(gps.getSIV()) + ","
      ;
      
    Serial1.println(data);
    Serial.println(data);
} 
*/

/*Fires solenoids based upon the angular position relative to the target position via BangBang

// fire solenoids using modified bang-bang
void fireSolenoidsByBB(float pos_deg, float tolerance_deg = 10)
{
  float target = 45.0;

  // delay(50); //TODO: delete before flight, for testing only
  
  /* Debugging Print Statements
  Serial.print("error cal: ");
  Serial.println(errorCalc(pos_deg, target));
  Serial.print("orientation: ");
  Serial.println(accelerometer.getOrientationX());
  Serial.print("Solenoid Timer: ");
  Serial.println(solenoid_timer.timeRemaining());
  Serial.print("Firing timer: ");
  Serial.println(firing_timer.timeRemaining());
  */
  
  if (solenoid_timer.isComplete()){

    Serial.println("SOL TIMER COMPLETE");

    if(velocitycap_timer.isComplete()){
      stopAll();
    }

    if(firing_timer.isComplete()){
      stopAll();
    }

    if(accelerometer.getGyroZ() > 60){ //too fast CCW
      fireCW();
      velocitycap_timer.reset();
      Serial.println("over 60");
      return;
    }

    else if(accelerometer.getGyroZ() < -60){ //too fast CW
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

      Serial.println("CLOCKWISE");
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
      Serial.println("COUNTER CLOCKWISE");
    }
    
    else //Do not rotate
    {
      stopAll();


      Serial.println("ALIGNED");
    }
    
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

double errorCalc(double current, double target){  
  return ((((int)(current - target)+ 540) % 360)-180); //error calculation formula given by mentor Drew but we were really close to getting it on our own
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
    //post flight note: missing a count=0 here
    return true;
  }
  return false;
}

void ledBlink(){  // blink for 1/20th of a sec at 1hz
  if(blinky_off_timer.isComplete()){
      blinky_on_timer.reset(); 
      digitalWrite(kMAIN_LED_1, HIGH); //high is on
      digitalWrite(kMAIN_LED_2, HIGH);
      Serial.print("ON");
      blinky_off_timer.reset();
  }
  else if(blinky_on_timer.isComplete()){
      digitalWrite(kMAIN_LED_1, LOW); //low is off
      digitalWrite(kMAIN_LED_2, LOW);
      Serial.print("OFF");
    }
  }

void setup() 
{
  Serial.begin(9600); //Serial is the USB output
  Serial1.begin(115200); //Serial1 is the openLog MicroSD card output (for flight day)

  //csv file header
  Serial1.println("Stage,UNIX,Year,Month,Day,Hour,Min,Sec,AccX(m/s^2),AccY(m/s^2),AccZ(m/s^2),Yaw(degrees),Roll(degrees),Pitch(degrees),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),Humidity(%rh),Pressure(mb),Temperature(C),Altitude(m),ICP-Pressure(kP),ICP-Temperature(C),Lat,Long,SIV,");

  int error_amount = 0;
  int error_stored;

  stage = FlightStage::LAUNCH; //initalize to launch stage

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
  
  //stage = FlightStage::STABILIZE; //TODO: not for flight, just for testing stabilization
}

void loop() 
{
  altitude = gps.getAltitude();
  ledBlink(); //TODO: uncomment, commented for testing so I dont go blind

  /* testing for state machiene
  if(altitude < 30000 && topReached == 0){
    altitude += 16;
  }
  else if(altitude >= 30000){
    topReached = 1;
  }
  
  if(topReached == 1){
    altitude -= 16;
  }

  if (altitude > 0){
    altitude -= 16;
  }
  else{
    altitude = 0;
  }
  */
  
  bme.prefetchData();
  accelerometer.prefetchData();
  gps.prefetchData();

  //flight stage switch case destermined by state machiene
  switch (stage)
  {
    case FlightStage::LAUNCH:
      FStage = "LAUNCH";
      //collectData(); //should we collect data here? at 50x a second it seems like a lot
      //post flight day note- it wouldnt hurt, could have been good

      if (posVelocity() == true){ //once pos velocity is achieved we can assume we are in the sky
        stage = FlightStage::ASCENT;
      }
    break;

    case FlightStage::ASCENT:
      FStage = "ASCENT";
      
      collectData();

      if (altitude < kSTABILIZATION_ALTITUDE) // less than stabilization alt
      {
        Serial.println("-----------less than alt to stabilize");
        ascent_timer.reset();
        break;
      }
        
      else if(ascent_timer.isComplete() && (kSTABILIZATION_ALTITUDE < altitude)) //at stabilization alt and had pos vel for 30s
      {
        Serial.println("----------- stabilize");
        stage = FlightStage::STABILIZE;
      }
      
      was_above_stabilization = altitude > kSTABILIZATION_ALTITUDE;

      else if (negVelocity(10)) //oopsie the balloon popped early, no stabilization for you
      {
        Serial.println("loosing alt");
        stage = FlightStage::DESCENT;
      }

      else{ //loop back thru again
        Serial.println("else");
      }
      Serial.println(ascent_timer.timeRemaining());
    break;

    case FlightStage::STABILIZE: //yippie my favorite stage :D
      FStage = "STABILIZE";
      
      collectData();

      if (negVelocity(30)) //falling for 30 seconds, in descent
        {
          stage = FlightStage::DESCENT;
        }
      //post flight note: we never had a negative velocity for 30 seconds, this didnt work
      
      fireSolenoidsByBB(accelerometer.getOrientationX()); //STABILIZATION!
      if(solenoid_timer.isComplete()){
        solenoid_timer.reset();
        firing_timer.reset();
      }
    break;

    case FlightStage::DESCENT:
      FStage = "DESCENT";

      collectData();

      if(novelocity_timer.isComplete() && abs((beginAlt - altitude)) > 10)//we had >10m altitude change, must still be falling
      {
        novelocity_timer.reset();
        beginAlt = altitude;
      }
      
      else if (novelocity_timer.isComplete() && abs((beginAlt - altitude)) < 10) //less than 10m altitude movement in 30s
      {
        stage = FlightStage::LANDED;
      }
    break;

    case FlightStage::LANDED:
      FStage = "LANDED";
      if (collection_timer.isComplete()) //slow down data collection rate on ground
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