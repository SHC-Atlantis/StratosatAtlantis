#include <SD.h>
#include <SPI.h>

float velocity;
float yaw;



enum class FlightStage
{
  LAUNCH,
  ASCENT,
  STABILIZE,
  DESCENT,
  LANDED
}

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

void setup() {
  

}

void loop() {

  loop.calculate();

}
