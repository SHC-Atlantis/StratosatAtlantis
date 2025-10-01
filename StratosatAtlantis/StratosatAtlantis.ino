#include <SD.h>
#include <SPI.h>

float velocity;
float yaw;

class PDCycle
{
  float m_kP, m_kD, m_error, m_last_error, m_time_ms, m_last_time_ms, m_ang_velocity;

  float m_p_output, m_d_output;

  public:

  inline float getAngularPos()
  {
    return 0.0;
  }

  inline float getTimeStamp()
  {
    return 0.0;
  }

  inline PDCycle(float kP, float kD)
  : m_kP(kP)
  , m_kD(kD)
  , m_error(0.0)
  , m_last_error(0.0)
  , m_time_ms(getTimeStamp())
  , m_last_time_ms(0.0)
  {}

  inline float calculate()
  {

  }

}

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
