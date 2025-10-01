#include "PDCycle.h"

float PDCycle::getAngularPos()
{
  return 0.0;
}

float PDCycle::getTimeStamp()
{
  return 0.0;
}

PDCycle::PDCycle(float target_deg, float kP, float kD)
  : m_kP(kP)
  , m_kD(kD)
  , m_error(0.0)
  , m_last_error(0.0)
  , m_time_ms(getTimeStamp())
  , m_last_time_ms(0.0)
  , m_pos_deg(getAngularPos())
  , m_target_pos_deg(target_deg)
  {}

float PDCycle::calculate()
{
  m_error = m_pos - m_target_pos;

  m_p_output = m_error * m_kP; //Proportional calculation

  m_d_output = ((m_error - m_last_error) / (m_time_ms - m_last_time_ms)) * m_kD; //Derivative calculation

  //Setup variables for next cycle
  m_last_error = m_error;
  m_last_time_ms = m_time_ms;

  return m_p_output + m_d_output //P + D

}