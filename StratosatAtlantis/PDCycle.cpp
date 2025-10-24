#include "PDCycle.h"


PDCycle::PDCycle(float target_deg, float current_deg, float kP, float kD, float tolerance)
  : m_kP(kP)
  , m_kD(kD)
  , m_tolerance(tolerance)
  , m_error(0.0)
  , m_last_error(0.0)
  , m_time_ms(0UL)
  , m_last_time_ms(0.0)
  , m_pos_deg(current_deg)
  , m_target_pos_deg(target_deg)
  {}

float PDCycle::calculate(unsigned long time_ms)
{
  m_time_ms = time_ms;

  m_error = m_pos_deg - m_target_pos_deg;

  m_p_output = m_error * m_kP; //Proportional calculation

  m_d_output = ((m_error - m_last_error) / (m_time_ms - m_last_time_ms)) * m_kD; //Derivative calculation

  //Setup variables for next cycle
  m_last_error = m_error;
  m_last_time_ms = m_time_ms;

  return (fabs(m_p_output + m_d_output) >= m_tolerance ) ?  m_p_output + m_d_output : 0.0; //P + D, or 0 if in tolerance

}