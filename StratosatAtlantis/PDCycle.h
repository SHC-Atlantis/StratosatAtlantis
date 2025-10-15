#include <math.h>

class PDCycle
{
  float m_kP, m_kD, m_error, m_last_error, m_pos_deg, m_target_pos_deg, m_p_output, m_d_output, m_tolerance;

  unsigned long m_time_ms, m_last_time_ms;

  public:

  PDCycle(float target_deg, float current_deg, float kP, float kD, float tolerance = 0.05);

  float calculate(unsigned long time_ms);

};