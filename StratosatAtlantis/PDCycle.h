class PDCycle
{
  float m_kP, m_kD, m_error, m_last_error, m_time_ms, m_last_time_ms, m_pos_deg, m_target_pos_deg;

  float m_p_output, m_d_output;

  public:

  float getAngularPos();

  float getTimeStamp();

  PDCycle(float target_deg, float kP, float kD)

  float calculate();

}