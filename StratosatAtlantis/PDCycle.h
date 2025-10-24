#include <math.h>

class PDCycle
{
  enum class OutputType
  {
    Percent,
    Direct
  };

  OutputType m_output_type;

  float m_kP, m_kD, m_error, m_last_error, m_pos_deg, m_target_pos_deg, m_p_output, m_d_output, m_tolerance;

  unsigned long m_time_ms, m_last_time_ms;

  public:

  PDCycle(float target_deg, float current_deg, OutputType output_type = OutputType::Direct, float kP = 1.0, float kD = 1.0, float tolerance = 0.05);

  float calculate(unsigned long time_ms);

  inline void setKP(float kP) { m_kP = kP; }
  inline void setKD(float kD) { m_kD = kD; }
  inline void setOutputType(OutputType output_type) { m_output_type = output_type; }

};