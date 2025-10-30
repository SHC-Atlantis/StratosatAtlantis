#include "LED.h"
#include <arduino.h>

LED::LED(int LED_pin, unsigned long length_ms, unsigned long delay_ms)
	: m_pin(LED_pin)
	, m_length_ms((length_ms == 0UL) ? 1UL : length_ms)
	, m_delay_ms((delay_ms == 0UL) ? 1UL : delay_ms)
    , m_phase_start(0UL)
    , m_phase_on(true)
	, m_on(true)
{}

LED::LED()
{}

bool LED::update(unsigned long time_ms)
{
  if (!m_on) 
		return LOW;

  // Determine current interval based on phase
  const unsigned long interval = m_phase_on ? m_length_ms : m_delay_ms;

  // If first call, start the clock
  if (m_phase_start == 0UL) {
    m_phase_start = time_ms;
  }

  // Has the current phase’s interval elapsed?
  if ((time_ms - m_phase_start) >= interval) {
    // Flip phase and restart timer
    m_phase_on = !m_phase_on;
    m_phase_start = time_ms;
  }

  return m_phase_on;
}
