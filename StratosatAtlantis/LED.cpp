#include "LED.h"
#include <arduino.h>

LED::LED(int LED_pin, unsigned long length_ms, unsigned long delay_ms)
	: m_pin(LED_pin)
	, m_length_ms((length_ms == 0UL) ? 1UL : length_ms)
	, m_delay_ms((delay_ms == 0UL) ? 1UL : delay_ms)
	// , m_length_time(0.0)
	// , m_delay_time(0.0)
	// , m_delay(false)
	// , m_blink(true)
    , m_phase_start(0UL)
    , m_phase_on(true)
	, m_on(true)
{}

LED::LED()
{}

int LED::update(unsigned long time_ms)
{
	// int return_value = 0; //Equivalent to LOW arduino digitalWrite enum

	// //If the blink state is true, and the current time is within the blinking length time frame, turn on LED and stop delay
	// if (m_on && m_blink && (time_ms - m_length_time) < (m_length_time + m_length_ms))
	// {
	// 	return_value = 1; //Equivalent to HIGH arduino digitalWrite enum
	// 	m_delay = false;
	// }
	// else
	// {
	// 	m_delay = true;
	// 	m_delay_time = time_ms; //If not true, start the delay
	// }

	// //If the blink state is false, and the current time is within the delay time frame, turn off LED and stop blink
	// if (m_on && m_delay && (time_ms - m_delay_time) < (m_delay_time + m_delay_ms))
	// {
	// 	return_value = 0;
	// 	m_blink = false;
	// }
	// else
	// {
	// 	m_blink = true;
	// 	m_length_time = time_ms; //If not true, start the blink
	// }

	// return return_value; 

    if (!m_on) 
			return 0;

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

  return m_phase_on ? 1 : 0;
}