class LED
{
	//unsigned long m_delay_ms, m_length_ms, m_length_time, m_delay_time;
    unsigned long m_delay_ms, m_length_ms, m_phase_start;

	int m_pin;

	//bool m_delay, m_blink, m_on;
    bool m_on, m_phase_on;

public:

/*
* @param LED_pin: The pin number of the LED that Arduino will recognize.
* @param length_ms: How long to blink the LED, in milliseconds.
* @param delay_ms: How long to wait until the next blink, in milliseconds.
*/
	LED(int LED_pin, unsigned long length_ms = 500UL, unsigned long delay_ms = 500UL);

	LED();

	/*
	  Updates the internal clock and the state of blinking. Returns a HIGH/LOW value for digitalWrite.

	  @param time_ms: new timestamp for the internal clock
	*/
	int update(unsigned long time_ms);

	inline void TurnOn() { m_on = true; }
	inline void TurnOff() { m_on = false; }
	inline void FlipOnOff() { m_on = !m_on; }

	/*
	* @param length_ms: How long to blink the LED, in milliseconds.
	*/
	inline void SetLength(const long length_ms) { m_length_ms = length_ms; }

	/*
	* @param delay_ms: How long to wait until the next blink, in milliseconds.
	*/
	inline void SetDelay(const long delay_ms) { m_delay_ms = delay_ms; }

};