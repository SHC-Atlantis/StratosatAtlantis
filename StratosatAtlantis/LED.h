class LED
{
	long m_delay_ms, m_length_ms, m_length_time, m_delay_time;

	int m_pin;

	bool m_delay, m_blink, m_on;

public:

/*
* @param LED_pin: The pin number of the LED that Arduino will recognize.
* @param length_ms: How long to blink the LED, in milliseconds.
* @param delay_ms: How long to wait until the next blink, in milliseconds.
*/
	LED(int LED_pin, long length_ms = 1000.0, long delay_ms = 1000.0);

	/*
	  Updates the internal clock and the state of blinking

	  @param time_ms: new timestamp for the internal clock
	*/
	int Update(long time_ms);

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