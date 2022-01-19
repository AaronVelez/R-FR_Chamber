bool SetLights(unsigned int Period_ON_t, unsigned int Period_OFF_t) {
	int DayMinute = (h * 60) + m;
	if (Period_ON_t < Period_OFF_t) {
		// OFF time happens before midnight
		if (DayMinute > Period_ON_t && DayMinute < Period_OFF_t) {
			// Trun ON
			return true;
		}
		else {
			// Turn OFF
			return false;
		}
	}
	else if (Period_ON_t > Period_OFF_t) {
		// OFF time happens after midnight
		if (DayMinute > Period_ON_t || DayMinute < Period_OFF_t) {
			// Turn ON
			return true;
		}
		else {
			// Turn OFF
			return false;
		}
	}
	else {
		// Turn OFF, ON and OFF time are the same
		return false;
	}
}