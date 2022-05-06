////// Get time from NTP an dupdate RTC and local time variables
bool GetNTPTime() {
    if (timeClient.forceUpdate()) {
        UTC_t = timeClient.getEpochTime();
        // set system time to UTC unix timestamp
        setTime(UTC_t);
        // Set RTC time to UTC time from system time
        rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
        // Convert to local time
        local_t = mxCT.toLocal(UTC_t);
        // Set system time lo local time
        setTime(local_t);
        LastNTP = UTC_t;
        if (debug) {
            Serial.println(F("NTP client update success!"));
            Serial.print(F("UTC time from NTP is: "));
            Serial.println( (unsigned long)UTC_t );
        }
        return true;
    }
    else {
        if (debug) { Serial.println(F("NTP update not succesfull")); }
        return false;
    }
}


////// Get time from RTC and update UTC and local time variables
void GetRTCTime() {
    RTCnow = rtc.now();                 // Get UTC time from RTC in DateTime format
    UTC_t = RTCnow.unixtime();            // Convert UTC time to UNIX format
    local_t = mxCT.toLocal(UTC_t);      // Calculate local time in UNIX format
    setTime(local_t);                   // Set system time to local
    s = second();                       // Set time variables to local time from system time
    m = minute();
    h = hour();
    dy = day();
    mo = month();
    yr = year();
    delay(250); // Wait to I2C device to release bus. It helps to prevent I2C bus getting stuck in noisy line
}