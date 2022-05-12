////// Get time from NTP an dupdate RTC and local time variables
bool GetNTPTime() {
    if (timeClient.forceUpdate()) {
        UTC_t = timeClient.getEpochTime();
        // set system time to UTC unix timestamp
        setTime(UTC_t);
        // Set RTC time to UTC time from system time
        RTCDate.Year = year();
        RTCDate.Month = month();
        RTCDate.Date = day();
        RTCtime.Hours = hour();
        RTCtime.Minutes = minute();
        RTCtime.Seconds = second();
        M5.Rtc.SetTime(&RTCtime);
        M5.Rtc.SetDate(&RTCDate);
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
// For M5 BM8563 RTC
void GetRTCTime() {
    M5.Rtc.GetTime(&RTCtime);   // Get UTC time from M5 RTC.
    M5.Rtc.GetDate(&RTCDate);   // Get UC date from M5 RTC

    setTime(RTCtime.Hours,                   // Set system time to UTC time
        RTCtime.Minutes,
        RTCtime.Seconds,
        RTCDate.Date,
        RTCDate.Month,
        RTCDate.Year);
    UTC_t = now();                      // Get UTC time from system time in UNIX format

    
    local_t = mxCT.toLocal(UTC_t);      // Calculate local time in UNIX format
    setTime(local_t);                   // Set system time to local
    s = second();                       // Set time variables to local time from system time
    m = minute();
    h = hour();
    dy = day();
    mo = month();
    yr = year();
}