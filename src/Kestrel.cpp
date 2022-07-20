/******************************************************************************
Kestrel
Interface for Kestrel data logger
Bobby Schulz @ GEMS Sensing
6/21/2022
https://github.com/gemsiot/Driver_-_Kestrel

Allows control of all elements of the Kestrel data logger, including IO interfacing and self diagnostics 

0.0.0

///////////////////////////////////////////////////////////////////FILL QUOTE////////////////////////////////////////////////////////////////////////////////

Distributed as-is; no warranty is given.
******************************************************************************/

#include <Kestrel.h>

Kestrel* Kestrel::selfPointer;

Kestrel::Kestrel() : ioOB(0x20), ioTalon(0x21), led(0x52), csaAlpha(2, 2, 2, 2, 0x18), csaBeta(2, 10, 10, 10, 0x14)
{
	// port = talonPort; //Copy to local
	// version = hardwareVersion; //Copy to local
}

String Kestrel::begin(time_t time, bool &criticalFault, bool &fault)
{
    selfPointer = this;
    System.on(time_changed, timechange_handler);
    #if defined(ARDUINO) && ARDUINO >= 100 
		Wire.begin();
        Wire.setClock(400000); //Confirm operation in fast mode
	#elif defined(PARTICLE)
		if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
	#endif

    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
    if(ioOB.begin() != 0) criticalFault = true;
    if(ioTalon.begin() != 0) criticalFault = true;
    enableAuxPower(true); //Turn on aux power 
    csaAlpha.begin();
    csaBeta.begin();
    // delay(100); //DEBUG! For GPS
    ioOB.pinMode(PinsOB::LED_EN, OUTPUT);
	ioOB.digitalWrite(PinsOB::LED_EN, LOW); //Turn on LED indicators 
    led.begin();
    led.setOutputMode(OpenDrain); //Set device to use open drain outputs
    led.setGroupMode(Blink); //Set system to blinking mode
    led.setOutputArray(Off); //Turn all off by default
    led.setBrightnessArray(ledBrightness); //Set all LEDs to 50% max brightness
	// led.setGroupBrightness(ledBrightness); //Set to 50% brightness
	led.setGroupBlinkPeriod(ledPeriod); //Set blink period to specified number of ms
	led.setGroupOnTime(ledOnTime); //Set on time for each blinking period 
    // setIndicatorState(IndicatorLight::ALL,IndicatorMode::WAITING); //Set all to blinking wait
    
    if(rtc.begin(true) == 0) criticalFault = true; //Use with external oscilator, set critical fault if not able to connect 
    if(gps.begin() == false) {
        criticalFault = true; //DEBUG! ??
        //Throw ERROR!
        Serial.println("GPS ERROR");
    }
    else {
        gps.setI2COutput(COM_TYPE_UBX);
    }
    // if(Particle.connected() == false) criticalFault = true; //If not connected to cell set critical error
    if(criticalFault) setIndicatorState(IndicatorLight::STAT, IndicatorMode::ERROR); //If there is a critical fault, set the stat light
    for(int i = 1; i <= 4; i++) {
        enablePower(i, false); //Default all power to off
        enableData(i, false); //Default all data to off
    }
    
    syncTime(); 
    // Particle.syncTime(); //DEBUG!
    // ioOB.pinMode(PinsOB::)
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    return "{}"; //DEBUG!
}

String Kestrel::getErrors()
{
    // uint32_t temp[10] = {0}; //Temp array to store error vals read in from drivers

    String output = "{\"KESTREL\":{"; // OPEN JSON BLOB
	output = output + "\"CODES\":["; //Open codes pair

	for(int i = 0; i < min(MAX_NUM_ERRORS, numErrors); i++) { //Interate over used element of array without exceeding bounds
		output = output + "\"0x" + String(errors[i], HEX) + "\","; //Add each error code
		errors[i] = 0; //Clear errors as they are read
	}
    for(int i = 0; i < min(sizeof(rtc.errors), rtc.numErrors); i++) { //Interate rtc errors
		output = output + "\"0x" + String(rtc.errors[i], HEX) + "\","; //Add each error code
		rtc.errors[i] = 0; //Clear errors as they are read
	}
	if(output.substring(output.length() - 1).equals(",")) {
		output = output.substring(0, output.length() - 1); //Trim trailing ','
	}
	output = output + "],"; //close codes pair
	output =  output + "\"OW\":"; //Open state pair
	if(numErrors > MAX_NUM_ERRORS || rtc.numErrors > sizeof(rtc.errors)) output = output + "1,"; //If overwritten, indicate the overwrite is true
	else output = output + "0,"; //Otherwise set it as clear
	output = output + "\"NUM\":" + String(numErrors + rtc.numErrors); //Append number of errors
	// output = output + "\"Pos\":[" + String(talonPort + 1) + "," + String(sensorPort + 1) + "]"; //Concatonate position 
	output = output + "}}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
    rtc.numErrors = 0; 
	return output;
}

String Kestrel::getData(time_t time)
{
    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
    enableAuxPower(true); //Turn on aux power 
    if(gps.getFixType() >= 2) { //Only update if GPS has at least a 2D fix
        longitude = gps.getLongitude();
        latitude = gps.getLatitude();
        posTime = getTime(); //Update time that GPS measure was made
    }
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    return "{\"Kestrel\":null}";
}

bool Kestrel::enablePower(uint8_t port, bool state) 
{
    //FIX! Throw error is port out of range
    if(port == 5) { //Port for (ext/batter port) is special case
        // return enableAuxPower(state); 
        return false; //DEBUG!
    }
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_ERROR | portErrorCode);
    else {
        bool obState = enableI2C_OB(true);
        bool globState = enableI2C_Global(false);
        // Wire.reset(); //DEBUG!
        ioTalon.pinMode(PinsTalon::EN[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::EN[port - 1], state);
        enableI2C_Global(globState); //Return to previous state
        enableI2C_OB(obState);
    }
    
    return false; //DEBUG!
}

bool Kestrel::enableData(uint8_t port, bool state)
{
    //FIX! Throw error is port out of range
    if(port == 5) { //Port for (ext/batter port) is special case
        // if(state) enableI2C_Global(true); //Turn on global
        enableI2C_External(state); 
        return false; //DEBUG!
    }
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_ERROR | portErrorCode);
    else {
        bool obState = enableI2C_OB(true);
        bool globState = enableI2C_Global(false);
        // Wire.reset(); //DEBUG!
        // ioTalon.pinMode(PinsTalon::SEL[port - 1], OUTPUT);
        // ioTalon.digitalWrite(PinsTalon::SEL[port - 1], LOW); //DEBUG!
        ioTalon.pinMode(PinsTalon::I2C_EN[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::I2C_EN[port - 1], state);
        // Serial.println(PinsTalon::I2C_EN[port - 1]); //DEBUG!
        enableI2C_Global(globState); //Return to previous state
        enableI2C_OB(obState);
    }
    
    return false; //DEBUG!
}

bool Kestrel::setDirection(uint8_t port, bool sel)
{
    if(port == 5) { //Port for (ext/batter port) is special case
        // if(state) enableI2C_Global(true); //Turn on global
        // enableI2C_External(state); 
        return false; //DEBUG!
    }
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_ERROR | portErrorCode);
    else {
        bool obState = enableI2C_OB(true);
        bool globState = enableI2C_Global(false);
        // Wire.reset(); //DEBUG!
        ioTalon.pinMode(PinsTalon::SEL[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::SEL[port - 1], sel); //DEBUG!
        // ioTalon.pinMode(PinsTalon::I2C_EN[port - 1], OUTPUT);
        // ioTalon.digitalWrite(PinsTalon::I2C_EN[port - 1], state);
        // Serial.println(PinsTalon::I2C_EN[port - 1]); //DEBUG!
        enableI2C_Global(globState); //Return to previous state
        enableI2C_OB(obState);
    }
    
    return false; //DEBUG!
}

bool Kestrel::enableI2C_OB(bool state)
{
    bool currentState = digitalRead(Pins::I2C_OB_EN); 
    pinMode(Pins::I2C_OB_EN, OUTPUT);
	digitalWrite(Pins::I2C_OB_EN, state);
    // Wire.reset(); //DEBUG!
    return currentState; 
}

bool Kestrel::enableI2C_Global(bool state)
{
    bool currentState = digitalRead(Pins::I2C_GLOBAL_EN); 
    pinMode(Pins::I2C_GLOBAL_EN, OUTPUT);
	digitalWrite(Pins::I2C_GLOBAL_EN, state);
    // Wire.reset(); //DEBUG!
    return currentState; 
}

bool Kestrel::enableI2C_External(bool state)
{
    bool globState = enableI2C_Global(false);
	bool obState = enableI2C_OB(true);
	//Turn on external I2C port
    bool currentState = ioOB.digitalRead(PinsOB::I2C_EXT_EN);
	ioOB.pinMode(PinsOB::I2C_EXT_EN, OUTPUT);
	ioOB.digitalWrite(PinsOB::I2C_EXT_EN, state);
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    return currentState; //DEBUG! How to return failure? Don't both and just throw error??
}

bool Kestrel::disablePowerAll()
{
    for(int i = 1; i <= 5; i++) {
        enablePower(i, false);
    }
    return false; //DEBUG!
}

bool Kestrel::disableDataAll()
{
    for(int i = 1; i <= 5; i++) {
        enableData(i, false);
    }
    return 0; //DEBUG!
}

// int Kestrel::throwError(uint32_t error)
// {
// 	errors[(numErrors++) % MAX_NUM_ERRORS] = error; //Write error to the specified location in the error array
// 	if(numErrors > MAX_NUM_ERRORS) errorOverwrite = true; //Set flag if looping over previous errors 
// 	return numErrors;
// }

bool Kestrel::enableSD(bool state)
{
    bool globState = enableI2C_Global(false);
	bool obState = enableI2C_OB(true);
    bool currentState = ioOB.digitalRead(PinsOB::SD_EN);
    if(state) {
        enableAuxPower(true); //Make sure Aux power is on
        ioOB.pinMode(PinsOB::SD_EN, OUTPUT);
        ioOB.digitalWrite(PinsOB::SD_EN, HIGH);
    }
    else if(!state) {
        ioOB.pinMode(PinsOB::SD_EN, OUTPUT);
        ioOB.digitalWrite(PinsOB::SD_EN, LOW);
    }
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    return currentState; //DEBUG! How to return failure? Don't both and just throw error??
}

bool Kestrel::enableAuxPower(bool state)
{
    bool globState = enableI2C_Global(false);
	bool obState = enableI2C_OB(true);
    bool currentState = ioOB.digitalRead(PinsOB::AUX_EN);
    ioOB.pinMode(PinsOB::AUX_EN, OUTPUT);
    ioOB.digitalWrite(PinsOB::AUX_EN, state); 
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    return currentState; //DEBUG! How to return failure? Don't both and just throw error??
}

uint8_t Kestrel::updateTime()
{
    // Serial.print("Current Timer Period: "); //DEBUG!
    // Serial.println(logPeriod);
    static uint8_t timeSource = syncTime();
    static time_t lastRunTime = millis();

    if((millis() - lastRunTime) > 60000) { //Only sync time if it has been more than 60 seconds since last synchronization
        timeSource = syncTime(); 
        lastRunTime = millis();
    }
    // if(Time.isValid()) { //If particle time is valid, set from this
    currentDateTime.source = timeSource; //sync time and record source of current time
    currentDateTime.year = Time.year();
    currentDateTime.month = Time.month();
    currentDateTime.day = Time.day();
    currentDateTime.hour = Time.hour();
    currentDateTime.minute = Time.minute();
    currentDateTime.second = Time.second();
    
    return currentDateTime.source; //debug!
    // }
    // else { //If time is not valid, write to default time //FIX??
    //     currentDateTime.year = 2000;
    //     currentDateTime.month = 1;
    //     currentDateTime.day = 1;
    //     currentDateTime.hour = 0;
    //     currentDateTime.minute = 0;
    //     currentDateTime.second = 0;
    //     currentDateTime.source = TimeSource::NONE;
    //     return false;
    // }
    // return false;
}

uint8_t Kestrel::syncTime()
{
    //Synchronize time across GPS, Cell and RTC
    Serial.println("TIME SYNC!"); //DEBUG!
    // Timestamp t = getRawTime(); //Get updated time
    
    time_t gpsTime = 0;
    time_t particleTime = 0;
    time_t rtcTime = 0;
    

    /////////// CELL TIME //////////////////
    if(Particle.connected()) {
        timeSyncRequested = true;
        Particle.syncTime();
        waitFor(Particle.syncTimeDone, 5000); //Wait until sync is done, at most 5 seconds //FIX!
        Time.zone(0); //Set to UTC 
        particleTime = Time.now();
        timeSyncRequested = false; //Release control of time sync override 
        Serial.print("Cell Time: "); 
        Serial.println(particleTime);
    }

    ////////// GPS TIME ///////////////////
    bool currentAux = enableAuxPower(true);
    bool currentGlob = enableI2C_Global(false);
    bool currentOB = enableI2C_OB(true);
    if(gps.getDateValid() && gps.getTimeValid()) {
        struct tm timeinfo = {0}; //Create struct in C++ time land
        timeinfo.tm_year = gps.getYear() - 1900; //Years since 1900
        timeinfo.tm_mon = gps.getMonth() - 1; //Months since january
        timeinfo.tm_mday = gps.getDay();
        timeinfo.tm_hour = gps.getHour();
        timeinfo.tm_min = gps.getMinute();
        timeinfo.tm_sec = gps.getSecond();
        gpsTime = timegm(&timeinfo); //Convert struct to unix time
        Serial.print("GPS Time: ");
        Serial.println(gpsTime); //DEBUG!
    }

	

    /////////// RTC TIME //////////////
    rtcTime = rtc.getTimeUnix();
    Serial.print("RTC Time: ");
    Serial.println(rtcTime); //DEBUG!
    Serial.print("Particle Time: ");
    Serial.println(particleTime);  
    uint8_t source = TimeSource::NONE; //Default to none unless otherwise set
    if(abs(rtcTime - gpsTime) < maxTimeError && abs(rtcTime - particleTime) < maxTimeError && rtcTime != 0 && gpsTime != 0 && particleTime != 0) { //If both updated sources match local time
        Serial.println("CLOCK SOURCE: All match");
        timeGood = true;
        source = TimeSource::CELLULAR; //Report cell as the most comprehensive source
    }

    else if(abs(particleTime - gpsTime) < maxTimeError && gpsTime != 0 && particleTime != 0) { //If both updated variables match 
        Serial.println("CLOCK SOURCE: GPS and Cell match");
        rtc.setTime(Time.year(), Time.month(), Time.day(), Time.hour(), Time.minute(), Time.second()); //Set RTC from Cell
        timeGood = true;
        //Throw error
        source = TimeSource::CELLULAR;
    }
    else if(abs(particleTime - rtcTime) < maxTimeError && rtcTime != 0 && particleTime != 0) { //If cell and rtc agree
        Serial.println("CLOCK SOURCE: Cell and local match");
        //Can we set the GPS time??
        //Throw error
        timeGood = true;
        source = TimeSource::CELLULAR;
    }
    else if(abs(gpsTime - rtcTime) < maxTimeError && gpsTime != 0 && rtcTime != 0) { //If gps and rtc agree
        Serial.println("CLOCK SOURCE: GPS and local match");
        Time.setTime(gpsTime); //Set particle time from GPS time
        //Throw error
        timeGood = true;
        source = TimeSource::GPS;
    }
    else { //No two sources agree, very bad!
        
        if(rtcTime > 1641016800) { //Jan 1, 2022, date seems to be reeasonable //FIX!
            Serial.println("CLOCK SOURCE: Stale RTC"); //DEBUG!
            Time.setTime(rtc.getTimeUnix()); //Set time from RTC   
            timeGood = true;
            source = TimeSource::RTC;
        }
        else {
            Serial.println("CLOCK SOURCE: NONE"); //DEBUG!
            criticalFault = true; //FIX??
            timeGood = false; 
            Time.setTime(946684800); //Set time back to year 2000
            source = TimeSource::NONE;
            //Throw error!
        }
        
    }
    // return false; //DEBUG!
    enableAuxPower(currentAux); //Return all to previous states
    enableI2C_Global(currentGlob);
    enableI2C_OB(currentOB);
    return source;
}

time_t Kestrel::getTime()
{
    if(!Time.isValid() || !timeGood) { //If time has not been synced, do so now
        syncTime();
    }
    if(Time.isValid() && timeGood) { //If time is good, report current value
        return Time.now();
    }
    // return Time.now();
    //THROW ERROR! //FIX!
    return 0; //DEBUG! //If time is not valid, return failure value
}

String Kestrel::getTimeString()
{
    time_t currentTime = getTime();
    if(currentTime == 0) return "null"; //If time is bad, return null value to JSON
    // else return "DUMMY"; //FIX!
    else return String((int)currentTime);
    // else return String(currentTime); //Otherwise return normal string val
}

String Kestrel::getPosLat()
{
    if(latitude != 0) return String(latitude*(10E-8)); //Return in degrees if value is legit
    else return "null"; //Return null if position has not been initalized
}

String Kestrel::getPosLong()
{
    if(longitude != 0) return String(longitude*(10E-8)); //Return in degrees if value if legit
    else return "null"; //Return null if position has not been initalized
}

time_t Kestrel::getPosTime()
{
    return posTime;
}

String Kestrel::getPosTimeString()
{
    if(posTime > 0) return String((int)posTime); //Return in time of last position measurment if legitimate
    else return "null"; //Return null if position has not been initalized
}

bool Kestrel::startTimer(time_t period)
{
    if(period == 0) period = defaultPeriod; //If no period is specified, assign default period 
    bool currentOB = enableI2C_OB(true);
    bool currentGlob = enableI2C_Global(false);
    rtc.setAlarm(period); //Set alarm from current time
    timerStart = millis(); 
    Serial.print("Time Start: "); //DEBUG!
    Serial.println(timerStart);
    logPeriod = period;
    enableI2C_Global(currentGlob);
    enableI2C_OB(currentOB);
    return false; //DEBUG!
}

bool Kestrel::waitUntilTimerDone()
{
    if(logPeriod == 0) return false; //Return if not already setup
    Serial.print("Time Now: "); //DEBUG!
    Serial.println(millis());
    pinMode(Pins::Clock_INT, INPUT);
    while(digitalRead(Pins::Clock_INT) == HIGH && ((millis() - timerStart) < (logPeriod*1000 + 500))){delay(1);} //Wait until either timer has expired or clock interrupt has gone off //DEBUG! Give 500 ms cushion for testing RTC
    if(digitalRead(Pins::Clock_INT) == LOW) return true; //If RTC triggers properly, return true, else return false 
    else return false; 
}

bool Kestrel::statLED(bool state)
{
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);
    if(state) led.setOutput(7, On); //Turn stat on
    else led.setOutput(7, Off); //Turn stat off
    enableI2C_Global(currentGlob); //Reset to previous state
    enableI2C_OB(currentOB); 
    return false; //DEBUG!
}

bool Kestrel::setIndicatorState(uint8_t ledBank, uint8_t mode)
{
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);
    led.setBrightness(3, 25); //Reduce brightness of green LEDs //DEBUG!
    led.setBrightness(5, 25);
    led.setBrightness(1, 25);
    led.setBrightnessArray(ledBrightness); //Set all LEDs to 50% max brightness
	led.setGroupBlinkPeriod(ledPeriod); //Set blink period to specified number of ms
	led.setGroupOnTime(ledOnTime); //Set on time for each blinking period 
    switch(ledBank) {
        case IndicatorLight::SENSORS:
            if(mode == IndicatorMode::PASS) {
                led.setOutput(0, PWM); //Turn green on
                led.setOutput(1, Off); //Turn amber off
                led.setOutput(2, Off); //Turn red off
            }
            if(mode == IndicatorMode::PREPASS) {
                led.setOutput(0, Group); //Turn green on blinking
                led.setOutput(1, Off); //Turn amber off
                led.setOutput(2, Off); //Turn red off
            }
            if(mode == IndicatorMode::WAITING) {
                led.setOutput(0, Off); //Turn green off
                led.setOutput(1, Group); //Blink amber with group
                led.setOutput(2, Off); //Turn red off
            }
            if(mode == IndicatorMode::ERROR) {
                led.setOutput(0, Off); //Turn green off
                led.setOutput(1, PWM); //Turn amber on
                led.setOutput(2, Off); //Turn red on
            }
            if(mode == IndicatorMode::ERROR_CRITICAL) {
                led.setOutput(0, Off); //Turn green off
                led.setOutput(1, Off); //Turn amber off
                led.setOutput(2, PWM); //Turn red on
            }
            break;
        case IndicatorLight::GPS:
            if(mode == IndicatorMode::PASS) {
                led.setOutput(4, Off); //Turn amber off
                led.setOutput(3, PWM); //Turn green on
            }
            if(mode == IndicatorMode::PREPASS) {
                led.setOutput(4, Off); //Turn amber off
                led.setOutput(3, Group); //Blink green with group
            }
            if(mode == IndicatorMode::WAITING) {
                led.setOutput(4, Group); //Blink amber with group
                led.setOutput(3, Off); //Turn green off
            }
            if(mode == IndicatorMode::ERROR) {
                led.setOutput(4, PWM); //Turn amber on
                led.setOutput(3, Off); //Turn green off
            }
            if(mode == IndicatorMode::ERROR_CRITICAL) {
                led.setOutput(4, PWM); //Turn amber on
                led.setOutput(3, Off); //Turn green off
            }
            break;
        case IndicatorLight::CELL:
            if(mode == IndicatorMode::PASS) {
                led.setOutput(6, Off); //Turn amber off
                led.setOutput(5, PWM); //Turn green on
            }
            if(mode == IndicatorMode::PREPASS) {
                led.setOutput(6, Off); //Turn amber off
                led.setOutput(5, Group); //Blink green with group
            }
            if(mode == IndicatorMode::WAITING) {
                led.setOutput(6, Group); //Blink amber with group
                led.setOutput(5, Off); //Turn green off
            }
            if(mode == IndicatorMode::ERROR) {
                led.setOutput(6, PWM); //Turn amber on
                led.setOutput(5, Off); //Turn green off
            }
            if(mode == IndicatorMode::ERROR_CRITICAL) {
                led.setOutput(6, PWM); //Turn amber on
                led.setOutput(5, Off); //Turn green off
            }
            break;
        case IndicatorLight::ALL:
            if(mode == IndicatorMode::WAITING) {
                led.setOutputArray(Off); //Turn all LEDs off //DEBUG!
                // led.setBrightness(6, 50);
                // led.setBrightness(4, 50);
                // led.setBrightness(1, 50);
                led.setOutput(6, Group); //Set CELL amber to blink
                led.setOutput(4, Group); //Set GPS amber to blink
                led.setOutput(1, Group); //Set SENSOR amber to blink
            }
            if(mode == IndicatorMode::NONE) {
                led.setOutputArray(Off); //Turn all LEDs off 
            }
            if(mode == IndicatorMode::INIT) {
                led.setOutputArray(Off); //Turn all LEDs off //DEBUG!
                led.setOutput(1, Group); //Blink amber with group
                led.setOutput(2, Group); //Blink red with group
                led.setOutput(6, Group); //Blink amber with group
                led.setOutput(4, Group); //Blink amber with group
            }
            if(mode == IndicatorMode::IDLE) {
                led.setOutputArray(Group); //Turn all LEDs to group blink
            }
            if(mode == IndicatorMode::COMMAND) {
                led.setOutputArray(Group); //Turn all LEDs to group blink
                led.setGroupBlinkPeriod(100); //Set to very fast blinking
	            led.setGroupOnTime(25);  
            }
            break;
    }
    enableI2C_Global(currentGlob); //Reset to previous state
    enableI2C_OB(currentOB); 
    return 0; //DEBUG!
}

unsigned long Kestrel::getMessageID()
{
    unsigned long currentTime = getTime(); //Grab current UNIX time
    //Create a hash between getTime (current UNIX time, 32 bit) and seconds since program start (use seconds to ensure demoninator is always smaller to not lose power of mod)
    if(currentTime != 0) return getTime() % (millis() / 1000); //If current time is valid, create the hash as usual
    else return HAL_RNG_GetRandomNumber(); //Else return cryptographic 32 bit random 
}

bool Kestrel::testForBat()
{
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);
    ioOB.pinMode(PinsOB::CE, OUTPUT);
    ioOB.pinMode(PinsOB::CSA_EN, OUTPUT);
    ioOB.digitalWrite(PinsOB::CE, HIGH); //Disable charging
    ioOB.digitalWrite(PinsOB::CSA_EN, HIGH); //Enable voltage sense
    csaAlpha.EnableChannel(CH1, true);
    csaAlpha.Update(); //Force new readings 
    delay(5000); //Wait for cap to discharge 
    // csaAlpha.SetCurrentDirection(CH1, BIDIRECTIONAL);
    float vBat = csaAlpha.GetBusVoltage(CH1);
    ioOB.digitalWrite(PinsOB::CE, LOW); //Turn charging back on
    bool result = false;
    if(vBat < 2.0) { //If less than 2V (min bat voltage) give error 
        //THROW ERROR???
        result = false;
    }
    else result = true;
    enableI2C_Global(currentGlob); //Reset to previous state
    enableI2C_OB(currentOB); 
    Serial.print("BATTERY STATE: "); //DEBUG!
    Serial.print(vBat);
    Serial.print("\t");
    Serial.println(result);
    return result;
    // enableI2C_External(true);
}

bool Kestrel::feedWDT()
{
    if(!criticalFault) { //If there is currently no critical fault, feed WDT
        pinMode(Pins::WD_HOLD, OUTPUT);
        digitalWrite(Pins::WD_HOLD, LOW);
        delay(1);
        digitalWrite(Pins::WD_HOLD, HIGH);
        delay(1);
        digitalWrite(Pins::WD_HOLD, LOW);
        return true;
    } 
    else {
        // System.reset(); //DEBUG!
        return false;
    }
}

bool Kestrel::configTalonSense()
{
    Serial.println("CONFIG TALON SENSE"); //DEBUG!
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);
    csaBeta.SetCurrentDirection(CH4, UNIDIRECTIONAL); //Bulk voltage, unidirectional
	csaBeta.EnableChannel(CH1, false); //Disable all channels but 4
	csaBeta.EnableChannel(CH2, false);
	csaBeta.EnableChannel(CH3, false);
	csaBeta.EnableChannel(CH4, true);
    enableI2C_Global(currentGlob); //Reset to previous state
    enableI2C_OB(currentOB); 
    // enableI2C_Global(true); //Connect all together 
    return false; //DEBUG!
}

void Kestrel::timechange_handler(system_event_t event, int param)
{
    // Serial.print("Time Change Handler: "); //DEBUG!
    // Serial.print(event); //DEBUG!
    // Serial.print("\t");
    // Serial.println(param); //DEBUG!
    if(event == time_changed) { //Confirm event type before proceeding 
        if(param == time_changed_sync && !(selfPointer->timeSyncRequested)) { 
            Serial.println("TIME CHANGE: Auto"); //DEBUG!
            selfPointer->syncTime(); //if time update not from manual sync (and sync not requested), call time sync
        }
        if(param == time_changed_sync && selfPointer->timeSyncRequested) Serial.println("TIME CHANGE: Requested"); //DEBUG!
        if(param == time_changed_manually) Serial.println("TIME CHANGE: Manual"); //DEBUG!
    }
    
}

time_t Kestrel::timegm(struct tm *tm)
{
    time_t ret;
    char *tz;

   tz = getenv("TZ");
    setenv("TZ", "", 1);
    tzset();
    ret = mktime(tm);
    if (tz)
        setenv("TZ", tz, 1);
    else
        unsetenv("TZ");
    tzset();
    return ret;
}


