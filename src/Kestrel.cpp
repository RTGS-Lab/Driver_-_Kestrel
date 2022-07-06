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

Kestrel::Kestrel() : ioOB(0x20), ioTalon(0x21)
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

    enableI2C_Global(false); //Turn off external I2C
    enableI2C_OB(true); //Turn on internal I2C

    ioOB.begin();
    ioTalon.begin();
    if(rtc.begin(true) == 0) criticalFault = true; //Use with external oscilator, set critical fault if not able to connect 
    if(Particle.connected() == false) criticalFault = true; //If not connected to cell set critical error
    for(int i = 1; i <= 4; i++) {
        enablePower(i, false); //Default all power to off
        enableData(i, false); //Default all data to off
    }
    syncTime(); 
    Particle.syncTime(); //DEBUG!
    // ioOB.pinMode(PinsOB::)

    return ""; //DEBUG!
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

bool Kestrel::enablePower(uint8_t port, bool state) 
{
    //FIX! Throw error is port out of range
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_ERROR | portErrorCode);
    else {
        enableI2C_OB(true);
        enableI2C_Global(false);
        // Wire.reset(); //DEBUG!
        ioTalon.pinMode(PinsTalon::EN[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::EN[port - 1], state);
    }
    
    return false; //DEBUG!
}

bool Kestrel::enableData(uint8_t port, bool state)
{
    //FIX! Throw error is port out of range
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_ERROR | portErrorCode);
    else {
        enableI2C_OB(true);
        enableI2C_Global(false);
        // Wire.reset(); //DEBUG!
        ioTalon.pinMode(PinsTalon::SEL[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::SEL[port - 1], LOW); //DEBUG!
        ioTalon.pinMode(PinsTalon::I2C_EN[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::I2C_EN[port - 1], state);
        // Serial.println(PinsTalon::I2C_EN[port - 1]); //DEBUG!
    }
    
    return false; //DEBUG!
}

bool Kestrel::enableI2C_OB(bool state)
{
    pinMode(Pins::I2C_OB_EN, OUTPUT);
	digitalWrite(Pins::I2C_OB_EN, state);
    Wire.reset(); //DEBUG!
    return false; //DEBUG!
}

bool Kestrel::enableI2C_Global(bool state)
{
    pinMode(Pins::I2C_GLOBAL_EN, OUTPUT);
	digitalWrite(Pins::I2C_GLOBAL_EN, state);
    Wire.reset(); //DEBUG!
    return false; //DEBUG!
}

bool Kestrel::disablePowerAll()
{
    for(int i = 1; i <= 4; i++) {
        enablePower(i, false);
    }
    return false; //DEBUG!
}

bool Kestrel::disableDataAll()
{
    for(int i = 1; i <= 4; i++) {
        enableData(i, false);
    }
    return 0; //DEBUG!
}

int Kestrel::throwError(uint32_t error)
{
	errors[(numErrors++) % MAX_NUM_ERRORS] = error; //Write error to the specified location in the error array
	if(numErrors > MAX_NUM_ERRORS) errorOverwrite = true; //Set flag if looping over previous errors 
	return numErrors;
}

bool Kestrel::enableSD(bool state)
{
    enableI2C_OB(true);
    enableI2C_Global(false);
    if(state) {
        enableAuxPower(true); //Make sure Aux power is on
        ioOB.pinMode(PinsOB::SD_EN, OUTPUT);
        ioOB.digitalWrite(PinsOB::SD_EN, HIGH);
    }
    else if(!state) {
        ioOB.pinMode(PinsOB::SD_EN, OUTPUT);
        ioOB.digitalWrite(PinsOB::SD_EN, LOW);
    }
    return false; //DEBUG!
}

bool Kestrel::enableAuxPower(bool state)
{
    enableI2C_OB(true);
    enableI2C_Global(false);
    ioOB.pinMode(PinsOB::AUX_EN, OUTPUT);
    ioOB.digitalWrite(PinsOB::AUX_EN, state); 
    return false; //DEBUG!
}

bool Kestrel::updateTime()
{
    Serial.print("Current Timer Period: "); //DEBUG!
    Serial.println(logPeriod);
    if(Time.isValid()) { //If particle time is valid, set from this
        currentDateTime.year = Time.year();
        currentDateTime.month = Time.month();
        currentDateTime.day = Time.day();
        currentDateTime.hour = Time.hour();
        currentDateTime.minute = Time.minute();
        currentDateTime.second = Time.second();
        currentDateTime.source = TimeSource::CELLULAR;
        return true; //debug!
    }
    else { //If time is not valid, write to default time //FIX??
        currentDateTime.year = 2000;
        currentDateTime.month = 1;
        currentDateTime.day = 1;
        currentDateTime.hour = 0;
        currentDateTime.minute = 0;
        currentDateTime.second = 0;
        currentDateTime.source = TimeSource::NONE;
        return false;
    }
    return false;
}

bool Kestrel::syncTime()
{
    //Synchronize time across GPS, Cell and RTC
    Serial.println("TIME SYNC!"); //DEBUG!
    if(Particle.connected()) {
        timeSyncRequested = true;
        Particle.syncTime();
        waitFor(Particle.syncTimeDone, 5000); //Wait until sync is done, at most 5 seconds //FIX!
        Serial.print("Particle Time: "); 
        Serial.println(Time.now());
        Serial.print("RTC Time: ");
        Serial.println(rtc.getTimeUnix());
        timeSyncRequested = false; //Release control of time sync override 
    }
    return false; //DEBUG!
}

time_t Kestrel::getTime()
{
    if(!Time.isValid()) { //If time has not been synced, do so now
        syncTime();
    }
    if(Time.isValid()) { //If time is good, report current value
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
    else return "DUMMY"; //FIX!
    // else return String(currentTime); //Otherwise return normal string val
}

bool Kestrel::startTimer(time_t period)
{
    if(period == 0) period = defaultPeriod; //If no period is specified, assign default period 
    enableI2C_OB(true);
    enableI2C_Global(false);
    rtc.setAlarm(period); //Set alarm from current time
    timerStart = millis(); 
    logPeriod = period;
    return false; //DEBUG!
}

bool Kestrel::waitUntilTimerDone()
{
    if(logPeriod == 0) return false; //Return if not already setup
    pinMode(Pins::Clock_INT, INPUT);
    while(digitalRead(Pins::Clock_INT) == HIGH && ((millis() - timerStart) < logPeriod*1000 + 500)); //Wait until either timer has expired or clock interrupt has gone off //DEBUG! Give 500 ms cushion for testing RTC
    if(digitalRead(Pins::Clock_INT) == LOW) return true; //If RTC triggers properly, return true, else return false 
    else return false; 
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
        System.reset(); //DEBUG!
        return false;
    }
}

void Kestrel::timechange_handler(system_event_t event, int param)
{
    Serial.print("Time Change Handler: "); //DEBUG!
    Serial.print(event); //DEBUG!
    Serial.print("\t");
    Serial.println(param); //DEBUG!
    if(event == time_changed) { //Confirm event type before proceeding 
        if(param == time_changed_sync && !(selfPointer->timeSyncRequested)) { 
            Serial.println("TIME CHANGE: Auto"); //DEBUG!
            selfPointer->syncTime(); //if time update not from manual sync (and sync not requested), call time sync
        }
        if(param == time_changed_sync && selfPointer->timeSyncRequested) Serial.println("TIME CHANGE: Requested"); //DEBUG!
        if(param == time_changed_manually) Serial.println("TIME CHANGE: Manual"); //DEBUG!
    }
    
}


