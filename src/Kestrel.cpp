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
    // talonPort = 0x0F; //Set dummy Talon port for reporting
    sensorInterface = BusType::CORE;
}

String Kestrel::begin(time_t time, bool &criticalFault, bool &fault)
{
    selfPointer = this;
    System.on(time_changed, timechange_handler);
    System.on(out_of_memory, outOfMemoryHandler);
    #if defined(ARDUINO) && ARDUINO >= 100 
		Wire.begin();
        Wire.setClock(400000); //Confirm operation in fast mode
	#elif defined(PARTICLE)
		if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
	#endif
    if(!initDone) throwError(SYSTEM_RESET | ((System.resetReason() << 8) & 0xFF00)); //Throw reset error with reason for reset as subtype. Truncate resetReason to one byte. This will include all predefined reasons, but will prevent issues if user returns some large (technically can be up to 32 bits) custom reset reason
    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
    if(ioOB.begin() != 0) criticalFault = true;
    if(ioTalon.begin() != 0) criticalFault = true;
    ioTalon.safeMode(PCAL9535A::SAFEOFF); //DEBUG! //Turn safe mode off to speed up turn-off times for Talons
    enableAuxPower(true); //Turn on aux power 
    csaAlpha.begin();
    csaBeta.begin();
    csaAlpha.setFrequency(Frequency::SPS_64); //Set to ensure at least 24 hours between accumulator rollover 
    // delay(100); //DEBUG! For GPS
    ioOB.pinMode(PinsOB::LED_EN, OUTPUT);
	ioOB.digitalWrite(PinsOB::LED_EN, LOW); //Turn on LED indicators 
    led.begin();
    if(!initDone) { //Only set state if not done already
        led.setOutputMode(OpenDrain); //Set device to use open drain outputs
        led.setGroupMode(Blink); //Set system to blinking mode
        led.setOutputArray(Off); //Turn all off by default
        led.setBrightnessArray(ledBrightness); //Set all LEDs to 50% max brightness
        // led.setGroupBrightness(ledBrightness); //Set to 50% brightness
        led.setGroupBlinkPeriod(ledPeriod); //Set blink period to specified number of ms
        led.setGroupOnTime(ledOnTime); //Set on time for each blinking period 
    }
    
    // setIndicatorState(IndicatorLight::ALL,IndicatorMode::WAITING); //Set all to blinking wait
    
    if(rtc.begin(true) == 0) criticalFault = true; //Use with external oscilator, set critical fault if not able to connect 
    if(gps.begin() == false) {
        criticalFault = true; //DEBUG! ??
        //Throw ERROR!
        Serial.println("GPS ERROR");
    }
    else {
        gps.setI2COutput(COM_TYPE_UBX);
        gps.setNavigationFrequency(1); //Produce 1 solutions per second
        gps.setAutoPVT(false); //DEBUG!
        Serial.print("GPS Stats: "); //DEBUG!
        Serial.print(gps.getNavigationFrequency());
        Serial.print("\t");
        Serial.print(gps.getMeasurementRate());
        Serial.print("\t");
        Serial.println(gps.getNavigationRate());
        Serial.print("GPS Attitude: ");
        Serial.print(gps.getATTroll());
        Serial.print("\t");
        Serial.print(gps.getATTpitch());
        Serial.print("\t");
        Serial.println(gps.getATTheading());
    }
    //Read in accel offset from EEPROM. Do this here so it is only done once per reset cycle and is immediately available 
    for(int i = 0; i < 3; i++) { 
        float temp = 0;
        EEPROM.get(i*4, temp); //Read in offset vals
        if(!isnan(temp)) accel.offset[i] = temp; //Set offset vals if real number (meaning offset has been established)
        else accel.offset[i] = 0; //If there is no existing offset, set to 0
    }
    // if(Particle.connected() == false) criticalFault = true; //If not connected to cell set critical error
    // if(criticalFault) setIndicatorState(IndicatorLight::STAT, IndicatorMode::ERROR_CRITICAL); //If there is a critical fault, set the stat light
    for(int i = 1; i <= 4; i++) {
        enablePower(i, false); //Default all power to off
        enableData(i, false); //Default all data to off
    }
    
    syncTime(); 
    // Particle.syncTime(); //DEBUG!
    // ioOB.pinMode(PinsOB::)
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    initDone = true;
    return ""; //DEBUG!
}

String Kestrel::getErrors()
{
    // uint32_t temp[10] = {0}; //Temp array to store error vals read in from drivers

    String output = "\"KESTREL\":{"; // OPEN JSON BLOB
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
	output = output + "}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
    rtc.numErrors = 0; 
	return output;
}

String Kestrel::getData(time_t time)
{
    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
    enableAuxPower(true); //Turn on aux power 
    gps.getPVT(); //Force updated call //DEBUG!
    if(gps.getFixType() >= 2) { //Only update if GPS has at least a 2D fix
        longitude = gps.getLongitude();
        latitude = gps.getLatitude();
        altitude = gps.getAltitude();
        posTime = getTime(); //Update time that GPS measure was made
    }
    else {
        throwError(GPS_UNAVAILABLE); //If no fix available, throw error
    }
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    // return "{\"Kestrel\":null}";
    return "";
}

String Kestrel::getMetadata()
{
    //RTC UUID
    //GPS SN
    //B402 ID/SN
    //File names?? -> this goes in file handler? 
    //SD info?? -> this goes in file handler?
    //IDs of onboard sensors (if any have them)
        //SHT40
    //

    bool auxState = enableAuxPower(true); //Turn on AUX power for GPS
    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
    String metadata = "\"Kestrel\":{";
    ////////// ADD GPS INFO
    // if(gps.begin() == false) throwError(GPS_INIT_FAIL);
    // else {
    //     gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    //     uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes. MAX_PAYLOAD_SIZE defaults to 256. The CFG_RATE payload is only 6 bytes!
    //     gps.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);
    //     ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
    //     customCfg.cls = UBX_CLASS_CFG; // This is the message Class
    //     customCfg.id = UBX_MON_VER; // This is the message ID
    //     customCfg.len = 0; // Setting the len (length) to zero let's us poll the current settings
    //     customCfg.startingSpot = 0; // Always set the startingSpot to zero (unless you really know what you are doing)
    //     uint16_t maxWait = 250; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)
    //     if (gps.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) throwError(GPS_READ_FAIL); // We are expecting data and an ACK, throw error otherwise

    // }

    String rtcUUID = rtc.getUUIDString(); 
    if(!rtcUUID.equals("null")) rtcUUID = "\"" + rtcUUID + "\""; //If not null, wrap with quotes for JSON, otherwise leave as null 
    metadata = metadata + "\"RTC UUID\":" + rtcUUID + ","; //Append RTC UUID

    /////// Particle info //////////// 
    // metadata = metadata + "\"OS\":\"" + System.version() + "\",";
    // metadata = metadata + "\"ID\":\"" + System.deviceID() + "\",";
    if(PLATFORM_ID == PLATFORM_BSOM) metadata = metadata + "\"Model\":\"BSoM\","; //Report BSoM
    else if (PLATFORM_ID == PLATFORM_B5SOM) metadata = metadata + "\"Model\":\"B5SoM\","; //Report B5SoM
    else metadata = metadata + "\"Model\":null,"; //Report null if for some reason the firmware is running on another device 

    ///////// ADD SHT40!

	
	metadata = metadata + "\"Firmware\":\"v" + FIRMWARE_VERSION + "\","; //Report firmware version as modded BCD
	metadata = metadata + "\"Pos\":[15]"; //Concatonate position 
	metadata = metadata + "}"; //CLOSE  
    enableAuxPower(auxState); //Return to previous state
    enableI2C_Global(globState); 
    enableI2C_OB(obState);
	return metadata; 
	return ""; //DEBUG!
}

String Kestrel::selfDiagnostic(uint8_t diagnosticLevel, time_t time)
{
    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
	String output = "\"Kestrel\":{";
	if(diagnosticLevel == 0) {
		//TBD
		// output = output + "\"lvl-0\":{},";
	}

	if(diagnosticLevel <= 1) {
		//TBD
		// output = output + "\"lvl-1\":{},";
	}

	if(diagnosticLevel <= 2) {
		//TBD
		// output = output + "\"lvl-2\":{},";
        output = output + "\"Accel_Offset\":[" + String(accel.offset[0]) + "," + String(accel.offset[1]) + "," + String(accel.offset[2]) + "],"; 
	}

	if(diagnosticLevel <= 3) {
		//TBD
		// Serial.println(millis()); //DEBUG!
		// output = output + "\"lvl-3\":{"; //OPEN JSON BLOB
        Wire.beginTransmission(0x6F);
        uint8_t rtcError = Wire.endTransmission();
        if(rtcError == 0) {
            time_t currentTime = rtc.getTimeUnix();
            delay(1200); //Wait at least 1 second (+20%)
            if((rtc.getTimeUnix() - currentTime) == 0) throwError(RTC_OSC_FAIL); //If rtc is not incrementing, throw error 
        }
        else throwError(RTC_READ_FAIL | rtcError << 8); //Throw error since unable to communicate with RTC
        
		// /////// TEST I2C WITH LOOPBACK ////////////
		// output = output + "\"I2C_PORT_FAIL\":["; 
		// disableDataAll(); //Turn off all data 
		// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect to external I2C
		// Wire.beginTransmission(0x22);
		// int error = Wire.endTransmission(); //Get error from write to empty bus
		// if(error == 0) throwError(I2C_OB_ISO_FAIL | talonPortErrorCode); //We should not be able to connect to IO exp in this state, if we can throw error 
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// ioAlpha.digitalWrite(pinsAlpha::LOOPBACK_EN, HIGH); //Connect loopback 
		// for(int i = 0; i <= numPorts; i++) { //Iterate over each port
		// 	int totalErrors = 0; //Track how many of the test calls fail
		// 	if(i > 0) {
		// 		enablePower(i, true); //Turn on power to a given power after testing the base bus
		// 		enableData(i, true); //Turn on data to a given port after testing the base bus
		// 	}
		// 	digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect to external I2C (w/loopback enabled, so it is a combined bus now)
		// 	for(int adr = 0; adr < sizeof(expectedI2CVals); adr++) { //Check for addresses present 
		// 		Wire.beginTransmission(expectedI2CVals[adr]); //Check for each expected address
		// 		// Wire.write(0x00);
		// 		int error = Wire.endTransmission();
		// 		if(error == 2) { //If bad bus error detected 
		// 			// Serial.print("I2C Error Detected: "); //DEBUG!
		// 			// Serial.println(error);
		// 			totalErrors++; //If a failure occours, increment error count
		// 		}
		// 		// delay(1); //DEBUG!
		// 	}
		// 	Serial.print("Total Errors: "); //DEBUG!
		// 	Serial.println(totalErrors);
		// 	if(totalErrors > 0) { //If any bus failures were detected 
		// 		throwError(I2C_PORT_FAIL | talonPortErrorCode | i); //Throw error for the port under test
		// 		output = output + String(i) + ",";
		// 	}
		// 	if(i > 0) enableData(i, false); //Disable bus again
		// }
		// if(output.substring(output.length() - 1).equals(",")) {
		// 	output = output.substring(0, output.length() - 1); //Trim trailing ',' if present
		// }
		// output = output + "]"; //Close I2C port array
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// ioAlpha.digitalWrite(pinsAlpha::LOOPBACK_EN, LOW); //Disable loopback 
		// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Return to default external connecton

		// output = output + "},"; //CLOSE JSON BLOB
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;

 	}

	if(diagnosticLevel <= 4) {
		// String output = selfDiagnostic(5); //Call the lower level of self diagnostic 
		// output = output.substring(0,output.length() - 1); //Trim off closing brace
		// output = output + "\"lvl-4\":{"; //OPEN JSON BLOB

		// ioSense.begin(); //Initalize voltage sensor IO expander
		///////////// SENSE VOLTAGE AND CURRENT FOR PORTS ///////////
		// for(int p = 1; p <= numTalonPorts; p++) {
		// 	enablePower(p, true); //Turn on power to all ports before measuring //DEBUG!
		// }
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C

		// for(int i = pinsSense::MUX_SEL0; i <= pinsSense::MUX_EN; i++) { //Set all pins to output
		// 	ioSense.pinMode(i, OUTPUT); 
		// }
		// ioAlpha.digitalWrite(pinsAlpha::SENSE_EN, HIGH); //Make sure 3v3 Sense is turned on
		// ioSense.digitalWrite(pinsSense::MUX_EN, LOW); //Turn MUX on 
		// int SenseError = adcSense.Begin(); //Initialize ADC 
        static time_t lastAccReset = 0; //Grab time that accumulators were reset. Set to 0 on restart
        ioOB.digitalWrite(PinsOB::CSA_EN, HIGH); //Enable CSA GPIO control
        bool initA = csaAlpha.begin();
        bool initB = csaBeta.begin();
		if(initA == true || initB == true) { //Only proceed if one of the ADCs connects correctly
			// adcSense.SetResolution(18); //Set to max resolution (we paid for it right?) 
            //Setup CSAs
            if(initA == true) {
                csaAlpha.enableChannel(Channel::CH1, true); //Enable all channels
                csaAlpha.enableChannel(Channel::CH2, true);
                csaAlpha.enableChannel(Channel::CH3, true);
                csaAlpha.enableChannel(Channel::CH4, true);
                csaAlpha.setCurrentDirection(Channel::CH1, BIDIRECTIONAL);
                csaAlpha.setCurrentDirection(Channel::CH2, UNIDIRECTIONAL);
                csaAlpha.setCurrentDirection(Channel::CH3, UNIDIRECTIONAL);
                csaAlpha.setCurrentDirection(Channel::CH4, UNIDIRECTIONAL);
            }

            if(initB == true) {
                csaBeta.enableChannel(Channel::CH1, true); //Enable all channels
                csaBeta.enableChannel(Channel::CH2, true);
                csaBeta.enableChannel(Channel::CH3, true);
                csaBeta.enableChannel(Channel::CH4, true);
                csaBeta.setCurrentDirection(Channel::CH1, UNIDIRECTIONAL);
                csaBeta.setCurrentDirection(Channel::CH2, UNIDIRECTIONAL);
                csaBeta.setCurrentDirection(Channel::CH3, UNIDIRECTIONAL);
                csaBeta.setCurrentDirection(Channel::CH4, UNIDIRECTIONAL);
            }
			output = output + "\"PORT_V\":["; //Open group
			// ioSense.digitalWrite(pinsSense::MUX_SEL2, LOW); //Read voltages
            if(initA == true) {
                // csaAlpha.enableChannel(Channel::CH1, true); //Enable all channels
                // csaAlpha.enableChannel(Channel::CH2, true);
                // csaAlpha.enableChannel(Channel::CH3, true);
                // csaAlpha.enableChannel(Channel::CH4, true);
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    output = output + String(csaAlpha.getBusVoltage(Channel::CH1 + i, true), 6); //Get bus voltage with averaging 
                    output = output + ","; //Append comma 
                }
            }
            else output = output + "null,null,null,null,"; //Append nulls if can't connect to csa alpha

			if(initB == true) {
                
                // delay(1000); //Wait for new data //DEBUG!
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    output = output + String(csaBeta.getBusVoltage(Channel::CH1 + i, true), 6); //Get bus voltage with averaging 
                    if(i < 3) output = output + ","; //Append comma if not the last reading
                }
            }
            else {
                output = output + "null,null,null,null"; //Append nulls if can't connect to csa beta
                throwError(CSA_INIT_FAIL | 0xB00); //Throw error for ADC beta failure
            }

			output = output + "],"; //Close group
			output = output + "\"PORT_I\":["; //Open group
            if(initA == true) {
                // csaAlpha.enableChannel(Channel::CH1, true); //Enable all channels
                // csaAlpha.enableChannel(Channel::CH2, true);
                // csaAlpha.enableChannel(Channel::CH3, true);
                // csaAlpha.enableChannel(Channel::CH4, true);
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    output = output + String(csaAlpha.getCurrent(Channel::CH1 + i, true), 6); //Get bus voltage with averaging 
                    output = output + ","; //Append comma 
                }
            }
            else {
                output = output + "null,null,null,null,"; //Append nulls if can't connect to csa alpha
                throwError(CSA_INIT_FAIL | 0xA00); //Throw error for ADC failure
            }

			if(initB == true) {
                // csaBeta.enableChannel(Channel::CH1, true); //Enable all channels
                // csaBeta.enableChannel(Channel::CH2, true);
                // csaBeta.enableChannel(Channel::CH3, true);
                // csaBeta.enableChannel(Channel::CH4, true);
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    output = output + String(csaBeta.getCurrent(Channel::CH1 + i, true), 6); //Get bus voltage with averaging 
                    if(i < 3) output = output + ","; //Append comma if not the last reading
                }
            }
            else {
                output = output + "null,null,null,null"; //Append nulls if can't connect to csa beta
                throwError(CSA_INIT_FAIL | 0xB00); //Throw error for ADC failure
            }
			output = output + "],"; //Close group
            output = output + "\"AVG_P\":["; //Open group
            if(lastAccReset == 0) { //If unknown time since last reset, clear accumulators on csa Alpha
                csaAlpha.update(true); 
                lastAccReset = getTime(); //Update time of reset
            }
            if(initA == true) {
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    output = output + String(csaAlpha.getPowerAvg(Channel::CH1 + i)); //Get bus voltage with averaging 
                    if(i < 3) output = output + ","; //Append comma if not the last reading
                }
            }
            else {
                output = output + "null,null,null,null"; //Append nulls if can't connect to csa alpha
                throwError(CSA_INIT_FAIL | 0xA00); //Throw error for ADC failure
            }
            output = output + "],"; //Close group
            output = output + "\"LAST_CLR\":" + String((int)lastAccReset) + ","; //Append the time of the last accumulator clear
            if((getTime() - lastAccReset) > 86400 && (getTime() % 86400) < 3600) { //If it is zero hour in UTC and it has been more than 24 hours since the last reset, clear accumulators 
                csaAlpha.update(true); 
                lastAccReset = getTime(); //Update time of reset
            }
			
		}
		else { //If unable to initialzie ADC
			output = output + "\"PORT_V\":[null],\"PORT_I\":[null],\"AVG_P\":[null]";
			throwError(CSA_INIT_FAIL); //Throw error for global CSA failure
		}
        output = output + "\"ALS\":";
        if(als.begin() == 0) {
            output = output + String(als.GetLux()) + ","; //appenbd ALS results 
        }
        else {
            output = output + "null,";
            //THROW ERROR
        }

        String temperatureString = "\"Temperature\":["; //Used to gather temp from multiple sources
        if(atmos.begin()) {
            atmos.setPrecision(SHT4X_MED_PRECISION); //Set to mid performance 
            sensors_event_t humidity, temp;
            atmos.getEvent(&humidity, &temp);
            output = output + "\"RH\":" + String(humidity.relative_humidity, 4) + ","; //Concatonate atmos data 
            temperatureString = temperatureString + String(temp.temperature, 4) + ",";
        }
        else {
            output = output + "\"RH\":null,"; //append null string
            temperatureString = temperatureString + "null,";
            //THROW ERROR
        }
        atmos.~Adafruit_SHT4x(); //Delete objects

        int accelInitError = accel.begin();
        if(accelInitError == 0) {
            
            int accelError = accel.updateAccelAll();
            if(accelError != 0) {
                throwError(ACCEL_DATA_FAIL | (accelError << 8)); //Throw error for failure to communicate with accel, OR error code
                //FIX! Null outputs??
            }
            output = output + "\"ACCEL\":[" + String(accel.data[0]) + "," + String(accel.data[1]) + "," + String(accel.data[2]) + "],"; 
            temperatureString = temperatureString + String(accel.getTemp(), 4);
        }
        else {
            throwError(ACCEL_DATA_FAIL | (accelInitError << 8)); //Throw error for failure to communicate with accel, OR error code 
            output = output + "\"ACCEL\":[null],";
            temperatureString = temperatureString + "null";
        }
		// ioSense.digitalWrite(pinsSense::MUX_EN, HIGH); //Turn MUX back off 
		// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Return to default external connecton
        temperatureString = temperatureString + "]";
        output = output + "\"SIV\":" + String(gps.getSIV()) + ",\"FIX\":" + String(gps.getFixType()) + ",";
		output = output + temperatureString + ","; 
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;

	}

	if(diagnosticLevel <= 5) {
		// output = output + "\"lvl-5\":{"; //OPEN JSON BLOB
        if(System.freeMemory() < 15600) { //Throw error if RAM usage >90% //FIX! Check dynamically for amount of RAM available based on OS, etc 
            throwError(RAM_CRITICAL); 
            criticalFault = true; //Let WDT off leash to fix issue
        }
        else if(System.freeMemory() < 46800) throwError(RAM_LOW); //Throw error if RAM usage >75% //FIX! Check dynamically for amount of RAM available based on OS, etc 
        output = output + "\"Free Mem\":" + String(System.freeMemory()) + ","; //DEBUG! Move to higher level later on
        output = output + "\"Time Source\":" + String(timeSource) + ","; //Report the time souce selected from the last sync
        output = output + "\"Times\":[" + String((int)timeSyncVals[0]) + "," + String((int)timeSyncVals[1]) + "," + String((int)timeSyncVals[2]) + "],"; //Add reported times from last sync
        output = output + "\"Last Sync\":" + String((int)lastTimeSync) + ",";
        output = output + "\"OB\":" + ioOB.readBus() + ",\"Talon\":" + ioTalon.readBus() + ","; //Report the bus readings from the IO expanders
        output = output + "\"I2C\":["; //Append identifer 
        for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
            Wire.beginTransmission(adr);
            // Wire.write(0x00);
            int error = Wire.endTransmission();
            // if(adr == 0) { //DEBUG!
            //     // Serial.print("Zero Error: ");
            //     // Serial.println(error); 
            // }
            if(error == 0) {
                output = output + String(adr) + ",";
            }
            delay(1); //DEBUG!
        }
        if(output.substring(output.length() - 1).equals(",")) {
            output = output.substring(0, output.length() - 1); //Trim trailing ',' if present
        }
        output = output + "],"; //Close array
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// disableDataAll(); //Turn off all data 
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// for(int i = 0; i < numPorts; i++) {
		// 	// overflow[i] = ioBeta.getInterrupt(pinsBeta::OVF1 + i); //Read in overflow values
		// 	faults[i] = ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i); //Read in fault values
		// 	if (faults[i] == true) {
		// 		throwError(SENSOR_POWER_FAIL | portErrorCode | i); //Throw power fault error with given port appended 
		// 	}
		// }

		// output = output + "\"ALPHA\":" + String(ioAlpha.readBus()) + ","; //Append ALPHA port readout
		// // output = output + "\"BETA\":" + String(ioBeta.readBus()) + ","; //Append BETA port readout
		// output = output + "\"ALPHA_INT\":" + String(ioAlpha.getAllInterrupts(PCAL9535A::IntAge::BOTH)) + ","; //Append ALPHA interrupt readout
		// uint8_t senseBus = 0; //Manually concatonate pin reads from ioSense expander
		// for(int i = 0; i < 4; i++) { //NOT associated with numPorts, which is why variable is not used
		// 	senseBus = senseBus | (ioSense.read(i) << i); //DEBUG!
		// 	// senseBus = 0; //DEBUG!
		// }
		// output = output + "\"MUX\":" + String(senseBus) + ","; //Append MUX controller readout

		// output = output + "\"I2C_OB\":[";
		// digitalWrite(KestrelPins::PortBPins[talonPort], HIGH); //Connect to internal I2C
		// // digitalWrite(D6, HIGH); //Connect to internal I2C
		// ioAlpha.digitalWrite(pinsAlpha::LOOPBACK_EN, LOW); //Make sure loopback is turned off
		// for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
		// 	Wire.beginTransmission(adr);
		// 	// Wire.write(0x00);
		// 	if(Wire.endTransmission() == 0) {
		// 		output = output + String(adr) + ",";
		// 	}
		// 	delay(1); //DEBUG!
		// }
		// if(output.substring(output.length() - 1).equals(",")) {
		// 	output = output.substring(0, output.length() - 1); //Trim trailing ',' is present
		// }
		// output = output + "],"; // close array

		// disableDataAll(); //Make sure all data ports are turned off to begin with
		// for(int port = 1; port <= numPorts; port++) { //CHECK EACH SENSOR PORT FOR ADDRESSES
		// 	output = output + "\"I2C_" + String(port) + "\":["; //Append identifer for
		// 	Serial.print("Enable States: "); //DEBUG! And following prints
		// 	// Serial.print(enablePower(port, true)); //Turn on power to given port //DEBUG! REPLACE!
		// 	Serial.println(enableData(port, true)); //Turn on data to given port
		// 	// delay(10);
		// 	// digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Connect to external I2C
		// 	for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
		// 		Wire.beginTransmission(adr);
		// 		// Wire.write(0x00);
		// 		int error = Wire.endTransmission();
		// 		if(adr == 0) { //DEBUG!
		// 			Serial.print("Zero Error: ");
		// 			Serial.println(error); 
		// 		}
		// 		if(error == 0) {
		// 			output = output + String(adr) + ",";
		// 		}
		// 		delay(1); //DEBUG!
		// 	}
		// 	enableData(port, false); //Turn off data to given port
		// 	if(output.substring(output.length() - 1).equals(",")) {
		// 		output = output.substring(0, output.length() - 1); //Trim trailing ',' if present
		// 	}
		// 	output = output + "]"; //Close array
		// 	if(port < numPorts) output = output + ","; //Only add comma if not the last entry in array 
		// }
		// output = output + "}"; //Close pair
		
		
		// // output = output + "}"; //CLOSE JSON BLOB, 
		// ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
		// // ioBeta.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Beta
		// // return output + ",\"Pos\":[" + String(port) + "]}}";
		// // return output;
	}
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
	return output + "\"Pos\":[15]}"; //Write position in logical form - Return compleated closed output
	// else return ""; //Return empty string if reaches this point 

	// return "{}"; //Return null if reach end	
	// return output + ",\"Pos\":[" + String(port) + "]}}"; //Append position and return
	// return "{}"; //DEBUG!
}
bool Kestrel::connectToCell()
{
    //FIX! Check for cell module on, etc
    Particle.connect();
    waitFor(Particle.connected, CELL_TIMEOUT); //Wait for cell to connect
    if(Particle.connected()) return true;
    else {
        throwError(CELL_FAIL); //FIX! add varing reasons for fail
        return false;        
    } 
}

bool Kestrel::enablePower(uint8_t port, bool state) 
{
    //FIX! Throw error is port out of range
    if(port == 5) { //Port for (ext/batter port) is special case
        // return enableAuxPower(state); 
        return false; //DEBUG!
    }
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_FAIL | portErrorCode);
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
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_FAIL | portErrorCode);
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
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_FAIL | portErrorCode);
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

bool Kestrel::getFault(uint8_t port) 
{
    if(port == 5) { //Port for (ext/batter port) is special case
        return false; //DEBUG!
    }
    if(port == 0 || port > numTalonPorts) throwError(KESTREL_PORT_RANGE_FAIL | portErrorCode);
    else {
        bool state = true;
        bool globState = enableI2C_Global(false);
        bool obState = enableI2C_OB(true);
        if(ioTalon.digitalRead(PinsTalon::EN[port - 1]) == HIGH) state = false; //If fault line is high, return false for no fault 
        else state = true; //If there is a read failure or otherwise unable to read, assume a fault
        enableI2C_Global(globState); //Return to previous state
        enableI2C_OB(obState);
        return state;
    }
    return true;
    
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

bool Kestrel::sdInserted()
{
    ioOB.pinMode(PinsOB::SD_CD, INPUT_PULLUP);
    if(ioOB.digitalRead(PinsOB::SD_CD) == LOW) return true; //If switch is closed, return true
    else return false; //Otherwise it is not inserted 
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
    
    Serial.print("Timebase Start: "); //DEBUG!
    Serial.println(millis());
    /////////// CELL TIME //////////////////
    if(Particle.connected()) {
        timeSyncRequested = true;
        Particle.syncTime();
        waitFor(Particle.syncTimeDone, 5000); //Wait until sync is done, at most 5 seconds //FIX!
        if(Particle.syncTimeDone()) { //Make sure sync time was actually completed 
            Time.zone(0); //Set to UTC 
            particleTime = Time.now();
            // timeSyncRequested = false; //Release control of time sync override 
            Serial.print("Cell Time: "); 
            Serial.println(particleTime);
            timeSyncVals[0] = Time.now(); //Grab last time
        }
        else {
            timeSyncVals[0] = 0;
            throwError(CLOCK_UNAVAILABLE | 0x06); //OR with Cell indicator 
        }
        timeSyncRequested = false; //Release control of time sync override 
        
    }
    else {
        timeSyncVals[0] = 0; //Clear if not updated
        throwError(CLOCK_UNAVAILABLE | 0x06); //OR with Cell indicator 
    }

    ////////// GPS TIME ///////////////////
    bool currentAux = enableAuxPower(true);
    bool currentGlob = enableI2C_Global(false);
    bool currentOB = enableI2C_OB(true);
    // if(gps.begin() == false) throwError(GPS_INIT_FAIL);
    // else {
        gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
        uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes. MAX_PAYLOAD_SIZE defaults to 256. The CFG_RATE payload is only 6 bytes!
        gps.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);
        ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
        customCfg.cls = UBX_CLASS_NAV; // This is the message Class
        // customCfg.id = UBX_NAV_TIMELS; // This is the message ID
        customCfg.id = UBX_NAV_TIMEUTC; // This is the message ID
        customCfg.len = 0; // Setting the len (length) to zero let's us poll the current settings
        customCfg.startingSpot = 0; // Always set the startingSpot to zero (unless you really know what you are doing)
        uint16_t maxWait = 1500; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)
        if (gps.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) {
            Serial.println("GPS READ FAIL"); //DEBUG!
            throwError(GPS_READ_FAIL); // We are expecting data and an ACK, throw error otherwise
        }
        // Serial.print("GPS UTC Seconds: "); //DEBUG!
        // Serial.println(customPayload[18]);
        Serial.print("GPS UTC Validity: "); //DEBUG!
        Serial.println(customPayload[19], HEX);

        // Serial.print("GPS Leap Seconds: "); //DEBUG!
        // Serial.println(customPayload[9]);
        
        // Serial.print("GPS Time: "); //DEBUG!
        // Serial.print(gps.getHour());
        // Serial.print(":");
        // Serial.print(gps.getMinute());
        // Serial.print(":");
        // Serial.println(gps.getSecond());
    // }
    // if(gps.getDateValid() && gps.getTimeValid() && gps.getTimeFullyResolved()) {
    if((customPayload[19] & 0x0F) == 0x07) { //Check if all times are valid
        // struct tm timeinfo = {0}; //Create struct in C++ time land

        // timeinfo.tm_year = (customPayload[12] | (customPayload[13] << 8)) - 1900; //Years since 1900
        // timeinfo.tm_mon = customPayload[14] - 1; //Months since january
        // timeinfo.tm_mday = customPayload[15];
        // timeinfo.tm_hour = customPayload[16];
        // timeinfo.tm_min = customPayload[17];
        // timeinfo.tm_sec = customPayload[18];
        // gpsTime = timegm(&timeinfo); //Convert struct to unix time
        gpsTime = cstToUnix((customPayload[12] | (customPayload[13] << 8)), customPayload[14], customPayload[15], customPayload[16], customPayload[17], customPayload[18]); //Convert current time to Unix time
        // gpsTime = 0xDEADBEEF; //DEBUG!
        Serial.print("GPS Time: ");
        Serial.println(gpsTime); //DEBUG!
        timeSyncVals[1] = gpsTime; //Grab last time
    }
    else {
        timeSyncVals[1] = 0; //Clear if not updated
        throwError(CLOCK_UNAVAILABLE | 0x08); //OR with GPS indicator 
    }

	
    /////////// RTC TIME //////////////
    Wire.beginTransmission(0x6F); //Check for presence of RTC //FIX! Find a better way to test if RTC time is available 
    uint8_t rtcError = Wire.endTransmission();
    if(rtcError != 0) {
        timeSyncVals[2] = 0; //Clear if unable to connect to RTC
        throwError(CLOCK_UNAVAILABLE | 0x05); //OR with RTC indicator 
    }
    else { //Only read values in if able to connect to RTC
        rtcTime = rtc.getTimeUnix();
        Serial.print("RTC Time: ");
        Serial.println(rtcTime); //DEBUG!
        Serial.print("Particle Time: ");
        Serial.println(particleTime);  
        timeSyncVals[2] = rtcTime; //Grab last time
    }
    
    uint8_t source = TimeSource::NONE; //Default to none unless otherwise set
    if(abs(rtcTime - gpsTime) < maxTimeError && abs(rtcTime - particleTime) < maxTimeError && rtcTime != 0 && gpsTime != 0 && particleTime != 0) { //If both updated sources match local time
        Serial.println("CLOCK SOURCE: All match");
        timeGood = true;
        source = TimeSource::CELLULAR; //Report cell as the most comprehensive source
    }

    if(abs(particleTime - gpsTime) < maxTimeError && gpsTime != 0 && particleTime != 0) { //If both updated variables match //FIX! This should be just an if, not an if else, so the RTC is updated 
        Serial.println("CLOCK SOURCE: GPS and Cell match");
        // time_t currentTime = Time.now(); //Ensure sync and that local offset is not applied 
        // rtc.setTime(Time.year(currentTime), Time.month(currentTime), Time.day(currentTime), Time.hour(currentTime), Time.minute(currentTime), Time.second(currentTime)); //Set RTC from Cell
        if(particleTime != 0) { //DEBUG! RESTORE!
            time_t currentTime = particleTime; //Grab time from cell, even though it is old, to ensure correct time is being set //FIX!
            Serial.print("RTC Set Time: "); //DEBUG!
            Serial.print(Time.hour(currentTime));
            Serial.print(":");
            Serial.print(Time.minute(currentTime));
            Serial.print(":");
            Serial.println(Time.second(currentTime));
            rtc.setTime(Time.year(currentTime), Time.month(currentTime), Time.day(currentTime), Time.hour(currentTime), Time.minute(currentTime), Time.second(currentTime)); //Set RTC from Cell
        }
        
        timeGood = true;
        source = TimeSource::CELLULAR;
        if(!(abs(rtcTime - gpsTime) < maxTimeError && abs(rtcTime - particleTime) < maxTimeError && rtcTime != 0 && gpsTime != 0 && particleTime != 0) && timeSyncVals[2] != 0) throwError(CLOCK_MISMATCH | 0x05); //Check if RTC is within range of others, if not throw error (only if not caused by unavailability)
    }
    else if(abs(particleTime - rtcTime) < maxTimeError && rtcTime != 0 && particleTime != 0) { //If cell and rtc agree
        Serial.println("CLOCK SOURCE: Cell and local match");
        //Can we set the GPS time??
        //Throw error
        timeGood = true;
        source = TimeSource::CELLULAR;
        if(timeSyncVals[1] != 0) throwError(CLOCK_MISMATCH | 0x08); //Throw clock mismatch error, OR with GPS indicator (only if not caused by clock unavailabilty) 
    }
    else if(abs(gpsTime - rtcTime) < maxTimeError && gpsTime != 0 && rtcTime != 0) { //If gps and rtc agree
        Serial.println("CLOCK SOURCE: GPS and local match");
        Time.setTime(gpsTime); //Set particle time from GPS time
        //Throw error
        timeGood = true;
        source = TimeSource::GPS;
        if(timeSyncVals[0] != 0) throwError(CLOCK_MISMATCH | 0x06); //Throw clock mismatch error, OR with Cell indicator (only if not caused by clock unavailabilty) 
    }
    else { //No two sources agree, very bad!
        
        if(rtcTime > 1641016800) { //Jan 1, 2022, date seems to be reeasonable //FIX!
            Serial.println("CLOCK SOURCE: Stale RTC"); //DEBUG!
            Time.setTime(rtc.getTimeUnix()); //Set time from RTC   
            timeGood = true;
            source = TimeSource::RTC;
            //Throw mismatch errors as needed //FIX!??
            // if(timeSyncVals[0] != 0) throwError(CLOCK_MISMATCH | 0x300); //Throw clock mismatch error, OR with Cell indicator (only if not caused by clock unavailabilty) 
            // if(timeSyncVals[1] != 0) throwError(CLOCK_MISMATCH | 0x200); //Throw clock mismatch error, OR with GPS indicator (only if not caused by clock unavailabilty) 
        }
        else {
            Serial.println("CLOCK SOURCE: NONE"); //DEBUG!
            criticalFault = true; //FIX??
            timeGood = false; 
            Time.setTime(946684800); //Set time back to year 2000
            source = TimeSource::NONE;
        }
        throwError(CLOCK_NO_SYNC); //Throw error regardless of which state, because no two sources are able to agree we have a sync failure 

        
    }
    if(source != TimeSource::NONE && source != TimeSource::RTC) lastTimeSync = getTime(); //If time has been sourced, update the last sync time
    else lastTimeSync = 0; //Otherwise, set back to unknown time
    timeSource = source; //Grab the time source used 
    // return false; //DEBUG!
    enableAuxPower(currentAux); //Return all to previous states
    enableI2C_Global(currentGlob);
    enableI2C_OB(currentOB);
    Serial.print("Timebase End: "); //DEBUG!
    Serial.println(millis());
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

String Kestrel::getPosAlt()
{
    if(altitude != 0) return String(altitude*(10E-4)); //Return in m 
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
    while(digitalRead(Pins::Clock_INT) == HIGH && ((millis() - timerStart) < (logPeriod*1000 + 500))){ //Wait until either timer has expired or clock interrupt has gone off //DEBUG! Give 500 ms cushion for testing RTC
        delay(1); 
        Particle.process(); //Run process continually while waiting in order to make sure device is responsive 
    } 
    if(digitalRead(Pins::Clock_INT) == LOW) return true; //If RTC triggers properly, return true, else return false 
    else return false; 
}

bool Kestrel::statLED(bool state)
{
    // bool currentGlob = enableI2C_Global(false);
	// bool currentOB = enableI2C_OB(true);
    // if(state) led.setOutput(7, On); //Turn stat on
    // else led.setOutput(7, Off); //Turn stat off
    // enableI2C_Global(currentGlob); //Reset to previous state
    // enableI2C_OB(currentOB);
    if(state) { //Assert control and set color to orange 
        RGB.control(true);
        RGB.color(0xFF,0x80,0x00); //Set to orange
    }
    else { //Release control if state is off
        RGB.control(false);
    }
    return false; //DEBUG!
}

bool Kestrel::setIndicatorState(uint8_t ledBank, uint8_t mode)
{
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);

    led.setBrightnessArray(ledBrightness); //Set all LEDs to 50% max brightness
	led.setGroupBlinkPeriod(ledPeriod); //Set blink period to specified number of ms
	led.setGroupOnTime(ledOnTime); //Set on time for each blinking period 
    led.setBrightness(3, 25); //Reduce brightness of green LEDs //DEBUG!
    led.setBrightness(5, 25);
    led.setBrightness(1, 25);
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
        case IndicatorLight::STAT:
            if(mode == IndicatorMode::PASS) {
                led.setOutput(7, Off); //Turn red off
            }
            if(mode == IndicatorMode::PREPASS) {
                led.setOutput(7, Group); //Blinking
            }
            if(mode == IndicatorMode::WAITING) {
                led.setOutput(7, Group); //Blinking
            }
            if(mode == IndicatorMode::ERROR) {
                led.setOutput(7, On); //Solid
            }
            if(mode == IndicatorMode::ERROR_CRITICAL) {
                led.setOutput(7, Group); //Blinking
            }
            break;
        case IndicatorLight::ALL:
            if(mode == IndicatorMode::WAITING) {
                // led.setOutputArray(Off); //Turn all LEDs off //DEBUG!
                // led.setBrightness(6, 50);
                // led.setBrightness(4, 50);
                // led.setBrightness(1, 50);
                for(int i = 0; i < 6; i++) led.setOutput(i, Off); //Turn off all but Stat LED
                led.setOutput(6, Group); //Set CELL amber to blink
                led.setOutput(4, Group); //Set GPS amber to blink
                led.setOutput(1, Group); //Set SENSOR amber to blink
            }
            if(mode == IndicatorMode::NONE) {
                led.setOutputArray(Off); //Turn all LEDs off 
            }
            if(mode == IndicatorMode::INIT) {
                // led.setOutputArray(Off); //Turn all LEDs off //DEBUG!
                // led.setOutput(1, Group); //Blink amber with group
                // led.setOutput(2, Group); //Blink red with group
                // led.setOutput(6, Group); //Blink amber with group
                // led.setOutput(4, Group); //Blink amber with group
                // for(int i = 0; i < 6; i++) led.setOutput(i, Off); //Turn off all but Stat LED
                led.setOutputArray(Group); //Turn all LEDs to group blink
                // for(int i = 0; i < 6; i++) led.setOutput(i, Group); //Control all but Stat LED
                led.setGroupBlinkPeriod(250); //Set to fast blinking
	            led.setGroupOnTime(25); 
            }
            if(mode == IndicatorMode::IDLE) {
                // led.setOutputArray(Group); //Turn all LEDs to group blink
                for(int i = 0; i < 6; i++) led.setOutput(i, Group); //Control all but Stat LED
                //Allow to be normal blinking 
            }
            if(mode == IndicatorMode::COMMAND) {
                // led.setOutputArray(Group); //Turn all LEDs to group blink
                for(int i = 0; i < 6; i++) led.setOutput(i, Group); //Control all but Stat LED
                led.setGroupBlinkPeriod(2000); //Set to very slow blinking
	            led.setGroupOnTime(1000);  
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
    csaAlpha.enableChannel(CH1, true);
    csaAlpha.update(); //Force new readings 
    delay(5000); //Wait for cap to discharge 
    // csaAlpha.SetCurrentDirection(CH1, BIDIRECTIONAL);
    float vBat = csaAlpha.getBusVoltage(CH1);
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
        throwError(WDT_OFF_LEASH); //Report incoming error 
        return false;
    }
}

bool Kestrel::zeroAccel(bool reset)
{
    if(reset) { //If commanded to reset, clear accel values
        accel.offset[0] = 0;
        accel.offset[1] = 0;
        accel.offset[2] = 0;
        // return false;
    }
    else {
        // for(int i = 0; i < 3; i++) {
        //     accel.offset[i] = -accel.getAccel(i); //Null each axis
        // }
        if(accel.begin() == 0) {
            // accel.offset[0] = -accel.getAccel(0); //Null each axis
            // accel.offset[1] = -accel.getAccel(1); //Null each axis
            accel.offset[0] = 0; //Null each axis
            accel.offset[1] = 0; //Null each axis
            accel.offset[2] = 1 - accel.getAccel(2); //Set z to 1 
        }
        else {
            accel.offset[0] = 0;
            accel.offset[1] = 0;
            accel.offset[2] = 0;
            //THROW ERROR!
        }
        
        // return true;
    }
    EEPROM.put(0, accel.offset[0]); //Write to long term storage
    EEPROM.put(4, accel.offset[1]);
    EEPROM.put(8, accel.offset[2]);
    return reset;
    
}

bool Kestrel::configTalonSense()
{
    Serial.println("CONFIG TALON SENSE"); //DEBUG!
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);
    csaBeta.setCurrentDirection(CH4, UNIDIRECTIONAL); //Bulk voltage, unidirectional
	csaBeta.enableChannel(CH1, false); //Disable all channels but 4
	csaBeta.enableChannel(CH2, false);
	csaBeta.enableChannel(CH3, false);
	csaBeta.enableChannel(CH4, true);
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
            // Serial.println("TIME CHANGE: Auto"); //DEBUG!
            selfPointer->syncTime(); //if time update not from manual sync (and sync not requested), call time sync to return to desired val
        }
        // if(param == time_changed_sync && selfPointer->timeSyncRequested) Serial.println("TIME CHANGE: Requested"); //DEBUG!
        // if(param == time_changed_manually) Serial.println("TIME CHANGE: Manual"); //DEBUG!
    }
}

void Kestrel::outOfMemoryHandler(system_event_t event, int param) {
    // outOfMemory = param;
    selfPointer->throwError(selfPointer->RAM_FULL); //Report RAM usage
    selfPointer->criticalFault = true; //Let WDT off leash
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

time_t Kestrel::cstToUnix(int year, int month, int day, int hour, int minute, int second)
{
    unsigned long unixDate = day - 32075 + 1461*(year + 4800 + (month - 14)/12)/4 + 367*(month - 2 - (month - 14)/12*12)/12 - 3*((year + 4900 + (month - 14)/12)/100)/4 - 2440588; //Stolen from Communications of the ACM in October 1968 (Volume 11, Number 10), Henry F. Fliegel and Thomas C. Van Flandern - offset from Julian Date. Why mess with success? 
    return unixDate*86400 + hour*3600 + minute*60 + second; //Convert unixDate to seconds, sum partial seconds from the current day

}
