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
© 2023 Regents of the University of Minnesota. All rights reserved.
******************************************************************************/

#include <Kestrel.h>

Kestrel* Kestrel::selfPointer;

Kestrel::Kestrel(ITimeProvider& timeProvider, 
                 IGpio& gpio,
                 ISystem& system,
                 IWire& wire,
                 ICloud& cloud,
                 ISerial& serialDebug,
                 ISerial& serialSdi12,
                 IIOExpander& ioOB,
                 IIOExpander& ioTalon,
                 ICurrentSenseAmplifier& csaAlpha,
                 ICurrentSenseAmplifier& csaBeta,
                 bool useSensors) : 
                 led(0x52), 
                 m_csaAlpha(csaAlpha), 
                 m_csaBeta(csaBeta), 
                 m_timeProvider(timeProvider), 
                 m_gpio(gpio), 
                 m_system(system), 
                 m_wire(wire),
                 m_cloud(cloud),
                 m_serialDebug(serialDebug),
                 m_serialSdi12(serialSdi12),
                 m_ioOB(ioOB),
                 m_ioTalon(ioTalon)
{
	// port = talonPort; //Copy to local
	// version = hardwareVersion; //Copy to local
    // talonPort = 0x0F; //Set dummy Talon port for reporting
    reportSensors = useSensors;
    sensorInterface = BusType::CORE;
}

String Kestrel::begin(time_t time, bool &criticalFault, bool &fault)
{
    selfPointer = this;
    m_system.on(IEventType::TIME_CHANGED, timechange_handler);
    m_system.on(IEventType::OUT_OF_MEMORY, outOfMemoryHandler);
    // #if defined(ARDUINO) && ARDUINO >= 100 
		// m_wire.begin();
        // m_wire.setClock(100000); //Confirm operation in fast mode
	// #elif defined(PARTICLE)
		if(!m_wire.isEnabled()) m_wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
        m_wire.setClock(400000);
	// #endif
    if(!initDone) throwError(SYSTEM_RESET | ((static_cast<uint32_t>(m_system.resetReason()) << 8) & 0xFF00)); //Throw reset error with reason for reset as subtype. Truncate resetReason to one byte. This will include all predefined reasons, but will prevent issues if user returns some large (technically can be up to 32 bits) custom reset reason
    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
    if(m_ioOB.begin() != 0) criticalFault = true;
    if(m_ioTalon.begin() != 0) criticalFault = true;
    m_ioTalon.safeMode(PCAL9535A::SAFEOFF); //DEBUG! //Turn safe mode off to speed up turn-off times for Talons
    enableAuxPower(true); //Turn on aux power 
    if(m_csaAlpha.begin() == false) { //If fails at default address, then try alt v1.9 address
        m_csaAlpha.setAddress(0x19);
        if(m_csaAlpha.begin()) boardVersion = HardwareVersion::MODEL_1v9; //If alt address works, board must be a v1.9
        //else THROW ERROR! FIX!
    }
    m_csaBeta.begin();
    m_csaAlpha.setFrequency(Frequency::SPS_64); //Set to ensure at least 24 hours between accumulator rollover 
    // m_timeProvider.delay(100); //DEBUG! For GPS
    m_ioOB.pinMode(PinsOB::LED_EN, OUTPUT);
	m_ioOB.digitalWrite(PinsOB::LED_EN, LOW); //Turn on LED indicators 
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
    //0b00000000 = SERIAL_8N1 defined in hal/inc/usart_hal.h
    m_serialSdi12.begin(1200, 0b00000000); //Initialize SDI12 serial port //DEBUG! - Used to fix wakeup issue
    // setIndicatorState(IndicatorLight::ALL,IndicatorMode::WAITING); //Set all to blinking wait
    m_gpio.pinMode(Pins::Clock_INT, IPinMode::INPUT); //Make sure interrupt pin is always an input
    if(rtc.begin(true) == 0) criticalFault = true; //Use with external oscilator, set critical fault if not able to connect 
    else {
        rtc.enableAlarm(false, 0); //Disable all alarms on startup //DEBUG! Use to prevent alarm 1 from ever being activated 
        rtc.enableAlarm(false, 1); 
        rtc.setMode(MCP79412::Mode::Normal); //Make sure to enforce normal mode
    }
    //Perform wakeup in case switched off already
    m_serialDebug.println("Wake GPS"); //DEBUG!
    m_ioOB.pinMode(PinsOB::GPS_INT, OUTPUT); //Turn GPS back on by toggling int pin
    m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
    m_timeProvider.delay(1000);
    m_ioOB.digitalWrite(PinsOB::GPS_INT, HIGH);
    m_timeProvider.delay(1000);
    m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
    m_timeProvider.delay(1000);
    if(gps.begin() == false) {
        criticalFault = true; //DEBUG! ??
        throwError(GPS_INIT_FAIL);
        m_serialDebug.println("GPS ERROR");
    }
    else {
        gps.setI2COutput(COM_TYPE_UBX);
        gps.setNavigationFrequency(1); //Produce 1 solutions per second
        gps.setAutoPVT(false); //DEBUG!
        m_serialDebug.print("GPS Stats: "); //DEBUG!
        m_serialDebug.print(gps.getNavigationFrequency());
        m_serialDebug.print("\t");
        m_serialDebug.print(gps.getMeasurementRate());
        m_serialDebug.print("\t");
        m_serialDebug.println(gps.getNavigationRate());
        m_serialDebug.print("GPS Attitude: ");
        m_serialDebug.print(gps.getATTroll());
        m_serialDebug.print("\t");
        m_serialDebug.print(gps.getATTpitch());
        m_serialDebug.print("\t\n");
        m_serialDebug.print(gps.getATTheading());
    }
    /// AUTO ZERO ACCEL
    int accelInitError = accel.begin();
    if(accelInitError == -1) {
        if(bma456.begin() == true) accelUsed = AccelType::BMA456; //If BMA456 detected, switch to that
        else throwError(ACCEL_INIT_FAIL); //If MXC6655 fails AND BMA456 fails, throw a general fail init error
        // m_serialDebug.println("MXC6655 Detect fail!"); //DEBUG!
    }
     
    if(accelInitError == 0 && accelUsed == AccelType::MXC6655) { //If accel read is good and MXC6655 is used, try zero
        int accelError = accel.updateAccelAll(); //Get updated values
        if(accelError != 0) {
            throwError(ACCEL_DATA_FAIL | (accelError << 8)); //Throw error for failure to communicate with accel, OR error code
            //FIX! Null outputs??
        }
        if(abs(accel.data[0]) < 0.04366 && abs(accel.data[1]) < 0.04366 ) zeroAccel(); //If x and y are < +/- 2.5 degrees, zero the accelerometer Z axis
        // output = output + "\"ACCEL\":[" + String(accel.data[0]) + "," + String(accel.data[1]) + "," + String(accel.data[2]) + "],"; 
    }
    else if(accelInitError != -1 && accelUsed == AccelType::MXC6655) { //If detected (not -1) but some other error, report that 
        throwError(ACCEL_DATA_FAIL | (accelInitError << 8)); //Throw error for failure to communicate with accel, OR error code 
        // output = output + "\"ACCEL\":[null],";
    }

    //Read in accel offset from EEPROM. Do this here so it is only done once per reset cycle and is immediately available 
    for(int i = 0; i < 3; i++) { 
        float temp = 0;
        EEPROM.get(i*4, temp); //Read in offset vals
        if(!isnan(temp)) accel.offset[i] = temp; //Set offset vals if real number (meaning offset has been established)
        else accel.offset[i] = 0; //If there is no existing offset, set to 0
    }
    // if(m_cloud.connected() == false) criticalFault = true; //If not connected to cell set critical error
    // if(criticalFault) setIndicatorState(IndicatorLight::STAT, IndicatorMode::ERROR_CRITICAL); //If there is a critical fault, set the stat light
    for(int i = 1; i <= numTalonPorts; i++) {
        enablePower(i, false); //Default all power to off
        enableData(i, false); //Default all data to off
    }
    initDone = true;
    syncTime(true); //Force a time sync on startup 
    // m_cloud.syncTime(); //DEBUG!
    // m_ioOB.pinMode(PinsOB::)
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
   
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
    if(reportSensors) {
        bool auxState = enableAuxPower(true); //Turn on AUX power for light sensor
        bool globState = enableI2C_Global(false); //Turn off external I2C
        bool obState = enableI2C_OB(true); //Turn on internal I2C
        String output = "\"Kestrel\":{"; //Open JSON blob
        output = output + "\"ALS\":{";
        int error = als.begin();
        if(error == 0) {
            bool readState = false;
            als.AutoRange(); //Get new values
            // m_timeProvider.delay(1000); //DEBUG!
            String alsStr[5]; 
            for(int i = 0; i < 5; i++) { //Grab values from each channel, check for error and insert nulls as appropriate 
                float val = als.GetValue(static_cast<VEML3328::Channel>(i), readState); 
                if(readState) {
                    alsStr[i] = "null"; //If error, report null
                    throwError(ALS_DATA_FAIL); //Throw error
                    readState = false; //Reset flag
                }
                else alsStr[i] = String(val);
            }
            output = output + "\"Clear\":" + alsStr[0] + ",\"Red\":" + alsStr[1] + ",\"Green\":" + alsStr[2] + ",\"Blue\":" + alsStr[3] + ",\"IR\":" + alsStr[4]; //appenbd ALS results 
            // if(readState[0] || readState[1] || readState[2] || readState[3] || readState[4]) throwError(ALS_DATA_FAIL); //If ANY of the channel reads fail, throw an error 
        }
        else {
            output = output + "\"Red\":null,\"Green\":null,\"Blue\":null,\"Clear\":null,\"IR\":null";
            throwError(ALS_INIT_FAIL | (error << 8)); //Throw error with I2C status included 
        }
        output = output + "},\"Pos\":[15]}";
        enableI2C_Global(globState); //Return to previous state
        enableI2C_OB(obState);
        enableAuxPower(auxState);
        return output;
    }
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
    unsigned long metadataStart = m_timeProvider.millis();
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
    // metadata = metadata + "\"OS\":\"" + m_system.version() + "\",";
    // metadata = metadata + "\"ID\":\"" + m_system.deviceID() + "\",";
    if(PLATFORM_ID == PLATFORM_BSOM) metadata = metadata + "\"Model\":\"BSoM\","; //Report BSoM
    else if (PLATFORM_ID == PLATFORM_B5SOM) metadata = metadata + "\"Model\":\"B5SoM\","; //Report B5SoM
    else metadata = metadata + "\"Model\":null,"; //Report null if for some reason the firmware is running on another device 

    if(boardVersion == HardwareVersion::PRE_1v9) metadata = metadata + "\"Hardware\":\"<v1.9\",";
    if(boardVersion == HardwareVersion::MODEL_1v9) metadata = metadata + "\"Hardware\":\"v1.9\",";
    ///////// ADD SHT40!

	
	metadata = metadata + "\"Firmware\":\"v" + FIRMWARE_VERSION + "\","; //Report firmware version as modded BCD
	metadata = metadata + "\"Pos\":[15]"; //Concatonate position 
	metadata = metadata + "}"; //CLOSE  
    if((m_timeProvider.millis() - metadataStart) > loggerCollectMax) throwError(EXCEED_COLLECT_TIME | 0x300 | portErrorCode); //Throw error for metadata taking too long
    enableAuxPower(auxState); //Return to previous state
    enableI2C_Global(globState); 
    enableI2C_OB(obState);
	return metadata; 
	// return ""; //DEBUG!
}

String Kestrel::selfDiagnostic(uint8_t diagnosticLevel, time_t time)
{
    unsigned long diagnosticStart = m_timeProvider.millis(); //Keep track of when the test starts  
    bool globState = enableI2C_Global(false); //Turn off external I2C
    bool obState = enableI2C_OB(true); //Turn on internal I2C
	String output = "\"Kestrel\":{";
	if(diagnosticLevel == 0) {
		//TBD
	}

	if(diagnosticLevel <= 1) {
		//TBD
	}

	if(diagnosticLevel <= 2) {
		//TBD
        if(accelUsed == AccelType::MXC6655) output = output + "\"Accel_Offset\":[" + String(accel.offset[0]) + "," + String(accel.offset[1]) + "," + String(accel.offset[2]) + "],"; 
        if(accelUsed == AccelType::BMA456) output = output + "\"Accel_Offset\":[0,0,0],"; 
        uint8_t rtcConfigA = (rtc.readByte(0) & 0x80); //Read in ST bit
        rtcConfigA = rtcConfigA | ((rtc.readByte(3) & 0x38) << 1); //Read in OSCRUN, PWRFAIL, VBATEN bits
        uint8_t rtcConfigB = rtc.readByte(7); //Read in control byte
        uint8_t rtcConfigC = rtc.readByte(8); //Read in trim byte
        uint8_t rtcConfigD = rtc.readByte(0x0D); //Read in ALARM0 reg
        uint8_t rtcConfigE = rtc.readByte(0x14); //Read in ALARM1 reg
        output = output + "\"RTC_Config\":[" + String(rtcConfigA) + "," + String(rtcConfigB) + "," + String(rtcConfigC) + "," + String(rtcConfigD) + "," + String(rtcConfigE) + "],"; //Concatonate to output
	}

	if(diagnosticLevel <= 3) {
		//TBD
        m_wire.beginTransmission(0x6F);
        uint8_t rtcError = m_wire.endTransmission();
        if(rtcError == 0) {
            time_t currentTime = rtc.getTimeUnix();
            m_timeProvider.delay(1200); //Wait at least 1 second (+20%)
            if((rtc.getTimeUnix() - currentTime) == 0) throwError(RTC_OSC_FAIL); //If rtc is not incrementing, throw error 
        }
        else throwError(RTC_READ_FAIL | rtcError << 8); //Throw error since unable to communicate with RTC

        //GRAB TTFF FROM GPS
        enableAuxPower(true); //Make sure power is applied to GPS
        uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes. MAX_PAYLOAD_SIZE defaults to 256. The CFG_RATE payload is only 6 bytes!
        gps.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);
        ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
        customCfg.cls = UBX_CLASS_NAV; // This is the message Class
        customCfg.id = UBX_NAV_STATUS; // This is the message ID
        customCfg.len = 0; // Setting the len (length) to zero let's us poll the current settings
        customCfg.startingSpot = 0; // Always set the startingSpot to zero (unless you really know what you are doing)
        uint16_t maxWait = 1500; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)
        if (gps.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) {
            m_serialDebug.println("GPS READ FAIL"); //DEBUG!
            throwError(GPS_READ_FAIL); // We are expecting data and an ACK, throw error otherwise
        }
        unsigned long ttff = 0;
        for(int i = 0; i < 4; i++) ttff = ttff | (customPayload[8 + i] << 8*i); //Concatonate the 4 bytes of the TTFF value
        if(customPayload[4] >= 2 && customPayload[4] <= 4) output = output + "\"TTFF\":" + String(ttff) + ","; //Append TTFF
        else output = output + "\"TTFF\":null,"; //If no fix, append null
        // m_serialDebug.print("GPS UTC Seconds: "); //DEBUG!
        // m_serialDebug.println(customPayload[18]);
        // m_serialDebug.print("GPS UTC Validity: "); //DEBUG!
        // m_serialDebug.println(customPayload[19], HEX);
 	}

	if(diagnosticLevel <= 4) {
        static time_t lastAccReset = 0; //Grab time that accumulators were reset. Set to 0 on restart
        m_ioOB.digitalWrite(PinsOB::CSA_EN, HIGH); //Enable CSA GPIO control
        bool initA = m_csaAlpha.begin();
        bool initB = m_csaBeta.begin();
		if(initA == true || initB == true) { //Only proceed if one of the ADCs connects correctly
			// adcSense.SetResolution(18); //Set to max resolution (we paid for it right?) 
            //Setup CSAs
            if(initA == true) {
                m_csaAlpha.enableChannel(Channel::CH1, true); //Enable all channels
                m_csaAlpha.enableChannel(Channel::CH2, true);
                m_csaAlpha.enableChannel(Channel::CH3, true);
                m_csaAlpha.enableChannel(Channel::CH4, true);
                m_csaAlpha.setCurrentDirection(Channel::CH1, BIDIRECTIONAL);
                m_csaAlpha.setCurrentDirection(Channel::CH2, UNIDIRECTIONAL);
                m_csaAlpha.setCurrentDirection(Channel::CH3, UNIDIRECTIONAL);
                m_csaAlpha.setCurrentDirection(Channel::CH4, UNIDIRECTIONAL);
            }

            if(initB == true) {
                m_csaBeta.enableChannel(Channel::CH1, true); //Enable all channels
                m_csaBeta.enableChannel(Channel::CH2, true);
                m_csaBeta.enableChannel(Channel::CH3, true);
                m_csaBeta.enableChannel(Channel::CH4, true);
                m_csaBeta.setCurrentDirection(Channel::CH1, UNIDIRECTIONAL);
                m_csaBeta.setCurrentDirection(Channel::CH2, UNIDIRECTIONAL);
                m_csaBeta.setCurrentDirection(Channel::CH3, UNIDIRECTIONAL);
                m_csaBeta.setCurrentDirection(Channel::CH4, UNIDIRECTIONAL);
            }
			output = output + "\"PORT_V\":["; //Open group
			// ioSense.digitalWrite(pinsSense::MUX_SEL2, LOW); //Read voltages
            if(initA == true) {
                // m_csaAlpha.enableChannel(Channel::CH1, true); //Enable all channels
                // m_csaAlpha.enableChannel(Channel::CH2, true);
                // m_csaAlpha.enableChannel(Channel::CH3, true);
                // m_csaAlpha.enableChannel(Channel::CH4, true);
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    bool err = false;
                    float val = m_csaAlpha.getBusVoltage(Channel::CH1 + i, true, err); //Get bus voltage with averaging 
                    if(!err) output = output + String(val, 6); //If no error, report as normal
                    else {
                        throwError(CSA_OB_READ_FAIL | 0xA00); //Throw read error for CSA A
                        output = output + "null"; //Otherwise append null
                    }
                    output = output + ","; //Append comma 
                }
            }
            else output = output + "null,null,null,null,"; //Append nulls if can't connect to csa alpha

			if(initB == true) {
                
                // m_timeProvider.delay(1000); //Wait for new data //DEBUG!
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    bool err = false;
                    float val = m_csaBeta.getBusVoltage(Channel::CH1 + i, true, err); //Get bus voltage with averaging 
                    if(!err) output = output + String(val, 6); //If no error, report as normal
                    else {
                        throwError(CSA_OB_READ_FAIL | 0xB00); //Throw read error for CSA B
                        output = output + "null"; //Otherwise append null
                    }
                    if(i < 3) output = output + ","; //Append comma if not the last reading
                }
            }
            else {
                output = output + "null,null,null,null"; //Append nulls if can't connect to csa beta
                throwError(CSA_OB_INIT_FAIL | 0xB00); //Throw error for ADC beta failure
            }

			output = output + "],"; //Close group
			output = output + "\"PORT_I\":["; //Open group
            if(initA == true) {
                // m_csaAlpha.enableChannel(Channel::CH1, true); //Enable all channels
                // m_csaAlpha.enableChannel(Channel::CH2, true);
                // m_csaAlpha.enableChannel(Channel::CH3, true);
                // m_csaAlpha.enableChannel(Channel::CH4, true);
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    bool err = false;
                    float val = m_csaAlpha.getCurrent(Channel::CH1 + i, true, err); //Get current with averaging
                    if(!err) output = output + String(val, 6); //If no error, report as normal
                    else {
                        throwError(CSA_OB_READ_FAIL | 0xA00); //Throw read error for CSA A
                        output = output + "null"; //Otherwise append null
                    }
                    output = output + ","; //Append comma 
                }
            }
            else {
                output = output + "null,null,null,null,"; //Append nulls if can't connect to csa alpha
                throwError(CSA_OB_INIT_FAIL | 0xA00); //Throw error for ADC failure
            }

			if(initB == true) {
                // m_csaBeta.enableChannel(Channel::CH1, true); //Enable all channels
                // m_csaBeta.enableChannel(Channel::CH2, true);
                // m_csaBeta.enableChannel(Channel::CH3, true);
                // m_csaBeta.enableChannel(Channel::CH4, true);
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    bool err = false;
                    float val = m_csaBeta.getCurrent(Channel::CH1 + i, true, err); //Get current with averaging
                    if(!err) output = output + String(val, 6); //If no error, report as normal
                    else {
                        throwError(CSA_OB_READ_FAIL | 0xB00); //Throw read error for CSA B
                        output = output + "null"; //Otherwise append null
                    }
                    if(i < 3) output = output + ","; //Append comma if not the last reading
                }
            }
            else {
                output = output + "null,null,null,null"; //Append nulls if can't connect to csa beta
                throwError(CSA_OB_INIT_FAIL | 0xB00); //Throw error for ADC failure
            }
			output = output + "],"; //Close group
            output = output + "\"AVG_P\":["; //Open group
            if(lastAccReset == 0) { //If unknown time since last reset, clear accumulators on csa Alpha
                m_csaAlpha.update(true); 
                lastAccReset = getTime(); //Update time of reset
            }
            if(initA == true) {
                for(int i = 0; i < 4; i++){ //Increment through all ports
                    bool err = false;
                    float val = m_csaAlpha.getPowerAvg(Channel::CH1 + i, err); //Get bus power
                    if(!err) output = output + String(val); //If no error, report as normal
                    else {
                        throwError(CSA_OB_READ_FAIL | 0xA00); //Throw read error for CSA A
                        output = output + "null"; //Otherwise append null
                    }
                    if(i < 3) output = output + ","; //Append comma if not the last reading
                }
            }
            else {
                output = output + "null,null,null,null"; //Append nulls if can't connect to csa alpha
                throwError(CSA_OB_INIT_FAIL | 0xA00); //Throw error for ADC failure
            }
            output = output + "],"; //Close group
            output = output + "\"LAST_CLR\":" + String((int)lastAccReset) + ","; //Append the time of the last accumulator clear
            if((getTime() - lastAccReset) > 86400 && (getTime() % 86400) < 3600) { //If it is zero hour in UTC and it has been more than 24 hours since the last reset, clear accumulators 
                m_csaAlpha.update(true); 
                lastAccReset = getTime(); //Update time of reset
            }
			
		}
		else { //If unable to initialzie ADC
			output = output + "\"PORT_V\":[null],\"PORT_I\":[null],\"AVG_P\":[null],";
			throwError(CSA_OB_INIT_FAIL); //Throw error for global CSA failure
		}
        output = output + "\"ALS\":";
        int error = als.begin();
        if(error == 0) {
            als.AutoRange();
            output = output + String(als.GetLux()) + ","; //appenbd ALS results 
        }
        else {
            output = output + "null,";
            throwError(ALS_INIT_FAIL | (error << 8)); //Throw error with I2C status included 
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

        if(accelUsed == AccelType::MXC6655) { //If MXC6655 is used, proceed with reading
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
                throwError(ACCEL_DATA_FAIL); //Throw error for failure to communicate with accel
                output = output + "\"ACCEL\":[null],";
                temperatureString = temperatureString + "null";
            }
        }
        else if (accelUsed == AccelType::BMA456) { //If BMA456 is used, proceed with reading
            bool bma456Present = bma456.begin();
            float x = 0, y = 0, z = 0;
            int32_t temp = 0;
            bma456.initialize();
            for(int i = 0; i < 5; i++) { //FIX! DEBUG!
                bma456.getAcceleration(&x, &y, &z);
                m_timeProvider.delay(10);
            }
            
            temp = bma456.getTemperature();

            if(bma456Present) { //FIX! Check directly for failure instead of implied failure by presence or abscence 
                output = output + "\"ACCEL\":[" + String(x/1000.0) + "," + String(y/1000.0) + "," + String(z/1000.0) + "],"; 
                temperatureString = temperatureString + String(temp);
            }
            else {
                output = output + "\"ACCEL\":[null,null,null],"; 
                temperatureString = temperatureString + "null";
            }
        }

		// ioSense.digitalWrite(pinsSense::MUX_EN, HIGH); //Turn MUX back off 
		// m_gpio.digitalWrite(KestrelPins::PortBPins[talonPort], LOW); //Return to default external connecton
        temperatureString = temperatureString + "]";
        output = output + "\"SIV\":" + String(gps.getSIV()) + ",\"FIX\":" + String(gps.getFixType()) + ",";
		output = output + temperatureString + ","; 
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;

	}

	if(diagnosticLevel <= 5) {
		// output = output + "\"lvl-5\":{"; //OPEN JSON BLOB
        if(m_system.freeMemory() < 15600) { //Throw error if RAM usage >90% //FIX! Check dynamically for amount of RAM available based on OS, etc 
            throwError(RAM_CRITICAL); 
            criticalFault = true; //Let WDT off leash to fix issue
        }
        else if(m_system.freeMemory() < 46800) throwError(RAM_LOW); //Throw error if RAM usage >75% //FIX! Check dynamically for amount of RAM available based on OS, etc 
        output = output + "\"Free Mem\":" + String(m_system.freeMemory()) + ","; //DEBUG! Move to higher level later on
        output = output + "\"Time Fix\":" + String(timeFix) + ","; //Append time sync value
        output = output + "\"Time Source\":[\"" + sourceNames[timeSourceA] + "\",\"" + sourceNames[timeSourceB] + "\"],"; //Report the time souce selected from the last sync
        output = output + "\"Times\":{\"LOCAL\":" + String((int)times[numClockSources - 1]) + ","; //Always have current time listed 
        for(int i = 0; i < numClockSources - 1; i++) {
            if(sourceRequested[i] == true) { //Only report the clock sources which were requested 
                if(sourceAvailable[i] == true) {
                    output = output + "\"" + sourceNames[i] + "\":" + String((int)times[i]) + ","; //If result is valid, append number
                }
                else output = output + "\"" + sourceNames[i] + "\":null,"; //If result not valid, append null
            }
        }
        if(output.endsWith(",")) output.remove(output.length() - 1); //Trim trailing comma if needed
        output = output + "},"; //Close blob
        // output = output + "\"Times\":[" + String((int)timeSyncVals[0]) + "," + String((int)timeSyncVals[1]) + "," + String((int)timeSyncVals[2]) + "],"; //Add reported times from last sync
        if(lastTimeSync > 0) output = output + "\"Last Sync\":" + String((int)lastTimeSync) + ",";
        else output = output + "\"Last Sync\":null,";
        output = output + "\"OB\":" + m_ioOB.readBus() + ",\"Talon\":" + m_ioTalon.readBus() + ","; //Report the bus readings from the IO expanders
        output = output + "\"I2C\":["; //Append identifer 
        for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
            m_wire.beginTransmission(adr);
            // m_wire.write(0x00);
            int error = m_wire.endTransmission();
            // if(adr == 0) { //DEBUG!
            //     // m_serialDebug.print("Zero Error: ");
            //     // m_serialDebug.println(error); 
            // }
            if(error == 0) {
                output = output + String(adr) + ",";
            }
            m_timeProvider.delay(1); //DEBUG!
        }
        if(output.substring(output.length() - 1).equals(",")) {
            output = output.substring(0, output.length() - 1); //Trim trailing ',' if present
        }
        output = output + "],"; //Close array
	}
    if((m_timeProvider.millis() - diagnosticStart) > loggerCollectMax) throwError(EXCEED_COLLECT_TIME | 0x200 | portErrorCode); //Throw error for diagnostic taking too long
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
	return output + "\"Pos\":[15]}"; //Write position in logical form - Return compleated closed output
}

bool Kestrel::updateLocation(bool forceUpdate) 
{
    bool status = false;
    if(updateGPS || forceUpdate) {
        bool globState = enableI2C_Global(false); //Turn off external I2C
        bool obState = enableI2C_OB(true); //Turn on internal I2C
        enableAuxPower(true); //Turn on aux power 
        m_serialDebug.print("PVT Response: "); //DEBUG!
        m_serialDebug.println(gps.getPVT());
        // gps.getPVT(); //Force updated call //DEBUG!
        if(gps.getPVT() && gps.getFixType() >= 2 && gps.getFixType() <= 4 && gps.getGnssFixOk()) { //Only update if GPS has at least a 2D fix
            m_serialDebug.println("UPDATE GPS"); //DEBUG!
            longitude = gps.getLongitude();
            latitude = gps.getLatitude();
            altitude = gps.getAltitude();
            posTime = getTime(); //Update time that GPS measure was made
            updateGPS = false; //Clear flag when done
            status = true;
        }
        else {
            throwError(GPS_UNAVAILABLE); //If no fix available, throw error
            status = false;
        }
        enableI2C_Global(globState); //Return to previous state
        enableI2C_OB(obState);
    }
    return status;
}

bool Kestrel::connectToCell()
{
    //FIX! Check for cell module on, etc
    m_cloud.connect();
    m_system.waitForCondition([this]() { return m_cloud.connected(); }, std::chrono::milliseconds(CELL_TIMEOUT)); //Wait for cell to connect
    if(m_cloud.connected()) return true;
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
        // m_wire.reset(); //DEBUG!
        m_ioTalon.pinMode(PinsTalon::EN[port - 1], OUTPUT);
        m_ioTalon.digitalWrite(PinsTalon::EN[port - 1], state);
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
        // m_wire.reset(); //DEBUG!
        // m_ioTalon.pinMode(PinsTalon::SEL[port - 1], OUTPUT);
        // m_ioTalon.digitalWrite(PinsTalon::SEL[port - 1], LOW); //DEBUG!
        m_ioTalon.pinMode(PinsTalon::I2C_EN[port - 1], OUTPUT);
        m_ioTalon.digitalWrite(PinsTalon::I2C_EN[port - 1], state);
        // m_serialDebug.println(PinsTalon::I2C_EN[port - 1]); //DEBUG!
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
        // m_wire.reset(); //DEBUG!
        m_ioTalon.pinMode(PinsTalon::SEL[port - 1], OUTPUT);
        m_ioTalon.digitalWrite(PinsTalon::SEL[port - 1], sel); //DEBUG!
        // m_ioTalon.pinMode(PinsTalon::I2C_EN[port - 1], OUTPUT);
        // m_ioTalon.digitalWrite(PinsTalon::I2C_EN[port - 1], state);
        // m_serialDebug.println(PinsTalon::I2C_EN[port - 1]); //DEBUG!
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
        if(m_ioTalon.digitalRead(PinsTalon::EN[port - 1]) == HIGH) state = false; //If fault line is high, return false for no fault 
        else state = true; //If there is a read failure or otherwise unable to read, assume a fault
        enableI2C_Global(globState); //Return to previous state
        enableI2C_OB(obState);
        return state;
    }
    return true;
    
}

bool Kestrel::enableI2C_OB(bool state)
{
    bool currentState = m_gpio.digitalRead(Pins::I2C_OB_EN); 
    m_gpio.pinMode(Pins::I2C_OB_EN, IPinMode::OUTPUT);
	m_gpio.digitalWrite(Pins::I2C_OB_EN, state);
    // m_wire.reset(); //DEBUG!
    return currentState; 
}

bool Kestrel::enableI2C_Global(bool state)
{
    bool currentState = m_gpio.digitalRead(Pins::I2C_GLOBAL_EN); 
    m_gpio.pinMode(Pins::I2C_GLOBAL_EN, IPinMode::OUTPUT);
	m_gpio.digitalWrite(Pins::I2C_GLOBAL_EN, state);
    // m_wire.reset(); //DEBUG!
    return currentState; 
}

bool Kestrel::enableI2C_External(bool state)
{
    bool globState = enableI2C_Global(false);
	bool obState = enableI2C_OB(true);
	//Turn on external I2C port
    bool currentState = m_ioOB.digitalRead(PinsOB::I2C_EXT_EN);
	m_ioOB.pinMode(PinsOB::I2C_EXT_EN, OUTPUT);
	m_ioOB.digitalWrite(PinsOB::I2C_EXT_EN, state);
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

bool Kestrel::enableSD(bool state)
{
    bool globState = enableI2C_Global(false);
	bool obState = enableI2C_OB(true);
    bool currentState = m_ioOB.digitalRead(PinsOB::SD_EN);
    if(state) {
        enableAuxPower(true); //Make sure Aux power is on
        m_ioOB.pinMode(PinsOB::SD_EN, OUTPUT);
        m_ioOB.digitalWrite(PinsOB::SD_EN, HIGH);
    }
    else if(!state) {
        m_ioOB.pinMode(PinsOB::SD_EN, OUTPUT);
        m_ioOB.digitalWrite(PinsOB::SD_EN, LOW);
    }
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    return currentState; //DEBUG! How to return failure? Don't both and just throw error??
}

bool Kestrel::sdInserted()
{
    m_ioOB.pinMode(PinsOB::SD_CD, INPUT_PULLUP);
    if(m_ioOB.digitalRead(PinsOB::SD_CD) == LOW) return true; //If switch is closed, return true
    else return false; //Otherwise it is not inserted 
}

bool Kestrel::enableAuxPower(bool state)
{
    bool globState = enableI2C_Global(false);
	bool obState = enableI2C_OB(true);
    bool currentState = m_ioOB.digitalRead(PinsOB::AUX_EN);
    m_ioOB.pinMode(PinsOB::AUX_EN, OUTPUT);
    m_ioOB.digitalWrite(PinsOB::AUX_EN, state); 
    enableI2C_Global(globState); //Return to previous state
    enableI2C_OB(obState);
    return currentState; //DEBUG! How to return failure? Don't both and just throw error??
}

uint8_t Kestrel::updateTime()
{
    // m_serialDebug.print("Current Timer Period: "); //DEBUG!
    // m_serialDebug.println(logPeriod);
    static uint8_t timeSource = syncTime();
    static time_t lastRunTime = m_timeProvider.millis();

    if((m_timeProvider.millis() - lastRunTime) > 60000) { //Only sync time if it has been more than 60 seconds since last synchronization
        timeSource = syncTime(); 
        lastRunTime = m_timeProvider.millis();
    }
    // if(m_timeProvider.isValid()) { //If particle time is valid, set from this
    currentDateTime.source = timeSource; //sync time and record source of current time
    currentDateTime.year = m_timeProvider.year();
    currentDateTime.month = m_timeProvider.month();
    currentDateTime.day = m_timeProvider.day();
    currentDateTime.hour = m_timeProvider.hour();
    currentDateTime.minute = m_timeProvider.minute();
    currentDateTime.second = m_timeProvider.second();
    
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

uint8_t Kestrel::syncTime(bool force)
{
    //Synchronize time across GPS, Cell and RTC
    m_serialDebug.println("TIME SYNC!"); //DEBUG!
    // Timestamp t = getRawTime(); //Get updated time
    bool currentAux = enableAuxPower(true);
    bool currentGlob = enableI2C_Global(false);
    bool currentOB = enableI2C_OB(true);
    
    // bool timeGood = true; //Result of time testing to see if time matches, assume the time is valid to start with 
    timeGood = true; //Assume good and clear if any deviation 

    time_t gpsTime = 0;
    time_t gpsSatTime = 0;
    time_t cellTime = 0;
    time_t rtcTime = 0;
    
    static time_t previousTime = 0; //Init to current time to throw warnings on startup, etc
    static unsigned long previousMillis = 0; //Init to current time to throw warnings on startup, etc
    
    
    //Grab Particle RTC time and expected time from millis delta
    time_t particleTime = m_timeProvider.isValid() ? m_timeProvider.now() : 0; //Set to current time if valid, if not, set to 0
    times[numClockSources - 1] = particleTime; //Grab current particle RTC time

    if(particleTime == 0) throwError(CLOCK_UNAVAILABLE); //If time is not valid, throw system wide error for base clock
    
    
    m_serialDebug.print("Timebase Start: "); //DEBUG!
    m_serialDebug.println(m_timeProvider.millis());

    /////////// RTC TIME //////////////
    m_wire.beginTransmission(0x6F); //Check for presence of RTC //FIX! Find a better way to test if RTC time is available 
    uint8_t rtcError = m_wire.endTransmission();
    sourceRequested[TimeSource::RTC] = true;
    if(rtcError != 0) {
        sourceAvailable[TimeSource::RTC] = false;
        times[TimeSource::RTC] = 0; //Clear if unable to connect to RTC
        throwError(CLOCK_UNAVAILABLE | 0x05); //OR with RTC indicator 
    }
    else { //Only read values in if able to connect to RTC
        rtcTime = rtc.getTimeUnix();
        m_serialDebug.print("RTC Time: ");
        m_serialDebug.println(rtcTime); //DEBUG!
        m_serialDebug.print("Particle Time: ");
        m_serialDebug.println(cellTime);  
        sourceAvailable[TimeSource::RTC] = true;
        times[TimeSource::RTC] = rtcTime; //Grab last time
    }
    /////////// INCREMENT TIME ///////////////////
    unsigned long deltaTime = m_timeProvider.millis() - previousMillis; //Calculate delta time since last call
    deltaTime = deltaTime/1000; //Convert to seconds - Do this as seperate process to make sure rollover math works correclty 
    sourceRequested[TimeSource::INCREMENT] = true; 
    if(previousTime == 0 || previousMillis == 0) sourceAvailable[TimeSource::INCREMENT] = false; //If set for the first time or not incremented, ignore
    else sourceAvailable[TimeSource::INCREMENT] = true;
    times[TimeSource::INCREMENT] = previousTime + deltaTime; //The expected time is the delta added to the last time recorded 
    
    /////////// CELL TIME //////////////////
    sourceRequested[TimeSource::CELLULAR] = true;
    if(m_cloud.connected()) { //Only enter if there is not already a sync pending 
        timeSyncRequested = true;
        m_cloud.syncTime();
        // m_system.waitForCondition(m_cloud.syncTimePending, 500); //Wait up to 0.5 seconds for system to assert a syncTime
        // m_system.waitForCondition(m_cloud.syncTimeDone, 10000); //Wait until sync is done, at most 10 seconds //FIX!
        // unsigned long localTime = m_timeProvider.millis();
        // while(m_cloud.syncTimePending() && (m_timeProvider.millis() - localTime) < 10000) m_cloud.process(); //Process command while waiting for sync to finish or timeout 
        // m_system.waitForCondition(m_cloud.syncTimePending, 10000); //Wait until sync is done, at most 10 seconds
        m_system.waitForCondition([this]() {return m_cloud.syncTimePending();}, std::chrono::milliseconds(500)); //Wait up to 0.5 seconds for system to assert a syncTime
        m_system.waitForCondition([this]() {return m_cloud.syncTimeDone();}, std::chrono::milliseconds(10000)); //Wait until sync is done, at most 10 seconds //FIX!
        if(m_cloud.syncTimeDone()) { //Make sure sync time was actually completed 
            m_timeProvider.zone(0); //Set to UTC 
            cellTime = m_timeProvider.now();
            // timeSyncRequested = false; //Release control of time sync override 
            m_serialDebug.print("Cell Time: "); 
            m_serialDebug.println(cellTime);
            sourceAvailable[TimeSource::CELLULAR] = true; 
            times[TimeSource::CELLULAR] = m_timeProvider.now(); //Grab last time
        }
        else {
            sourceAvailable[TimeSource::CELLULAR] = false;
            times[TimeSource::CELLULAR] = 0;
            throwError(CLOCK_UNAVAILABLE | 0x106); //OR with Cell indicator 
        }
        // timeSyncRequested = false; //Release control of time sync override 
        
    }
    else {
        sourceAvailable[TimeSource::CELLULAR] = false;
        times[TimeSource::CELLULAR] = 0; //Clear if not updated
        throwError(CLOCK_UNAVAILABLE | 0x06); //OR with Cell indicator 
    }

    ////////// GPS TIME ///////////////////
    uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes. MAX_PAYLOAD_SIZE defaults to 256. The CFG_RATE payload is only 6 bytes!
    // bool currentAux = enableAuxPower(true);
    // bool currentGlob = enableI2C_Global(false);
    // bool currentOB = enableI2C_OB(true);
    //Perform wakeup in case switched off already
    m_serialDebug.println("Wake GPS"); //DEBUG!
    m_ioOB.pinMode(PinsOB::GPS_INT, OUTPUT); //Turn GPS back on by toggling int pin
    m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
    m_timeProvider.delay(1000);
    m_ioOB.digitalWrite(PinsOB::GPS_INT, HIGH);
    m_timeProvider.delay(1000);
    m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
    m_timeProvider.delay(1000);
    // gps.begin();
    if(gps.begin() == false) {
        // throwError(GPS_INIT_FAIL); //DEBUG!
        criticalFault = true; //Set critical fault since we can't init GPS
        // timeSyncVals[1] = 0; //Clear if not updated
        throwError(GPS_INIT_FAIL); //Report failure to connect with GPS AND that GPS clock is not available 
        throwError(CLOCK_UNAVAILABLE | 0x08); //OR with GPS indicator 
        m_serialDebug.println("GPS FAIL"); //DEBUG!
    }
    // if(false); //DEBUG!
    else {
    
    // if(gps.begin() == false) throwError(GPS_INIT_FAIL);
    // else {
        sourceRequested[TimeSource::GPS] = true;
        gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
        
        gps.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);
        ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
        customCfg.cls = UBX_CLASS_NAV; // This is the message Class
        // customCfg.id = UBX_NAV_TIMELS; // This is the message ID
        customCfg.id = UBX_NAV_TIMEUTC; // This is the message ID
        customCfg.len = 0; // Setting the len (length) to zero let's us poll the current settings
        customCfg.startingSpot = 0; // Always set the startingSpot to zero (unless you really know what you are doing)
        uint16_t maxWait = 1500; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)
        if (gps.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) {
            m_serialDebug.println("GPS READ FAIL"); //DEBUG!
            throwError(GPS_READ_FAIL); // We are expecting data and an ACK, throw error otherwise
        }
        // m_serialDebug.print("GPS UTC Seconds: "); //DEBUG!
        // m_serialDebug.println(customPayload[18]);
        m_serialDebug.print("GPS UTC Validity: "); //DEBUG!
        m_serialDebug.println(customPayload[19], HEX);

        // if((customPayload[19] & 0x0F) == 0x07) { //Check if all times are valid
        //     // struct tm timeinfo = {0}; //Create struct in C++ time land

        //     // timeinfo.tm_year = (customPayload[12] | (customPayload[13] << 8)) - 1900; //Years since 1900
        //     // timeinfo.tm_mon = customPayload[14] - 1; //Months since january
        //     // timeinfo.tm_mday = customPayload[15];
        //     // timeinfo.tm_hour = customPayload[16];
        //     // timeinfo.tm_min = customPayload[17];
        //     // timeinfo.tm_sec = customPayload[18];
        //     // gpsTime = timegm(&timeinfo); //Convert struct to unix time
        //     gpsTime = cstToUnix((customPayload[12] | (customPayload[13] << 8)), customPayload[14], customPayload[15], customPayload[16], customPayload[17], customPayload[18]); //Convert current time to Unix time
        //     // gpsTime = 0xDEADBEEF; //DEBUG!
        //     m_serialDebug.print("GPS Time: ");
        //     m_serialDebug.println(gpsTime); //DEBUG!
        //     // timeSyncVals[1] = gpsTime; //Grab last time
        //     times[TimeSource::GPS_RTC] = gpsTime;
        //     sourceAvailable[TimeSource::GPS_RTC] = true;
        // }
        // else {
        //     // timeSyncVals[1] = 0; //Clear if not updated
        //     throwError(CLOCK_UNAVAILABLE | 0x08 | (customPayload[19] << 8)); //OR with GPS indicator, OR with validity payload 
        // }
        // m_serialDebug.print("GPS Leap Seconds: "); //DEBUG!
        // m_serialDebug.println(customPayload[9]);
        
        // m_serialDebug.print("GPS Time: "); //DEBUG!
        // m_serialDebug.print(gps.getHour());
        // m_serialDebug.print(":");
        // m_serialDebug.print(gps.getMinute());
        // m_serialDebug.print(":");
        // m_serialDebug.println(gps.getSecond());
    }
    // if(gps.getDateValid() && gps.getTimeValid() && gps.getTimeFullyResolved()) {
    uint8_t fixType = gps.getFixType();
    bool gnssFix = gps.getGnssFixOk();
    if((customPayload[19] & 0x0F) == 0x07 && (fixType >= 2 && fixType <= 4 && gnssFix)) { //Check if all times are valid AND fix is valid
        // struct tm timeinfo = {0}; //Create struct in C++ time land

        // timeinfo.tm_year = (customPayload[12] | (customPayload[13] << 8)) - 1900; //Years since 1900
        // timeinfo.tm_mon = customPayload[14] - 1; //Months since january
        // timeinfo.tm_mday = customPayload[15];
        // timeinfo.tm_hour = customPayload[16];
        // timeinfo.tm_min = customPayload[17];
        // timeinfo.tm_sec = customPayload[18];
        // gpsTime = timegm(&timeinfo); //Convert struct to unix time
        gpsTime = cstToUnix((customPayload[12] | (customPayload[13] << 8)), customPayload[14], customPayload[15], customPayload[16], customPayload[17], customPayload[18]); //Convert current time to Unix time
        m_serialDebug.print("GPS Time: ");
        m_serialDebug.println(gpsTime); //DEBUG!
        sourceAvailable[TimeSource::GPS] = true;
        sourceAvailable[TimeSource::GPS_RTC] = true; //By default
        times[TimeSource::GPS] = gpsTime; //Grab last time
        times[TimeSource::GPS_RTC] = gpsTime; //Grab last time
    }
    else if((customPayload[19] & 0x0F) == 0x07 && !(fixType >= 2 && fixType <= 4 && gnssFix)) { //RTC is good, but not active fix
        gpsTime = cstToUnix((customPayload[12] | (customPayload[13] << 8)), customPayload[14], customPayload[15], customPayload[16], customPayload[17], customPayload[18]); //Convert current time to Unix time
        m_serialDebug.print("GPS RTC Time: ");
        m_serialDebug.println(gpsTime); //DEBUG!
        sourceAvailable[TimeSource::GPS] = false;
        sourceAvailable[TimeSource::GPS_RTC] = true;
        times[TimeSource::GPS_RTC] = gpsTime; //Grab last time
    }
    else {
        sourceAvailable[TimeSource::GPS] = false;
        sourceAvailable[TimeSource::GPS_RTC] = false;
        times[TimeSource::GPS]  = 0; //Clear if not updated
        times[TimeSource::GPS_RTC]  = 0;
        throwError(CLOCK_UNAVAILABLE | 0x08 | (customPayload[19] << 8)); //OR with GPS indicator, OR with validity payload 
    }

    ////////////////////////////////// TEST VALIDITY OF CURRENT TIME ////////////////////////////////////////////
    for(int i = 0; i < numClockSources; i++) {
        if(sourceAvailable[i] == true && abs(particleTime - times[i]) > maxTimeError) {
            throwError(TIME_DISAGREE);
            timeGood = false; //Clear flag if any of the available times disagree with the current time
        }
    }

    /////////////////////////// EVALUATE TIME FIX FROM TIME SET ////////////////////////////////
    uint8_t newTimeFix = 0;
    if(timeGood) { //Only do if time is alredy valid
        if(sourceAvailable[TimeSource::GPS] == true && sourceAvailable[TimeSource::CELLULAR] == true) newTimeFix = 4; //If both remote time sources are present, best fix
        else if(sourceAvailable[TimeSource::GPS] == true || sourceAvailable[TimeSource::CELLULAR] == true) newTimeFix = 3; //If only ONE of the remote sources is present, level 3 fix
        else if(sourceAvailable[TimeSource::GPS_RTC] == true || sourceAvailable[TimeSource::RTC] == true) newTimeFix = 2; //If only local source present, level 2 fix
        else if(sourceAvailable[TimeSource::INCREMENT] == true) newTimeFix = 1; //If only delta time agrees, level 1 fix
        else {
            newTimeFix = 0; //If not even delta time agrees, there is no fix
            if(initDone) criticalFault = true; //Allow local only without resetting device only during init period 
            throwError(CLOCK_NO_SYNC | 0x100); //Report lack of sync as error 
        }
    }


    //////////////////////////// SET TIME ////////////////////////////
    uint8_t source = 0; 
    if(timeGood == false || force == true || newTimeFix > timeFix) { //If there is an error in time, go to set time, or if the new time availability is better than the last sync time 
        timeGood = false; //Clear flag once the timeset begins 
        // int8_t sourceA = TimeSource::NONE; //Keep track of which sources are used
        // int8_t sourceB = TimeSource::NONE;
        if(sourceAvailable[TimeSource::GPS] ^ sourceAvailable[TimeSource::CELLULAR]) { //If only ONE of the external sources is avilible 
            uint8_t remoteSource = sourceAvailable[TimeSource::GPS] ? TimeSource::GPS : TimeSource::CELLULAR; //Check which remote source is availible 
            timeSourceA = remoteSource;
            for(int t = 0; t < (numClockSources - 1); t++) {
                if(abs(times[remoteSource] - times[t]) < maxTimeError && t != remoteSource) { //If available time matches with another time (2 times agree) that is not iself, proceed with the time set
                    timeSourceB = t; //Record secondary source
                    m_serialDebug.println("SET PARTICLE RTC"); //DEBUG!
                    m_timeProvider.setTime(times[remoteSource]);  //Set 
                    rtc.setTime(m_timeProvider.year(times[remoteSource]), m_timeProvider.month(times[remoteSource]), m_timeProvider.day(times[remoteSource]), m_timeProvider.hour(times[remoteSource]), m_timeProvider.minute(times[remoteSource]), m_timeProvider.second(times[remoteSource]));
                    timeGood = true; //Assert flag after time set
                    break; //Exit after the highest tier is used
                }
            }
            if(!timeGood) { //If no matching time found, set to none and set time anyway
                timeSourceB = TimeSource::NONE; //Set even if find no agreeing time
                m_serialDebug.println("SET PARTICLE RTC"); //DEBUG!
                m_timeProvider.setTime(times[remoteSource]);  //Set
                rtc.setTime(m_timeProvider.year(times[remoteSource]), m_timeProvider.month(times[remoteSource]), m_timeProvider.day(times[remoteSource]), m_timeProvider.hour(times[remoteSource]), m_timeProvider.minute(times[remoteSource]), m_timeProvider.second(times[remoteSource]));
            }

        }
        else {
            for(int i = 0; i < (numClockSources - 1); i++) { //Set options do not include current time itself, start with most legit result and go from there
                if(sourceAvailable[i] == true && timeGood == false) { //If source is available and sync not finished
                    timeSourceA = i; //Record highest priority source
                    for(int t = 0; t < (numClockSources - 1); t++) {
                        if(abs(times[i] - times[t]) < maxTimeError && t != i) { //If available time matches with another time (2 times agree) that is not iself, proceed with the time set
                            timeSourceB = t; //Record secondary source
                            m_serialDebug.println("SET PARTICLE RTC"); //DEBUG!
                            m_timeProvider.setTime(times[i]);  //Set 
                            if(timeSourceA <= TimeSource::CELLULAR) { //If a tier 1 or 2 value is used, also update the kestrel RTC
                                rtc.setTime(m_timeProvider.year(times[i]), m_timeProvider.month(times[i]), m_timeProvider.day(times[i]), m_timeProvider.hour(times[i]), m_timeProvider.minute(times[i]), m_timeProvider.second(times[i]));
                            }
                            timeGood = true; //Assert flag after time set
                            break; //Exit after the highest tier is used
                        }
                    }
                }
            }
        } 
        //Evaluate new time set
        if(timeSourceA == TimeSource::GPS && timeSourceB == TimeSource::CELLULAR) timeFix = 4; //If both remote time sources are present, best fix
        else if(timeSourceA == TimeSource::GPS || timeSourceA == TimeSource::CELLULAR) timeFix = 3; //If only ONE of the remote sources is present, level 3 fix
        else if(timeSourceA == TimeSource::GPS_RTC || timeSourceA == TimeSource::RTC) timeFix = 2; //If only local source present, level 2 fix
        else if(timeSourceA == TimeSource::INCREMENT) timeFix = 1; //If only delta time agrees, level 1 fix
        else {
            timeFix = 0; //If not even delta time agrees, there is no fix
            if(initDone) criticalFault = true; //Allow local only without resetting device only during init period 
            throwError(CLOCK_NO_SYNC); //Report lack of sync as error 
        }
        if(timeFix > 0 && timeGood == true) { //Update for increment
            lastTimeSync = m_timeProvider.now(); //Update time of last sync
            previousTime = m_timeProvider.now(); //Grab updated time before exiting
            previousMillis = m_timeProvider.millis(); //Grab millis before exiting
        }
        else lastTimeSync = 0; //Otherwise indiciate sync failed
    }
    source = timeSourceA; //Use highest value as source

    

	
    
    
    // uint8_t source = TimeSource::NONE; //Default to none unless otherwise set
    // if(abs(rtcTime - gpsTime) < maxTimeError && abs(rtcTime - cellTime) < maxTimeError && rtcTime != 0 && gpsTime != 0 && cellTime != 0) { //If both updated sources match local time
    //     m_serialDebug.println("CLOCK SOURCE: All match");
    //     timeGood = true;
    //     source = TimeSource::CELLULAR; //Report cell as the most comprehensive source
    // }

    // if(abs(cellTime - gpsTime) < maxTimeError && gpsTime != 0 && cellTime != 0) { //If both remote variables match, update the time no mater what the state of the rest are
    //     m_serialDebug.println("CLOCK SOURCE: GPS and Cell match");
    //     // time_t currentTime = m_timeProvider.now(); //Ensure sync and that local offset is not applied 
    //     // rtc.setTime(m_timeProvider.year(currentTime), m_timeProvider.month(currentTime), m_timeProvider.day(currentTime), m_timeProvider.hour(currentTime), m_timeProvider.minute(currentTime), m_timeProvider.second(currentTime)); //Set RTC from Cell
    //     if(cellTime != 0) { //DEBUG! RESTORE!
    //         time_t currentTime = cellTime; //Grab time from cell, even though it is old, to ensure correct time is being set //FIX!
    //         m_serialDebug.print("RTC Set Time: "); //DEBUG!
    //         m_serialDebug.print(m_timeProvider.hour(currentTime));
    //         m_serialDebug.print(":");
    //         m_serialDebug.print(m_timeProvider.minute(currentTime));
    //         m_serialDebug.print(":");
    //         m_serialDebug.println(m_timeProvider.second(currentTime));
    //         rtc.setTime(m_timeProvider.year(currentTime), m_timeProvider.month(currentTime), m_timeProvider.day(currentTime), m_timeProvider.hour(currentTime), m_timeProvider.minute(currentTime), m_timeProvider.second(currentTime)); //Set RTC from Cell
    //     }
        
    //     timeGood = true;
    //     source = TimeSource::CELLULAR;
    //     if(!(abs(rtcTime - gpsTime) < maxTimeError && abs(rtcTime - cellTime) < maxTimeError && rtcTime != 0 && gpsTime != 0 && cellTime != 0) && timeSyncVals[2] != 0) throwError(CLOCK_MISMATCH | 0x05); //Check if RTC is within range of others, if not throw error (only if not caused by unavailability)
    // }
    // else if(abs(cellTime - rtcTime) < maxTimeError && rtcTime != 0 && cellTime != 0) { //If cell and rtc agree
    //     m_serialDebug.println("CLOCK SOURCE: Cell and local match");
    //     //Can we set the GPS time??
    //     //Throw error
    //     timeGood = true;
    //     source = TimeSource::CELLULAR;
    //     if(timeSyncVals[1] != 0) throwError(CLOCK_MISMATCH | 0x08); //Throw clock mismatch error, OR with GPS indicator (only if not caused by clock unavailabilty) 
    // }
    // else if(abs(gpsTime - rtcTime) < maxTimeError && gpsTime != 0 && rtcTime != 0) { //If gps and rtc agree
    //     m_serialDebug.println("CLOCK SOURCE: GPS and local match");
    //     m_timeProvider.setTime(gpsTime); //Set particle time from GPS time
    //     //Throw error
    //     timeGood = true;
    //     source = TimeSource::GPS;
    //     if(timeSyncVals[0] != 0) throwError(CLOCK_MISMATCH | 0x06); //Throw clock mismatch error, OR with Cell indicator (only if not caused by clock unavailabilty) 
    // }
    // else { //No two sources agree, very bad!
        
    //     if(rtcTime > 1641016800) { //Jan 1, 2022, date seems to be reeasonable //FIX!
    //         m_serialDebug.println("CLOCK SOURCE: Stale RTC"); //DEBUG!
    //         m_timeProvider.setTime(rtc.getTimeUnix()); //Set time from RTC   
    //         timeGood = true;
    //         source = TimeSource::RTC;
    //         //Throw mismatch errors as needed //FIX!??
    //         // if(timeSyncVals[0] != 0) throwError(CLOCK_MISMATCH | 0x300); //Throw clock mismatch error, OR with Cell indicator (only if not caused by clock unavailabilty) 
    //         // if(timeSyncVals[1] != 0) throwError(CLOCK_MISMATCH | 0x200); //Throw clock mismatch error, OR with GPS indicator (only if not caused by clock unavailabilty) 
    //     }
    //     else {
    //         m_serialDebug.println("CLOCK SOURCE: NONE"); //DEBUG!
    //         criticalFault = true; //FIX??
    //         timeGood = false; 
    //         m_timeProvider.setTime(946684800); //Set time back to year 2000
    //         source = TimeSource::NONE;
    //     }
    //     throwError(CLOCK_NO_SYNC); //Throw error regardless of which state, because no two sources are able to agree we have a sync failure 

        
    // }
    // if(source != TimeSource::NONE && source != TimeSource::RTC) lastTimeSync = getTime(); //If time has been sourced, update the last sync time
    // else lastTimeSync = 0; //Otherwise, set back to unknown time
    // timeSource = source; //Grab the time source used 
    // // return false; //DEBUG!
    enableAuxPower(currentAux); //Return all to previous states
    enableI2C_Global(currentGlob);
    enableI2C_OB(currentOB);
    m_serialDebug.print("Timebase End: "); //DEBUG!
    m_serialDebug.println(m_timeProvider.millis());
    // if(timeGood == true) {
    //     previousTime = m_timeProvider.now(); //Grab updated time before exiting
    //     previousMillis = m_timeProvider.millis(); //Grab millis before exiting
    // }
    return source;
}

time_t Kestrel::getTime()
{
    if(!m_timeProvider.isValid() || !timeGood) { //If time has not been synced, do so now
        syncTime();
    }
    if(m_timeProvider.isValid() && timeGood) { //If time is good, report current value
        return m_timeProvider.now();
    }
    // return m_timeProvider.now();
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
    if(updateGPS) updateLocation(); //Check if pending update, if so call it
    if(latitude != 0) return String(latitude*(10E-8)); //Return in degrees if value is legit
    else return "null"; //Return null if position has not been initalized
}

String Kestrel::getPosLong()
{
    if(updateGPS) updateLocation(); //Check if pending update, if so call it
    if(longitude != 0) return String(longitude*(10E-8)); //Return in degrees if value if legit
    else return "null"; //Return null if position has not been initalized
}

String Kestrel::getPosAlt()
{
    if(updateGPS) updateLocation(); //Check if pending update, if so call it
    if(altitude != 0) return String(altitude*(10E-4)); //Return in m 
    else return "null"; //Return null if position has not been initalized
}

time_t Kestrel::getPosTime()
{
    if(updateGPS) updateLocation(); //Check if pending update, if so call it
    return posTime;
}

String Kestrel::getPosTimeString()
{
    if(updateGPS) updateLocation(); //Check if pending update, if so call it
    if(posTime > 0) return String((int)posTime); //Return in time of last position measurment if legitimate
    else return "null"; //Return null if position has not been initalized
}

bool Kestrel::startTimer(time_t period)
{
    if(period == 0) period = defaultPeriod; //If no period is specified, assign default period 
    bool currentOB = enableI2C_OB(true);
    bool currentGlob = enableI2C_Global(false);
    rtc.setAlarm(period); //Set alarm from current time
    timerStart = m_timeProvider.millis(); 
    m_serialDebug.print("Time Start: "); //DEBUG!
    m_serialDebug.println(timerStart);
    logPeriod = period;
    enableI2C_Global(currentGlob);
    enableI2C_OB(currentOB);
    return false; //DEBUG!
}

bool Kestrel::waitUntilTimerDone()
{
    if(logPeriod == 0) return false; //Return if not already setup
    m_serialDebug.print("Time Now: "); //DEBUG!
    m_serialDebug.println(m_timeProvider.millis());
    
    while(m_gpio.digitalRead(Pins::Clock_INT) == HIGH && ((m_timeProvider.millis() - timerStart) < (logPeriod*1000 + 500))){ //Wait until either timer has expired or clock interrupt has gone off //DEBUG! Give 500 ms cushion for testing RTC
        m_timeProvider.delay(1); 
        m_cloud.process(); //Run process continually while waiting in order to make sure device is responsive 
    } 
    if(m_gpio.digitalRead(Pins::Clock_INT) == LOW) return true; //If RTC triggers properly, return true, else return false 
    else {
        throwError(ALARM_FAIL); //Throw alarm error since RTC did not wake device 
        return false; 
    }
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
    if(currentTime != 0) return getTime() % (m_timeProvider.millis() / 1000); //If current time is valid, create the hash as usual
    else return HAL_RNG_GetRandomNumber(); //Else return cryptographic 32 bit random 
}

bool Kestrel::testForBat()
{
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);
    m_ioOB.pinMode(PinsOB::CE, OUTPUT);
    m_ioOB.pinMode(PinsOB::CSA_EN, OUTPUT);
    m_ioOB.digitalWrite(PinsOB::CE, HIGH); //Disable charging
    m_ioOB.digitalWrite(PinsOB::CSA_EN, HIGH); //Enable voltage sense
    m_csaAlpha.enableChannel(CH1, true);
    m_csaAlpha.update(); //Force new readings 
    m_timeProvider.delay(5000); //Wait for cap to discharge 
    // m_csaAlpha.SetCurrentDirection(CH1, BIDIRECTIONAL);
    float vBat = m_csaAlpha.getBusVoltage(CH1);
    m_ioOB.digitalWrite(PinsOB::CE, LOW); //Turn charging back on
    bool result = false;
    if(vBat < 2.0) { //If less than 2V (min bat voltage) give error 
        //THROW ERROR???
        result = false;
    }
    else result = true;
    enableI2C_Global(currentGlob); //Reset to previous state
    enableI2C_OB(currentOB); 
    m_serialDebug.print("BATTERY STATE: "); //DEBUG!
    m_serialDebug.print(vBat);
    m_serialDebug.print("\t");
    m_serialDebug.println(result);
    return result;
    // enableI2C_External(true);
}

bool Kestrel::releaseWDT()
{ //Intentionally let the dog off the leash to reset the device
    bool state = wdtRelease;
    wdtRelease = true;
    return state;
}

bool Kestrel::feedWDT()
{
    if(!criticalFault && !wdtRelease) { //If there is currently no critical fault and no release called for, feed WDT
        m_gpio.pinMode(Pins::WD_HOLD, IPinMode::OUTPUT);
        m_gpio.digitalWrite(Pins::WD_HOLD, LOW);
        m_timeProvider.delay(1);
        m_gpio.digitalWrite(Pins::WD_HOLD, HIGH);
        m_timeProvider.delay(1);
        m_gpio.digitalWrite(Pins::WD_HOLD, LOW);
        return true;
    } 
    else if(wdtRelease) {
        throwError(WDT_OFF_LEASH | 0x100); //Report incoming error 
        return false;
    }
    else {
        // m_system.reset(); //DEBUG!
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
            float zVal = accel.getAccel(2);
            if(zVal > 0) accel.offset[2] = 1 - accel.getAccel(2); //Set z to 1 
            else if(zVal < 0) accel.offset[2] = accel.getAccel(2) + 1; //Set for negative offset
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
    m_serialDebug.println("CONFIG TALON SENSE"); //DEBUG!
    bool currentGlob = enableI2C_Global(false);
	bool currentOB = enableI2C_OB(true);
    m_csaBeta.setCurrentDirection(CH4, UNIDIRECTIONAL); //Bulk voltage, unidirectional
	m_csaBeta.enableChannel(CH1, false); //Disable all channels but 4
	m_csaBeta.enableChannel(CH2, false);
	m_csaBeta.enableChannel(CH3, false);
	m_csaBeta.enableChannel(CH4, true);
    enableI2C_Global(currentGlob); //Reset to previous state
    enableI2C_OB(currentOB); 
    // enableI2C_Global(true); //Connect all together 
    return false; //DEBUG!
}

int Kestrel::sleep()
{
    if((m_timeProvider.millis() - timerStart) > sysCollectMax) throwError(EXCEED_COLLECT_TIME | portErrorCode); //Throw error for whole logging system taking too long
    ISleepConfig config;
    ICloudDisconnectOptions options;
    // SystemSleepResult result;
    switch(powerSaveMode) {
        case PowerSaveModes::PERFORMANCE:
            return 0; //Nothing to do for performance mode 
            break; 
        case PowerSaveModes::BALANCED:
            config.mode = ISleepMode::ULTRA_LOW_POWER; //Configure sleep mode
            config.network = INetworkInterfaceIndex::CELLULAR; //Keep network alive
            // .flag(SystemSleepFlag::WAIT_CLOUD) //Wait for cloud communications to finish before going to sleep
            config.duration = 20min; //DEBUG!
            config.wakePin = Pins::Clock_INT;\
            config.wakePinMode = IInterruptMode::FALLING; //Trigger on falling clock pulse
            // enableSD(false); //Turn off SD power
            m_ioOB.digitalWrite(PinsOB::LED_EN, HIGH); //Disable LEDs (if not done already) 
            led.sleep(true); //Put LED driver into low power mode 
            if(!gps.powerOffWithInterrupt(3600000, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0)) throwError(GPS_READ_FAIL); //Shutdown for an hour unless woken up via pin trip

            // result = m_system.sleep(config);
            // m_system.sleep(config); //DEBUG!
            break;
        case PowerSaveModes::LOW_POWER:
            //options.graceful = true;
            //options.timeout = 30s;
            // m_cloud.disconnect(options); //Disconnect from cloud and make sure messages are sent first
            config.mode = ISleepMode::ULTRA_LOW_POWER; //Configure sleep mode
            config.network = INetworkInterfaceIndex::CELLULAR; //Keep network alive
                // .flag(SystemSleepFlag::WAIT_CLOUD) //Wait for cloud communications to finish before going to sleep
                // .duration(5min); //DEBUG!
            config.duration = 20min; //DEBUG!
            config.wakePin = Pins::Clock_INT;
            config.wakePinMode = IInterruptMode::FALLING; //Trigger on falling clock pulse
            // enableSD(false); //Turn off SD power
            enableAuxPower(false); //Turn all aux power off
            m_ioOB.digitalWrite(PinsOB::LED_EN, HIGH); //Disable LEDs (if not done already)
            led.sleep(true); //Put LED driver into low power mode  
            // gps.powerOffWithInterrupt(3600000, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0); //Shutdown for an hour unless woken up via pin trip

            // result = m_system.sleep(config);
            // m_system.sleep(config); //DEBUG!
            break;
        case PowerSaveModes::ULTRA_LOW_POWER:
            options.graceful = true;
            options.timeout = 30s;
            m_cloud.disconnect(options); //Disconnect from cloud and make sure messages are sent first
            // Cellular.off();
            // m_system.waitForCondition(Cellular.isOff, 30000); //Wait up to 30 seconds for cell to turn off before sleep
            config.mode = ISleepMode::ULTRA_LOW_POWER; //Configure sleep mode
                // .network(NETWORK_INTERFACE_CELLULAR) //Keep network alive
                // .flag(SystemSleepFlag::WAIT_CLOUD) //Wait for cloud communications to finish before going to sleep
                // .duration(5min); //DEBUG!
            config.wakePin = Pins::Clock_INT;
            config.wakePinMode = IInterruptMode::FALLING; //Trigger on falling clock pulse
            enableAuxPower(false); //Turn all aux power off
            m_ioOB.digitalWrite(PinsOB::LED_EN, HIGH); //Disable LEDs (if not done already) 
            led.sleep(true); //Put LED driver into low power mode 
            // m_ioOB.digitalWrite(PinsOB::CSA_EN, LOW); //Disable CSAs //DEBUG! //FIX!

            // //SLEEP FRAM //FIX!
            // m_wire.beginTransmission(0x7C);
            // m_wire.write((0x50 << 1) | 0x01); //Shift to add "r/w" bit
            // m_wire.endTransmission(false);
            // m_wire.beginTransmission(0x43);
            // m_wire.endTransmission();

            // //SLEEP ACCEL //FIX!
            // m_wire.beginTransmission(0x15);
            // m_wire.write(0x0D); //Write to control register
            // m_wire.write(0x01); //Set power down
            // m_wire.endTransmission();
            break;
        default:
            //THROW ERROR??
            return 0; //Mimic perfromance mode if not specificed  
            break; 
    }
    // return (int) result.wakeupReason(); //Return reason for wake if not bypassed
    int fpscr = __get_FPSCR();
    m_serialDebug.print("FPSCR Status: 0x");
    m_serialDebug.println(fpscr, HEX);
    m_serialDebug.print("Sleep Time: ");
    m_serialDebug.println(m_timeProvider.millis()); 
    // m_serialDebug.print("RTC Regs: \tCRTL:0x");
    // m_serialDebug.print(rtc.readByte(0x07), HEX); //DEBUG!
    // m_serialDebug.print("\tALM0:0x");
    // m_serialDebug.println(rtc.readByte(0x0D), HEX);  //DEBUG!
    // m_serialDebug.print("\tAML1:0x");
    // m_serialDebug.println(rtc.readByte(0x14), HEX);  //DEBUG!

    m_serialDebug.flush();
    m_timeProvider.delay(5000); //DEBUG!
    __set_FPSCR(fpscr & ~0x7); //Clear error bits to prevent assertion failure 
    if(m_gpio.digitalRead(Pins::Clock_INT) != LOW) {
        IWakeupReason result = m_system.sleep(config); //If clock not triggered already, go to sleep
        if(powerSaveMode == PowerSaveModes::LOW_POWER) { //Deal with extended sleep periods in low power mode
            m_timeProvider.delay(5000); //DEBUG!
            m_serialDebug.print("WAKE! - "); //DEBUG!
            m_serialDebug.print(static_cast<int>(result)); //DEBUG!
            m_serialDebug.print("\t");
            m_serialDebug.println(m_timeProvider.millis()); //DEBUG!
            m_serialDebug.flush(); //DEBUG!

            int wakeupCount = 0; //Count how many times in a row the system is woken up by the timer
            while((result == IWakeupReason::BY_RTC || result == IWakeupReason::BY_NETWORK) && wakeupCount < 16) { //FIX! Make variable 
                if(result == IWakeupReason::BY_RTC) {
                    m_cloud.connect();
                    m_system.waitForCondition([this]() { return m_cloud.connected(); }, std::chrono::milliseconds(5000)); //Wait for a max of 5 seconds for particle to connect
                    // m_cloud.publish("DUMMY"); //DEBUG!
                    if(!m_cloud.connected()) throwError(CELL_FAIL | 0x100); //Or with reconnect flag
                    wakeupCount++;
                }
                m_serialDebug.println("Wakeup Attemp Done - return to sleep"); //DEBUG!
                m_serialDebug.flush(); //DEBUG!
                m_system.waitForCondition([this]() { return m_cloud.connected(); }, std::chrono::milliseconds(5000));
                result = m_system.sleep(config); //Go back to sleep after re-connecting
            }
        }
        if(powerSaveMode == PowerSaveModes::BALANCED) {
            if(result == IWakeupReason::BY_RTC) throwError(ALARM_FAIL | 0x100); //Throw error due to wake from timer not clock int 
        }
    }
    else {
        m_serialDebug.println("ERR - Clock Already Triggered"); //DEBUG!
        return 0; //Otherwise exit with fail state //THROW ERROR!
    }
    return 1; //DEBUG! 
}

int Kestrel::wake()
{
    switch(powerSaveMode) {
        case PowerSaveModes::PERFORMANCE:
            return 0; //Nothing to do for performance mode 
            break; 
        case PowerSaveModes::BALANCED:
            if((getTime() - posTime) > 3600) { //If it has been more than an hour since last GPS point reading
                m_serialDebug.println("Wake GPS"); //DEBUG!
                m_ioOB.pinMode(PinsOB::GPS_INT, OUTPUT); //Turn GPS back on by toggling int pin
                m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
                m_timeProvider.delay(1000);
                m_ioOB.digitalWrite(PinsOB::GPS_INT, HIGH);
                m_timeProvider.delay(1000);
                m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
                m_timeProvider.delay(1000);
                if(gps.begin() == false) {
                    criticalFault = true; //DEBUG! ??
                    throwError(GPS_INIT_FAIL);
                    m_serialDebug.println("GPS ERROR");
                }
                else {
                    gps.setI2COutput(COM_TYPE_UBX);
                    // gps.setAutoPVT(true); //DEBUG!
                    unsigned long localTime = m_timeProvider.millis();
                    while((gps.getFixType() < 2 || gps.getFixType() > 4) && !gps.getGnssFixOk() && (localTime - m_timeProvider.millis()) < 30000); //Wait up to 30 seconds to get a GPS fix, if not, move on
                    if(!(gps.getFixType() >= 2 && gps.getFixType() <= 4)) { //If GPS failed to connect after that period, throw error
                        throwError(GPS_UNAVAILABLE | 0x100); //Set subtype to timeout
                    }
                    updateGPS = true; //Set flag so position is updated at next update call
                }
            }
            enableSD(true); //Turn SD back on
            updateTime(); //Grab updated time each wakeup
            break;
        case PowerSaveModes::LOW_POWER:
            enableAuxPower(true); //Turn aux power back on
            if((getTime() - posTime) > 14400) { //If it has been more than 4 hours since last GPS point reading
                m_serialDebug.println("Power Up GPS"); //DEBUG!
                m_ioOB.pinMode(PinsOB::GPS_INT, OUTPUT); //Turn GPS back on by toggling int pin
                m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
                m_timeProvider.delay(1000);
                m_ioOB.digitalWrite(PinsOB::GPS_INT, HIGH);
                m_timeProvider.delay(1000);
                m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
                m_timeProvider.delay(1000);
                if(gps.begin() == false) {
                    criticalFault = true; //DEBUG! ??
                    throwError(GPS_INIT_FAIL);
                    m_serialDebug.println("GPS ERROR");
                }
                else {
                    gps.setI2COutput(COM_TYPE_UBX);
                    // gps.setAutoPVT(true); //DEBUG!
                    unsigned long localTime = m_timeProvider.millis();
                    while((gps.getFixType() < 2 || gps.getFixType() > 4) && !gps.getGnssFixOk() && (localTime - m_timeProvider.millis()) < 60000); //Wait up to 60 seconds to get a GPS fix, if not, move on
                    if(!(gps.getFixType() >= 2 && gps.getFixType() <= 4)) { //If GPS failed to connect after that period, throw error
                        throwError(GPS_UNAVAILABLE | 0x100); //Set subtype to timeout
                    }
                    updateGPS = true; //Set flag so position is updated at next update call
                }
            }
            enableSD(true); //Turn SD back on
            updateTime(); //Grab updated time each wakeup
            break;
        case PowerSaveModes::ULTRA_LOW_POWER:
            enableAuxPower(true); //Turn aux power back on
            // m_cloud.connect();
            // m_system.waitForCondition(m_cloud.connected, 180s); //Wait for a max of 180 seconds for particle to connect
            // if(!m_cloud.connected()) throwError(CELL_FAIL | 0x100); //Throw error if not connected after given period - Or with reconnect flag
            //TURN ON GPS! FIX!
            // m_serialDebug.println("Power Up GPS"); //DEBUG!
            // m_ioOB.pinMode(PinsOB::GPS_INT, OUTPUT); //Turn GPS back on by toggling int pin
            // m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
            // m_timeProvider.delay(1000);
            // m_ioOB.digitalWrite(PinsOB::GPS_INT, HIGH);
            // m_timeProvider.delay(1000);
            // m_ioOB.digitalWrite(PinsOB::GPS_INT, LOW);
            // m_timeProvider.delay(1000);
            // if(gps.begin() == false) {
            //     criticalFault = true; //DEBUG! ??
            //     throwError(GPS_INIT_FAIL);
            //     m_serialDebug.println("GPS ERROR");
            // }
            // else {
            //     gps.setI2COutput(COM_TYPE_UBX);
            //     // gps.setAutoPVT(true); //DEBUG!
            //     unsigned long localTime = m_timeProvider.millis();
            //     while((gps.getFixType() < 2 || gps.getFixType() > 4) && !gps.getGnssFixOk() && (localTime - m_timeProvider.millis()) < 60000); //Wait up to 60 seconds to get a GPS fix, if not, move on
            //     if(!(gps.getFixType() >= 2 && gps.getFixType() <= 4)) { //If GPS failed to connect after that period, throw error
            //         throwError(GPS_UNAVAILABLE | 0x100); //Set subtype to timeout
            //     }
            //     updateGPS = true; //Set flag so position is updated at next update call
            // }
            //TURN ON ACCEL! FIX!
            //TURN ON FRAM! FIX!
            enableSD(true); //Turn SD back on
            updateTime(); //Grab updated time each wakeup
            break;

        default:
            //THROW ERROR??
            return 0; //Mimic perfromance mode if not specificed  
            break; 
    }
    return 1; //DEBUG!
}

void Kestrel::timechange_handler(IEventType event, int param)
{
    // m_serialDebug.print("Time Change Handler: "); //DEBUG!
    // m_serialDebug.print(event); //DEBUG!
    // m_serialDebug.print("\t");
    // m_serialDebug.println(param); //DEBUG!
    if(event == IEventType::TIME_CHANGED) { //Confirm event type before proceeding 
        if(param == time_changed_sync && !(selfPointer->initDone)) { 
            selfPointer->m_serialDebug.println("TIME CHANGE: Updating"); //DEBUG!
            // selfPointer->syncTime(); //if time update not from manual sync (and sync not requested), call time sync to return to desired val
        }
        if(param == time_changed_sync && !(selfPointer->timeSyncRequested) && (selfPointer->initDone)) { 
            selfPointer->m_serialDebug.println("TIME CHANGE: Auto"); //DEBUG!
            selfPointer->syncTime(); //if time update not from manual sync (and sync not requested), call time sync to return to desired val
        }
        if(param == time_changed_sync && selfPointer->timeSyncRequested) {
            selfPointer->m_serialDebug.println("TIME CHANGE: Requested"); //DEBUG!
            selfPointer->timeSyncRequested = false; //Clear flag
        }
        if(param == time_changed_manually) selfPointer->m_serialDebug.println("TIME CHANGE: Manual"); //DEBUG!
    }
}

void Kestrel::outOfMemoryHandler(IEventType event, int param) {
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
