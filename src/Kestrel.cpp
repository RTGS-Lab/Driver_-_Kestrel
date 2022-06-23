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

Kestrel::Kestrel() : ioOB(0x20), ioTalon(0x21)
{
	// port = talonPort; //Copy to local
	// version = hardwareVersion; //Copy to local
}

String Kestrel::begin()
{
    #if defined(ARDUINO) && ARDUINO >= 100 
		Wire.begin();
	#elif defined(PARTICLE)
		if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
	#endif

    enableI2C_Global(false); //Turn off external I2C
    enableI2C_OB(true); //Turn on internal I2C

    ioOB.begin();
    ioTalon.begin();

    for(int i = 1; i <= 4; i++) {
        enablePower(i, false); //Default all power to off
        enableData(i, false); //Default all data to off
    }
    // ioOB.pinMode(PinsOB::)

    return ""; //DEBUG!
}

bool Kestrel::enablePower(uint8_t port, bool state) 
{
    //FIX! Throw error is port out of range
    if(port == 0 || port > numTalonPorts) throwError(PORT_RANGE_ERROR | portErrorCode);
    else {
        enableI2C_OB(true);
        enableI2C_Global(false);
        ioTalon.pinMode(PinsTalon::EN[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::EN[port - 1], state);
    }
    
    return false; //DEBUG!
}

bool Kestrel::enableData(uint8_t port, bool state)
{
    //FIX! Throw error is port out of range
    if(port == 0 || port > numTalonPorts) throwError(PORT_RANGE_ERROR | portErrorCode);
    else {
        enableI2C_OB(true);
        enableI2C_Global(false);
        ioTalon.pinMode(PinsTalon::I2C_EN[port - 1], OUTPUT);
        ioTalon.digitalWrite(PinsTalon::I2C_EN[port - 1], state);
        Serial.println(PinsTalon::I2C_EN[port - 1]); //DEBUG!
    }
    
    return false; //DEBUG!
}

bool Kestrel::enableI2C_OB(bool state)
{
    pinMode(Pins::I2C_OB_EN, OUTPUT);
	digitalWrite(Pins::I2C_OB_EN, state);
    return false; //DEBUG!
}

bool Kestrel::enableI2C_Global(bool state)
{
    pinMode(Pins::I2C_GLOBAL_EN, OUTPUT);
	digitalWrite(Pins::I2C_GLOBAL_EN, state);
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