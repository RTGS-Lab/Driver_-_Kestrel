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
/**
 * @file Kestrel.h
 *
 * @mainpage Kestrel
 *
 * @section description Description
 * An interface to control the internal elements of the Kestrel data logger
 *
 * @section circuit Circuit
 * - Red LED connected to pin D2.
 * - Momentary push button connected to pin D3.
 *
 * @section libraries Libraries
 * - Arduino_LSM6DS3 (https://github.com/arduino-libraries/Arduino_LSM6DS3)
 *   - Interacts with on-board IMU.
 *
 * @section notes Notes
 * - Comments are Doxygen compatible.
 *
 * @section todo TODO
 * - Don't use Doxygen style formatting inside the body of a function.
 *
 * @section author Author
 * - Created by Bobby Schulz on 6/21/2022
 *
 * Copyright (c) 2020 Woolsey Workshop.  All rights reserved.
 */

#ifndef Kestrel_h
#define Kestrel_h

#include <Sensor.h>
#include <PCAL9535A.h>
#include "PCA9634/src/PCA9634.h"
// #include "MCP7940_Library/src/MCP7940.h"
#include "DRIVER_-_MCP79412/src/MCP79412.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library/src/SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "PAC1932_Library/src/PAC1934.h"
#include "VEML3328/src/VEML3328.h"
#include <Adafruit_SHT4x.h>
#include <MXC6655.h>
// #include <GlobalPins.h>



namespace Pins { //Use for B402
	constexpr uint16_t WD_HOLD  = D2;
	constexpr uint16_t SD_CS    = D8;
	constexpr uint16_t Clock_INT 	= D22;
	constexpr uint16_t TALON1_GPIOA = A3;
	constexpr uint16_t TALON1_GPIOB = D7;
	constexpr uint16_t TALON2_GPIOA = A2;
	constexpr uint16_t TALON2_GPIOB = D6;
	constexpr uint16_t TALON3_GPIOA = A1;
	constexpr uint16_t TALON3_GPIOB = D5;
	constexpr uint16_t I2C_GLOBAL_EN = D23; //FIX!
	constexpr uint16_t I2C_OB_EN = A6; //FIX!
}

namespace PinsOB {
	constexpr uint16_t I2C_EXT_EN = 10;
	constexpr uint16_t SD_CD = 8;
	constexpr uint16_t SD_EN = 12;
	constexpr uint16_t AUX_EN = 15;
	constexpr uint16_t CE = 11;
	constexpr uint16_t LED_EN = 13;
	constexpr uint16_t CSA_EN = 14;
}

namespace PinsTalon { //For Kestrel v1.1
    constexpr uint8_t SEL[4] = {0, 4, 8, 12};
    constexpr uint8_t I2C_EN[4] = {1, 5, 9, 13};
    constexpr uint8_t EN[4] = {3, 7, 11, 15};
    constexpr uint8_t FAULT[4] = {2, 6, 10, 14};
	// constexpr uint16_t SEL1 = 0;
	// constexpr uint16_t SEL2 = 4;
	// constexpr uint16_t SEL3 = 8;
	// constexpr uint16_t SEL4 = 12;
	// constexpr uint16_t I2C_EN1 = 1;
	// constexpr uint16_t I2C_EN2 = 5;
	// constexpr uint16_t I2C_EN3 = 9;
	// constexpr uint16_t I2C_EN4 = 13;
	// constexpr uint16_t EN1 = 3;
	// constexpr uint16_t EN2 = 7;
	// constexpr uint16_t EN3 = 11;
	// constexpr uint16_t EN4 = 15;
	// constexpr uint16_t FAULT1 = 2;
	// constexpr uint16_t FAULT2 = 6;
	// constexpr uint16_t FAULT3 = 10;
	// constexpr uint16_t FAULT4 = 14;
}

namespace TimeSource { //FIX!
	constexpr uint8_t RTC = 1;
	constexpr uint8_t CELLULAR = 3;
	constexpr uint8_t GPS = 2; 
	constexpr uint8_t NONE = 0;
}

namespace IndicatorLight {
	constexpr uint8_t SENSORS = 1;
	constexpr uint8_t GPS = 2;
	constexpr uint8_t CELL = 3;
	constexpr uint8_t STAT = 4;
	constexpr uint8_t ALL = 5;
}

namespace IndicatorMode {
	constexpr uint8_t NONE = 0;
	constexpr uint8_t PASS = 1;
	constexpr uint8_t WAITING = 2;
	constexpr uint8_t ERROR = 3;
	constexpr uint8_t ERROR_CRITICAL = 4;
	constexpr uint8_t PREPASS = 5;
	constexpr uint8_t INIT = 6;
	constexpr uint8_t IDLE = 7;
	constexpr uint8_t COMMAND = 8;
	
}

struct dateTimeStruct {
			int year;
			int month;
			int day;
			int hour;
			int minute;
			int second;
			uint8_t source = TimeSource::NONE;
		};

class Kestrel: public Sensor
{
    constexpr static int MAX_NUM_ERRORS = 10; ///<Maximum number of errors to log before overwriting previous errors in buffer
	const String FIRMWARE_VERSION = "1.0.0"; //FIX! Read from system??
	
    const uint32_t KESTREL_PORT_RANGE_ERROR = 0x90010300; ///<Kestrel port assignment is out of range
	const uint32_t CSA_INIT_FAIL = 0x100500F0; ///<Failure to initialize CSA Alpha or CSA Beta
	const uint32_t GPS_INIT_FAIL = 0x100A00F8; ///<Failure to initialize the onboard GPS
	const uint32_t GPS_READ_FAIL = 0x100B00F8; ///<Failure to read from the onboard GPS

    public:
        Kestrel();
		SFE_UBLOX_GNSS gps;
        String begin(time_t time, bool &criticalFault, bool &fault);
        bool enablePower(uint8_t port, bool state = true);
        bool enableData(uint8_t port, bool state = true);
		bool setDirection(uint8_t port, bool sel);
        bool disablePowerAll();
        bool disableDataAll();
		bool getFault(uint8_t pin);
        bool enableI2C_OB(bool state = true);
        bool enableI2C_Global(bool state = true);
		bool enableI2C_External(bool state = true);
		bool enableSD(bool state = true);
		bool enableAuxPower(bool state);
		time_t getTime();
		uint8_t syncTime();
		bool startTimer(time_t period = 0); //Default to 0, if 0, use default timer period
		bool waitUntilTimerDone();
		// time_t getTime();
		String getTimeString();
		String getData(time_t time);
		String getErrors();
		String getMetadata();
		String selfDiagnostic(uint8_t diagnosticLevel, time_t time);
		uint8_t totalErrors() {
			return numErrors + rtc.numErrors; 
		}

        static constexpr uint8_t numTalonPorts = 5; 
		static constexpr int MAX_MESSAGE_LENGTH = 1024; ///<Maximum number of characters allowed for single transmission 
		// static constexpr uint16_t 
		
		dateTimeStruct currentDateTime = {2049, 6, 16, 3, 27, 31, TimeSource::NONE}; //Initialize with dummy time //DEBUG!
		bool statLED(bool state);
		bool setIndicatorState(uint8_t ledBank, uint8_t mode);
		uint8_t updateTime();
		bool feedWDT();
		String getPosLat();
		String getPosLong();
		time_t getPosTime();
		String getPosTimeString();
		bool configTalonSense();
		unsigned long getMessageID();
		bool testForBat();


    private:
        PCAL9535A ioOB;
        PCAL9535A ioTalon;
		MCP79412 rtc;
		PAC1934 csaAlpha;
		PAC1934 csaBeta;
		VEML3328 als;
		Adafruit_SHT4x atmos;
		MXC6655 accel; 

		
		PCA9634 led;
		const int ledBrightness = 50; //Default to 50% on
		const int ledPeriod = 500; //Default to 500ms period
		const int ledOnTime = 250; //Default to 50% duty cycle
        // uint32_t errors[MAX_NUM_ERRORS] = {0};
        // uint8_t numErrors = 0; //Used to track the index of errors array
        // bool errorOverwrite = false; //Used to track if errors have been overwritten in time since last report
        const uint32_t portErrorCode = 0x0F0; //Used to easily OR with error codes to add the Kestrel ID
		const time_t defaultPeriod = 300; //Default logging period of 300 seconds
		time_t logPeriod = 0; //Used to store the current log period
        // int throwError(uint32_t error);
		time_t timerStart = 0; //Start time for timer 
		bool criticalFault = false; 
		static Kestrel* selfPointer;
		static void timechange_handler(system_event_t event, int param);
		bool timeSyncRequested = false; ///<Used to indicate to the system that a time sync was requested from Particle and not to override
		time_t timegm(struct tm *tm); //Portable implementation
		time_t maxTimeError = 30; //Max time error allowed between clock sources [seconds]
		bool timeGood = false; ///<Keep track of the legitimacy of the time based on the last sync attempt
		uint8_t timeSource = 0; ///<Keep track of where the time is coming from
		time_t timeSyncVals[3] = {0}; ///<Keep track of what the values of each device where the last time syncTime was called
		time_t lastTimeSync = 0; ///<Keep track of when the last time sync occoured 
		long latitude = 0; ///<Used to keep track of the last pos measurment 
		long longitude = 0; ///<Used to keep track of the last pos measurment 
		time_t posTime = 0; ///<Time last postition measurment was taken
		bool initDone = false; //Used to keep track if the initaliztion has run - used by hasReset() 

};		

// constexpr uint8_t Kestrel::numTalonPorts; 

#endif