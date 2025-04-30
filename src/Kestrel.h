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
//#include <PCAL9535A.h>
//#include "../../PCA9634/src/PCA9634.h"
// #include "MCP7940_Library/src/MCP7940.h"
//#include "../../Driver_-_MCP79412/src/MCP79412.h"
//#include "../../SparkFun_u-blox_GNSS_Arduino_Library/src/SparkFun_u-blox_GNSS_Arduino_Library.h"
//#include "../../PAC1932_Library/src/PAC1934.h"
//#include "../../VEML3328/src/VEML3328.h"
//#include <Adafruit_SHT4x.h>
//#include <MXC6655.h>
#include <arduino_bma456.h>
// #include <GlobalPins.h>

#include "../../FlightControl-platform-dependencies/src/ITimeProvider.h"
#include "../../FlightControl-platform-dependencies/src/IGpio.h"
#include "../../FlightControl-platform-dependencies/src/ISystem.h"
#include "../../FlightControl-platform-dependencies/src/IWire.h"
#include "../../FlightControl-platform-dependencies/src/ICloud.h"
#include "../../FlightControl-platform-dependencies/src/ISerial.h"

#include "../../FlightControl-hardware-dependencies/src/IIOExpander.h"
#include "../../FlightControl-hardware-dependencies/src/ICurrentSenseAmplifier.h"
#include "../../FlightControl-hardware-dependencies/src/ILed.h"
#include "../../FlightControl-hardware-dependencies/src/IRtc.h"
#include "../../FlightControl-hardware-dependencies/src/IAmbientLight.h"
#include "../../FlightControl-hardware-dependencies/src/IGps.h"
#include "../../FlightControl-hardware-dependencies/src/IHumidityTemperature.h"
#include "../../FlightControl-hardware-dependencies/src/IAccelerometer.h"

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
	constexpr uint16_t GPS_INT = 7; 
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
	constexpr uint8_t INCREMENT = 4;
	constexpr uint8_t RTC = 3;
	constexpr uint8_t GPS_RTC = 2;
	constexpr uint8_t CELLULAR = 1;
	constexpr uint8_t GPS = 0; 
	constexpr uint8_t NONE = 5;
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

namespace AccelType {
	constexpr uint8_t MXC6655 = 0;
	constexpr uint8_t BMA456 = 1;
}

namespace HardwareVersion {
	constexpr uint8_t PRE_1v9 = 0;
	constexpr uint8_t MODEL_1v9 = 1;
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
	const String FIRMWARE_VERSION = "1.7.5"; //FIX! Read from system??
	
    const uint32_t KESTREL_PORT_RANGE_FAIL = 0x90010300; ///<Kestrel port assignment is out of range
	const uint32_t CSA_OB_INIT_FAIL = 0x100500F7; ///<Failure to initialize CSA Alpha or CSA Beta
	const uint32_t CSA_OB_READ_FAIL = 0x101600F7; ///<Failure to initialize CSA Alpha or CSA Beta
	const uint32_t GPS_INIT_FAIL = 0x100A00F8; ///<Failure to initialize the onboard GPS
	const uint32_t GPS_READ_FAIL = 0x100B00F8; ///<Failure to read from the onboard GPS
	// const uint32_t GPS_TIMEOUT = 0xF00C00F8; ///<Timeout ocoured while waiting for GPS to connect (>30 seconds)
	const uint32_t CELL_FAIL = 0xF00700F6; ///<Failure to connect to cell network
    const uint32_t CLOUD_FAIL = 0xF00800F6; ///<Failure to connect to particle cloud
	const uint32_t SYSTEM_RESET = 0xF00A0000; ///<Reported on first init, along with reason for reset
	const uint32_t CLOCK_MISMATCH = 0x700100F0; ///<Mismatch between consensus time and one of the sources 
	const uint32_t CLOCK_NO_SYNC = 0x500300F0; ///<No two clock sources agree, unable to provide synced time
	const uint32_t CLOCK_UNAVAILABLE = 0x500400F0; ///<One of the system clocks is unavailable to read from 
	const uint32_t RTC_OSC_FAIL = 0x500500F5; ///<Failure of local RTC to increment 
	const uint32_t RTC_POWER_LOSS = 0x54B200F5; ///<Local RTC has encountered power failure 
	const uint32_t RTC_READ_FAIL = 0x100C00F5; ///<Failure to read the onboard RTC
	const uint32_t WDT_OFF_LEASH = 0xE00200FA; ///<WDT has not been fed when requested due to an outstanding critical fault
	const uint32_t RAM_LOW = 0xF00B00FB; ///<RAM usage is greater than 75%
	const uint32_t RAM_CRITICAL = 0x400400FB; ///<RAM usage greater than 90%, calling for reset
	const uint32_t RAM_FULL = 0x400500FB; ///<RAM filled up such that a variable cannot be allocated, try to reset
	const uint32_t GPS_UNAVAILABLE = 0xF00C00F8; ///<Timed out while waiting to connect to GPS
	const uint32_t ACCEL_INIT_FAIL = 0x100C00F7; ///<Failed to initialize onboard accelerometer 
	const uint32_t ACCEL_DATA_FAIL = 0x100D00F7; ///<Failed to read data from onboard accelerometer
	const uint32_t ALARM_FAIL = 0x500600F5; ///<RTC alarm failed to wake device 
	const uint32_t TIME_DISAGREE = 0x70030000; ///<At least one time source disagrees with the others
	const uint32_t ALS_INIT_FAIL = 0x101400F7; ///<Failure to initialize the ALS on the Kestrel board
	const uint32_t ALS_DATA_FAIL = 0x101500F7; ///<Failure to read data from the ALS on the Kestrel board

	const time_t CELL_TIMEOUT = 300000; ///<Amount of time [ms] to wait while trying to connect to cell
    public:
        Kestrel(ITimeProvider& timeProvider,
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
				ILed& led,
				IRtc& rtc,
				IAmbientLight& als,
				IGps& gps,
				IHumidityTemperature& humidityTemp,
				IAccelerometer& accel,
				bool useSensors = false);
		IGps& m_gps;
        String begin(time_t time, bool &criticalFault, bool &fault);
		int sleep();
		int wake();
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
		bool sdInserted();
		bool enableAuxPower(bool state);
		time_t getTime();
		uint8_t syncTime(bool force = false);
		bool startTimer(time_t period = 0); //Default to 0, if 0, use default timer period
		bool waitUntilTimerDone();
		// time_t getTime();
		String getTimeString();
		String getData(time_t time);
		String getErrors();
		String getMetadata();
		String selfDiagnostic(uint8_t diagnosticLevel, time_t time);
		uint8_t totalErrors() {
			return numErrors + m_rtc.numErrors; 
		}
		bool updateLocation(bool forceUpdate = false);
		bool connectToCell();

        static constexpr uint8_t numTalonPorts = 5; 
		static constexpr int MAX_MESSAGE_LENGTH = 1024; ///<Maximum number of characters allowed for single transmission 
		// static constexpr uint16_t 
		
		dateTimeStruct currentDateTime = {2049, 6, 16, 3, 27, 31, TimeSource::NONE}; //Initialize with dummy time //DEBUG!
		uint8_t timeFix = 0; ///<Keep track of the quality of time fix 
		bool statLED(bool state);
		bool setIndicatorState(uint8_t ledBank, uint8_t mode);
		uint8_t updateTime();
		bool feedWDT();
		bool releaseWDT();
		String getPosLat();
		String getPosLong();
		String getPosAlt();
		time_t getPosTime();
		String getPosTimeString();
		bool configTalonSense();
		unsigned long getMessageID();
		bool testForBat();
		bool zeroAccel(bool reset = false);


    private:
		ITimeProvider& m_timeProvider;
		IGpio& m_gpio;
		ISystem& m_system;
		IWire& m_wire;
		ICloud& m_cloud;
		ISerial& m_serialDebug;
		ISerial& m_serialSdi12;

		IIOExpander& m_ioOB;
		IIOExpander& m_ioTalon;

		ICurrentSenseAmplifier& m_csaAlpha;
		ICurrentSenseAmplifier& m_csaBeta;

		ILed& m_led;
		
		IRtc& m_rtc;
		IAmbientLight& m_als;
		IHumidityTemperature& m_humidityTemp;
		IAccelerometer& m_accel; 

		const int ledBrightness = 75; //Default to 75% on
		const int ledPeriod = 500; //Default to 500ms period
		const int ledOnTime = 250; //Default to 50% duty cycle
		const unsigned long sysCollectMax = 300000; //Allow for a max of 5 minutes for collecting info from system
		const unsigned long loggerCollectMax = 30000; //Allow for a max of 30 seconds for collecting info from logger itself (1 = data, 2 = diagnostic, 3 = metadata)
        // uint32_t errors[MAX_NUM_ERRORS] = {0};
        // uint8_t numErrors = 0; //Used to track the index of errors array
        // bool errorOverwrite = false; //Used to track if errors have been overwritten in time since last report
        const uint32_t portErrorCode = 0x0F0; //Used to easily OR with error codes to add the Kestrel ID
		const time_t defaultPeriod = 300; //Default logging period of 300 seconds
		time_t logPeriod = 0; //Used to store the current log period
        // int throwError(uint32_t error);
		time_t timerStart = 0; //Start time for timer 
		bool criticalFault = false; 
		bool wdtRelease = false;
		bool updateGPS = false; ///<Don't try to update until ready 
		static Kestrel* selfPointer;
		static void timechange_handler(IEventType event, int param);
		static void outOfMemoryHandler(IEventType event, int param);
		bool timeSyncRequested = false; ///<Used to indicate to the system that a time sync was requested from Particle and not to override
		time_t timegm(struct tm *tm); //Portable implementation
		time_t maxTimeError = 30; //Max time error allowed between clock sources [seconds]
		bool timeGood = false; ///<Keep track of the legitimacy of the time based on the last sync attempt
		const uint8_t numClockSources = 6; 
    	bool sourceRequested[6] = {true, true, true, true, true, true}; ///<Keep track of which clock sources were asked for at each interval
		bool sourceAvailable[6] = {false, false, false, false, false, false}; ///<Keep track of which sources are available for testing against
    	String sourceNames[6] = {"GPS","CELL","GPS_RTC","RTC","INC","LOCAL"}; 
		time_t times[6] = {0}; ///<Actual time storage from last time check: gpsSatTime, cellTime, gpsTime, rtcTime, incrementTime, particleTime  
		int8_t timeSourceA = 5;
		int8_t timeSourceB = 5;
		// uint8_t timeSource = 0; ///<Keep track of where the time is coming from
		// time_t timeSyncVals[3] = {0}; ///<Keep track of what the values of each device where the last time syncTime was called
		time_t lastTimeSync = 0; ///<Keep track of when the last time sync occoured 
		long latitude = 0; ///<Used to keep track of the last pos measurment 
		long longitude = 0; ///<Used to keep track of the last pos measurment 
		long altitude = 0; ///<Used to keep track of the last pos measurment 
		time_t posTime = 0; ///<Time last postition measurment was taken
		bool initDone = false; //Used to keep track if the initaliztion has run - used by hasReset() 
		struct tm timeinfo = {0}; //Create struct in C++ time land
		time_t cstToUnix(int year, int month, int day, int hour, int minute, int second);
		uint8_t accelUsed = AccelType::MXC6655; //Default to MXC6655, only change is BMA456 is detected 
		uint8_t boardVersion = HardwareVersion::PRE_1v9; //Assume pre v1.8 to start
		bool reportSensors = false; //Default to sensor report being false
};		

// constexpr uint8_t Kestrel::numTalonPorts; 

#endif