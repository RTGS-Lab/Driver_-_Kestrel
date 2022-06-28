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

#include <PCAL9535A.h>
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

class Kestrel
{
    constexpr static int MAX_NUM_ERRORS = 10; ///<Maximum number of errors to log before overwriting previous errors in buffer
	
    const uint32_t PORT_RANGE_ERROR = 0xF000; //FIX! 

	

    public:
        Kestrel();
        String begin();
        bool enablePower(uint8_t port, bool state = true);
        bool enableData(uint8_t port, bool state = true);
        bool disablePowerAll();
        bool disableDataAll();
        bool enableI2C_OB(bool state = true);
        bool enableI2C_Global(bool state = true);
		bool enableSD(bool state = true);
		bool enableAuxPower(bool state);
		time_t getTime();

        static constexpr uint8_t numTalonPorts = 4; 
		static constexpr int MAX_MESSAGE_LENGTH = 1024; ///<Maximum number of characters allowed for single transmission 
		// static constexpr uint16_t 
		struct {
		int year;
		int month;
		int day;
		int hour;
		int minute;
		int second;
		uint8_t source;
		} currentDateTime;

		bool updateTime();

    private:
        PCAL9535A ioOB;
        PCAL9535A ioTalon;
        uint32_t errors[MAX_NUM_ERRORS] = {0};
        uint8_t numErrors = 0; //Used to track the index of errors array
        bool errorOverwrite = false; //Used to track if errors have been overwritten in time since last report
        const uint32_t portErrorCode = 0x0F0; //Used to easily OR with error codes to add the Kestrel ID

        int throwError(uint32_t error);
		

};

// constexpr uint8_t Kestrel::numTalonPorts; 

#endif