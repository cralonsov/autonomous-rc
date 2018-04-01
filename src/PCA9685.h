#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <stdint.h>
#include "I2C.h"

// Registers & etc:
const int PCA9685_ADDRESS	= 0x40;
const int PCA9685_MODE1		= 0x00;
const int PCA9685_MODE2		= 0x01;
const int PCA9685_SUBADR1	= 0x02;
const int PCA9685_SUBADR2	= 0x03;
const int PCA9685_SUBADR3	= 0x04;
const int PRESCALE			= 0xFE;
const double CLOCK_FREQ		= 25000000.0;

const int LED0_ON_L			= 0x06;
const int LED0_ON_H			= 0x07;
const int LED0_OFF_L		= 0x08;
const int LED0_OFF_H		= 0x09;
const int ALL_LED_ON_L		= 0xFA;
const int ALL_LED_ON_H		= 0xFB;
const int ALL_LED_OFF_L		= 0xFC;
const int ALL_LED_OFF_H		= 0xFD;


// Bits:
const int RESTART			= 0x80;
const int SLEEP				= 0x10;
const int ALLCALL			= 0x01;
const int INVRT				= 0x10;
const int OUTDRV			= 0x04;

// Configure min and max servo pulse lengths
const int SERVO_MIN			= 260;  // Min pulse length out of 4096
const int SERVO_MAX			= 520;  // Max pulse length out of 4096
const int RIGHT_MAX			= 260;  // Min pulse length out of 4096
const int LEFT_MAX			= 520;  // Max pulse length out of 4096
const int DIR_REST			= 395;  // Steering wheel in the center position

class PCA9685 {
public:
	PCA9685(int bus, int address);
	virtual ~PCA9685();

	void setPwmFreq(int freq);
	void setPwm(const uint8_t channel, int value);
	void setPwm(const uint8_t channel, int on_value, int off_value);
	void setAllPwm(int on_value, int off_value);
	int getPwm(const uint8_t channel);

private:
	I2C *i2c;
	void reset(void);
};

#endif // _PCA9685_H_
