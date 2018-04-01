/*
 * Name        : PCA9685.cpp
 * Author      : Cristian Alonso
 * Version     : 0.0.0.1
 * Created on  : Jul 16, 2017
 *
 * Copyright © 2017 Cristian Alonso <cr.alonsov@gmail.com>
 */

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <syslog.h>		/* Syslog functionallity */
#include <inttypes.h>
#include <errno.h>
#include <math.h>
#include <ctime>

#include "PCA9685.h"

//! Constructor takes bus and address arguments
/*!
 \param bus the bus to use in /dev/i2c-%d.
 \param address the device address on bus
 */
PCA9685::PCA9685(int bus, int address)
{
	i2c = new I2C(bus,address);
	reset();
	setAllPwm(0,0);
	i2c->write_byte(PCA9685_MODE2, OUTDRV);
	i2c->write_byte(PCA9685_MODE1, ALLCALL);
	usleep(5000); // Wait for oscillator (5 ms)
	uint8_t mode1 = i2c->read_byte(PCA9685_MODE1);
	mode1 = mode1 & ~SLEEP; // Wake up (reset sleep)
	i2c->write_byte(PCA9685_MODE1, mode1);
	usleep(5000); // Wait for oscillator (5 ms)
}


PCA9685::~PCA9685()
{
	delete i2c;
}


//! Sets PCA9685 mode to 00
void PCA9685::reset()
{

		i2c->write_byte(PCA9685_MODE1, 0x00); //Normal mode
		i2c->write_byte(PCA9685_MODE2, 0x04); //totem pole
}


//! Set the frequency of PWM
/*!
 \param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void PCA9685::setPwmFreq(int freq)
{
		double prescaleVal = (CLOCK_FREQ / 4096.0 / double(freq))  - 1;
		uint8_t prescale = floor(prescaleVal + 0.5);

		uint8_t oldMode = i2c->read_byte(PCA9685_MODE1);
		uint8_t newMode = (oldMode&0x7F) | 0x10; // Sleep

		i2c->write_byte(PCA9685_MODE1, newMode); // Go to sleep
		i2c->write_byte(PRESCALE, prescale); // Set the prescaler (multiplyer for PWM frequency)
		i2c->write_byte(PCA9685_MODE1, oldMode); // Restart
		usleep(5000); // Wait for oscillator (5 ms)
		i2c->write_byte(PCA9685_MODE1, oldMode | 0x80);
}


//! PWM a single channel
/*!
 \param channel channel to set PWM value for
 \param value 0-4095 value for PWM
 */
void PCA9685::setPwm(const uint8_t channel, int value)
{
	setPwm(channel, 0, value);
}


//! PWM a single channel with custom on time
/*!
 \param channel channel to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void PCA9685::setPwm(const uint8_t channel, int on_value, int off_value)
{
		i2c->write_byte(LED0_ON_L + 4 * channel, on_value & 0xFF);
		i2c->write_byte(LED0_ON_H + 4 * channel, on_value >> 8);
		i2c->write_byte(LED0_OFF_L + 4 * channel, off_value & 0xFF);
		i2c->write_byte(LED0_OFF_H + 4 * channel, off_value >> 8);
}


//! PWM every channel with custom on time
/*!
 \param channel channel to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void PCA9685::setAllPwm(int on_value, int off_value)
{
		i2c->write_byte(ALL_LED_ON_L, on_value & 0xFF);
		i2c->write_byte(ALL_LED_ON_H, on_value >> 8);
		i2c->write_byte(ALL_LED_OFF_L, off_value & 0xFF);
		i2c->write_byte(ALL_LED_OFF_H, off_value >> 8);
}


int PCA9685::getPwm(const uint8_t channel)
{
	int ledval = 0;
	ledval = i2c->read_byte(LED0_OFF_H + 4 * channel);
	ledval = ledval & 0xf;
	ledval <<= 8;
	ledval += i2c->read_byte(LED0_OFF_L + 4 * channel);
	return ledval;
}
