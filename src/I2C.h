/* *
 * Name        : I2C.cpp
 * Author      : Cristian Alonso
 * Version     :
 * Created on  : 2016-10-09
 *
 * Copyright © 2016 Cristian Alonso <cr.alonsov@gmail.com>
 */

#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

#define BUFFER_SIZE 0x01  //1 byte buffer


class I2C {
public:
	I2C(int, int);
	virtual ~I2C();
	uint8_t dataBuffer[BUFFER_SIZE];
	uint8_t read_byte(uint8_t);
	uint8_t write_byte(uint8_t, uint8_t);
private:
	int _i2caddr;
	int _i2cbus;
	void openfd();
	char busfile[64];
	int fd;
};

#endif /* _I2C_H_ */
