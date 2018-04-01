#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/types.h>
#include <chrono>
#include <thread>

#include "PCA9685.h"

int main()
{
    PCA9685 pwm(1, 0x40);
    pwm.setPwmFreq(65);

    const uint8_t motor = 0;
    const uint8_t steering = 1;

    //pwm.setPwm(motor, SERVO_MIN);
    //std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    pwm.setPwm(steering, LEFT_MAX);
	std::this_thread::sleep_for(std::chrono::microseconds(1000000));
	pwm.setPwm(steering, RIGHT_MAX);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    
	int a;

    /*do
	{
	    std::cout << pwm.getPwm(motor);
	} while  (std::cin >> a);*/
	
	pwm.setPwm(steering, DIR_REST);
	std::this_thread::sleep_for(std::chrono::microseconds(1000000));
	pwm.setAllPwm(0, 0);

	return 0;

}
