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
    double speed = 395.0;
    char a;

    //pwm.setPwm(motor, SERVO_MIN);
    //std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    std::this_thread::sleep_for(std::chrono::microseconds(2000000));
    pwm.setPwm(steering, DIR_REST);
	std::this_thread::sleep_for(std::chrono::microseconds(1000000));
	pwm.setPwm(motor, speed);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));


    while(a != '2')
	{
	    std::cout << speed << std::endl;
	    std::cin >> a;
	    
	    if(a == 'w') speed -= 5;
	    if(a == 's') speed += 5;
	    if(a == '1') speed -= 1;
	    if(a == '2') speed = 0;
	    
	    pwm.setPwm(motor, speed);
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
	}
	
    pwm.setPwm(motor, 0);
    
	pwm.setPwm(steering, DIR_REST);
	std::this_thread::sleep_for(std::chrono::microseconds(1000000));
	pwm.setAllPwm(0, 0);

	return 0;

}
