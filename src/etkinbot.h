
#ifndef etkinbot_h
#define etkinbot_h

#include <Arduino.h>
#include "PID_v1.h"
///@brief Class for EtkinClass
class EtkinClass
{
	public:
		EtkinClass();
		double distancesensor();
		int linesensor();
		bool joystick(int data);

		void move(int direction, int speed);
		void motor(int motor_id, int speed);
		void ledcolor(int red, int green, int blue);
	private:
		int level;
		int line_left;
		int line_right;
		double distance;
		long duration;
		uint8_t trig;
        uint8_t echo;
        uint8_t motor1_dir;
        uint8_t motor2_dir;
        uint8_t motor1_pwm;
        uint8_t motor2_pwm;
        uint8_t rgb_red;
        uint8_t rgb_green;
        uint8_t rgb_blue;
        uint8_t cny70_R;
        uint8_t cny70_L;
		PID * myPID1;
		PID * myPID2;

		double InputMotor1,OutputMotor1,SetpointMotor1, Kp, Ki, Kd, InputMotor2,OutputMotor2,SetpointMotor2;

};
#endif
