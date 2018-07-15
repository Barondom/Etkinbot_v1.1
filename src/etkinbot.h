
#ifndef etkinbot_H
#define etkinbot_H

#include <Arduino.h>
#include <PID_v1.h>
///@brief Class for EtkinClass
class EtkinClass
{
	public:
		EtkinClass();
		double distancesensor();
		int linesensor();
		bool joystick(int data);
		void movePid(uint8_t direction, int speed, int distance);
		void pwmWriteDistance_1(int out, int dir, int dist);
		void pwmWriteDistance_2(int out, int dir, int dist);
		void stop(bool urgentStop);
		void move(int direction, int speed);
		void motor(int motor_id, int speed);
		void ledcolor(int red, int green, int blue);
		uint8_t trig;
        uint8_t echo;
		PID * myPID1;
		PID * myPID2;


	private:
		int level;
		int line_left;
		int line_right;
		double distance;
		long duration;
		uint8_t motor1_dir;
		uint8_t motor2_dir;
		uint8_t motor1_pwm;
		uint8_t motor2_pwm;
		uint8_t rgb_red;
		uint8_t rgb_green;
		uint8_t rgb_blue;
		uint8_t cny70_R;
		uint8_t cny70_L;

};
#endif
