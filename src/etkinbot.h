
#ifndef etkinbot_H
#define etkinbot_H

#include <Arduino.h>
#include "PID_v1.h"

class EtkinClass
{
  public:
    EtkinClass();
    double distancesensor();
    int linesensor();
    bool joystick(int data);
    void movePid(uint8_t _direction, int _speed, int distance);
    void pwmWriteDistance_1(int out, int dir, int dist);
    void pwmWriteDistance_2(int out, int dir, int dist);
    void stopp(bool urgentStop);
    void move(int direction, int speed);
    void motor(int motor_id, int speed);
    void ledcolor(int red, int green, int blue);
    void speed_control();
	void interrupt_motor1();
	void interrupt_motor2();
    uint8_t trig;
    uint8_t echo;
    PID * myPID1;
    PID * myPID2;


  private:
    int level;
    int line_left;
    int line_right;
    double distanceHcsr04;
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
    bool isFinished;

};
#endif
