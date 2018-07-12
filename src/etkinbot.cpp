/*
## ETKİNBOT AKILLI BOCEK ROBOTİK EĞİTİM KİTİ

Yerleşik özellikler;

# 2 adet encoderli Mikro dc motor ** motor 1
# Kullanıcı butonu
# Buzzer **
# RGB Led ***
# Çizgi izleme sensörleri
# Geliştirme Pinleri
--------------------------------------------------------------------------------
* Enkoderli DC Motorlar
- Motor1 = Sol Motor
		-Yön pini = 16(A2) *
		-Pwm pini = 10 *
		-Encoder pini = 3
- Motor2 = Sağ Motor
		-Yön pini = 4 *
		-Pwm pini = 11 *
 		-Encoder pini = 2
- Buzzer
		-pin 8 *
- Çizgi izleme sensorleri
		-Sağ sensör A6
		-Sol Sensör A3

--------------------------------------------------------------------------------
arge@etkinketnolojiler.com

*/

//pid eklenecek


#include "etkinbot.h"



uint8_t trig = 13;
uint8_t echo = 12;
uint8_t motor1_dir = 16;
uint8_t motor2_dir = 4;
uint8_t motor1_pwm = 10;
uint8_t motor2_pwm = 11;
uint8_t rgb_red = 6;
uint8_t rgb_green = 5;
uint8_t rgb_blue = 9;
uint8_t cny70_R = A6;
uint8_t cny70_L = A3;


EtkinClass::EtkinClass(){
	pinMode(motor1_dir, OUTPUT); //cizildi
	pinMode(motor2_dir, OUTPUT); //cizildi blue rgb
	pinMode(motor1_pwm, OUTPUT); //cizildi
	pinMode(motor2_pwm, OUTPUT); //cizildi green rgb
	pinMode(rgb_red, OUTPUT); //cizildi red rgb
	pinMode(rgb_green, OUTPUT); //cizildi
	pinMode(rgb_blue, OUTPUT); //cizildi
	//encoder eklenecek
	PID myPID1(&InputMotor1, &OutputMotor1, &SetpointMotor1, Kp, Ki, Kd, DIRECT);
	PID myPID2(&InputMotor2, &OutputMotor2, &SetpointMotor2, Kp, Ki, Kd, DIRECT);

	sei();
	level = 0;
	distance = 0;
	duration = 0;
	line_left = 0;
	line_right = 0;

}

/* distancesensor()************************************************************
*Hc-Sr04 Ultarasonik mesafe sensörü ile mesafe ölçümü.
******************************************************************************/

double EtkinClass::distancesensor(){
	digitalWrite(trig, LOW);
	delayMicroseconds(2);
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	duration = pulseIn(echo, HIGH);
	distance = duration / 58.2;
	delay(50);
	return distance;
}

/* linesensor()************************************************************
*Robotun üzerinde bulunan iki adet Cny70 çizgi sensorleri ile, çizgi izleme
yapılabilirinir.
*Sınır değeri olarak 100 ayarlanmıştır.
*
******************************************************************************/
int EtkinClass::linesensor(){
	int threshold_value  = 100;
	line_left = analogRead(cny70_L);
	line_right = analogRead(cny70_R);
	if (line_left < threshold_value && line_right < threshold_value){
		return 3;
	}
	else if (line_left < threshold_value && line_right > threshold_value){
		return 2;
	}
	else if (line_left > threshold_value && line_right < threshold_value){
		return 1;
	}
	else {
		return 0;
	}
}
void EtkinClass::ledcolor(int red, int green, int blue){
	analogWrite(rgb_blue, blue);
	analogWrite(rgb_green, green);
	analogWrite(rgb_red, red);

}
void EtkinClass::motor(int motor_id, int speed){
	if (motor_id == 1) {
		if (speed < 0) {
			digitalWrite(motor1_dir, LOW);
			speed = -speed;
		}
		else if (speed > 0){
			digitalWrite(motor1_dir, HIGH);
		}
		else {
			digitalWrite(motor1_dir, LOW);
		}
		analogWrite(motor1_pwm, speed);
	}
	if (motor_id == 2) {
		if (speed < 0) {
			digitalWrite(motor2_dir, LOW);
			speed = -speed;
		}
		else if (speed > 0){
			digitalWrite(motor2_dir, HIGH);
		}
		else {
			digitalWrite(motor2_dir, LOW);
		}
		analogWrite(motor2_pwm, speed);
	}
}
void EtkinClass::move(int direction, int speed){

/*
	direction = 1 --> ileri(forward)
	direction = 2 --> geri(backward)
	direction = 3 --> sola dön(left)
	direction = 4 --> sağa dön(right)
*/
	if (direction == 1){
		digitalWrite(motor1_dir, HIGH);
		digitalWrite(motor2_dir, HIGH);
	}else if(direction == 2){
		digitalWrite(motor1_dir, LOW);
		digitalWrite(motor2_dir, LOW);
	}else if(direction == 3){
		digitalWrite(motor1_dir, LOW);
		digitalWrite(motor2_dir, HIGH);
	}else if(direction == 4){
		digitalWrite(motor1_dir, HIGH);
		digitalWrite(motor2_dir, LOW);
	}
	analogWrite(motor1_pwm, speed);
  analogWrite(motor2_pwm, speed);
}
void EtkinClass::movePid(uint8_t direction, int speed, int distance)
{

	Setpoint = motor_speed_theoric;
	Setpoint1 = motor_speed_theoric;
	EtkinClass::move(direction, speed)
	digitalWrite(rgb_red, LOW);
	digitalWrite(rgb_green, LOW);
	digitalWrite(rgb_blue, LOW);

	while (1)
  {
    Input1 = map(motor_speed1, 0, 420, 0, 1023);
    Input = map(motor_speed, 0, 420, 0, 1023);
    myPID1.Compute();
    myPID2.Compute();


    pwmWriteDistance(Output, Output1, direction, (distance / 2.953));
    speed_control();
		if(Output1 == 0 && Output == 0)
		{
			break;
		}
  }
}


bool EtkinClass::joystick(int data){
	if(data==1){
		if(analogRead(A0)>650){
			return true;
		}
		else return false;
	}
	else if(data==2){
		if(analogRead(A0)<350){
			return true;
		}
		else return false;

	}
	else if(data==3){
		if(analogRead(A1)>650){
			return true;
		}
		else return false;
	}
	else{
		if(analogRead(A1)<350){
			return true;
		}
		else return false;
	}

}
