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


int counter=0, counter1=0;
double timer0, timer1, timer2, timer3, timer4, timer5;
double timer6, timer7;
double motor_speed=0, motor_speed1=0;
int motor_speed_theoric;

ISR(INT0_vect)
{
	timer0 = micros();
	if (counter == 0)
	{
		timer1 = timer0;
	}

	counter++;

	if (counter == 7)
	{
    	if (timer0 > timer1)
    	{
      		timer2 = timer0 - timer1;
			motor_speed = timer2 / 1000;
			motor_speed = motor_speed / 1000;
			motor_speed = 60 / motor_speed;
			motor_speed = motor_speed / 50;

			if (motor_speed > motor_speed1)
			{
			  SetpointMotor1 = map(motor_speed1, 0, 420, 0, 1023);
			}
			else
			{
			  SetpointMotor1 = motor_speed_theoric;
			}
    	}
		counter = 0;
		distance1++;
	}
}

ISR(INT1_vect)
{
	timer3 = micros();
	if (counter1 == 0)
	{
		timer3 = timer4;
	}

	counter++;

	if (counter1 == 7)
	{
    	if (timer3 > timer4)
    	{
      		timer5 = timer3 - timer4;
			motor_speed = timer5 / 1000;
			motor_speed = motor_speed1 / 1000;
			motor_speed = 60 / motor_speed1;
			motor_speed = motor_speed1 / 50;

			if (motor_speed1 > motor_speed)
			{
			  SetpointMotor2= map(motor_speed, 0, 420, 0, 1023);
			}
			else
			{
			  SetpointMotor2 = motor_speed_theoric;
			}
    	}
		counter1 = 0;
		distance2++;
	}
}

EtkinClass::EtkinClass(){
	trig = 13;
	echo = 12;
	motor1_dir = 16;
	motor2_dir = 4;
	motor1_pwm = 10;
	motor2_pwm = 11;
	rgb_red = 6;
	rgb_green = 5;
	rgb_blue = 9;
	cny70_R = A6;
	cny70_L = A3;

	pinMode(motor1_dir, OUTPUT); //cizildi
	pinMode(motor2_dir, OUTPUT); //cizildi blue rgb
	pinMode(motor1_pwm, OUTPUT); //cizildi
	pinMode(motor2_pwm, OUTPUT); //cizildi green rgb
	pinMode(rgb_red, OUTPUT); //cizildi red rgb
	pinMode(rgb_green, OUTPUT); //cizildi
	pinMode(rgb_blue, OUTPUT); //cizildi

	Kp = 0.002;
	Ki = 3;
	Kd = 0.001;

	myPID1 = new PID(&InputMotor1, &OutputMotor1, &SetpointMotor1, Kp, Ki, Kd, DIRECT);
	myPID2 = new PID(&InputMotor2, &OutputMotor2, &SetpointMotor2, Kp, Ki, Kd, DIRECT);

	myPID1->SetMode(AUTOMATIC);
	myPID2->SetMode(AUTOMATIC);

	myPID1->SetOutputLimits(AUTOMATIC);
	myPID2->SetOutputLimits(AUTOMATIC);

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
/*ledcolor()***********************************************************/
void EtkinClass::ledcolor(int red, int green, int blue){
	analogWrite(rgb_blue, blue);
	analogWrite(rgb_green, green);
	analogWrite(rgb_red, red);

}
/*motor()*************************************************************/
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
/*move()***********************************************************/
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

	motor_speed_theoric = 800;
	Setpoint = motor_speed_theoric;
	Setpoint1 = motor_speed_theoric;
	EtkinClass::move(direction, speed)
	digitalWrite(rgb_red, LOW);
	digitalWrite(rgb_green, LOW);
	digitalWrite(rgb_blue, LOW);

	while (1)
  {
    InputMotor1 = map(motor_speed1, 0, 420, 0, 1023);
    InputMotor2 = map(motor_speed, 0, 420, 0, 1023);
    myPID1->Compute();
    myPID2->Compute();


    pwmWriteDistance(Output, Output1, direction, (distance / 2.953));
    speed_control();
		if(Output1 < 80 && Output < 80)
		{
			break;
		}
  }
}

/*joystick()**********************************************************/
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
/*initINT()******************************************************
*Encoderli DC motorlarda bulunan encoderlerin kesme girisleri pin2
ve pin3'e baglanmistir.
*Bu metotda kesme icin gerekli ayarlamalar yapilmaktadir.
*/
void initINT()
{
	//Pin 2 ve 3 giris olarak ayarlandi
	DDRD &= ~(1 << 2);
	DDRD &= ~(1 << 3);
	//Pin 2 ve 3 pullup
	PORTD |= (1 << 2);
	PORTD |= (1 << 3);
	//Int0 ve Int1 icin dusen kenar tetiklemesi
	EICRA |= (1 << ISC01);
	EICRA |= (1 << ISC11);
	//Int0 ve Int1 aktif edildi
	EIMSK |= (1<< INT0);
	EIMSK |= (1<< INT1);
	//global kesme aktif et
	sei();
}
