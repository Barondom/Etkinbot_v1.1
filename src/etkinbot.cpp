/*
## ETKİNBOT AKILLI BOCEK ROBOTİK EĞİTİM KİTİ

Yerleşik özellikler;

# 2 adet encoderli Mikro dc motor
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
* Buzzer
		-pin 8 *
* Çizgi izleme sensorleri
		-Sağ sensör A6
		-Sol Sensör A3

--------------------------------------------------------------------------------
Çağatay YILMAZ
arge@etkinketnolojiler.com

Doğukan TUNCER
tasarım@etkinteknolojiler.com
*/

//#include<avr/interrupt.h>
#include "etkinbot.h"


volatile int counter =0, counter1 = 0;
volatile double timer0, timer1, timer2, timer3, timer4, timer5;
double timer6, timer7;
volatile double motor1_speed = 0, motor2_speed = 0;
int motor_speed_theoric;
void stop(bool urgentStop);
void pwmWriteDistance_1(int out, int dir, int dist);
void pwmWriteDistance_2(int out, int dir, int dist);
void initINT();
volatile int distance1 = 0, distance2 = 0;
int distancein = 0;
double InputMotor1,OutputMotor1,SetpointMotor1, InputMotor2,OutputMotor2,SetpointMotor2;
double Kp, Ki, Kd;
bool one_loop=0;

//************************************************************
//Interrupt 1
//************************************************************
ISR(INT0_vect)
{
	timer0 = micros();

	if (counter == 0)
		timer1 = timer0;

	counter++;

	if (counter == 7)
	{
		if (timer0 > timer1)
		{
			timer2 = timer0 - timer1;
			motor2_speed = timer2 / 1000;
			motor2_speed = motor2_speed / 1000;
			motor2_speed = 60 / motor2_speed;
			motor2_speed = motor2_speed / 50;

			//************************************************************
			//PID Setpoint Ayarlanıyor
			//************************************************************
			if (motor2_speed > motor1_speed)
			{
				SetpointMotor2 = map(motor1_speed, 0, 420, 0, 1023);
			}
			else
			{
				SetpointMotor2 = motor_speed_theoric;
			}
		}
		counter = 0;
		distance1++;
	}
}
//************************************************************
//Interrupt 2
//************************************************************
ISR(INT1_vect)
{
  timer3 = micros();

  if (counter1 == 0)
    timer4 = timer3;

  counter1++;

  if (counter1 == 7)
  {
    if (timer3 > timer4)
    {
      timer5 = timer3 - timer4;
      motor1_speed = timer5 / 1000;
      motor1_speed = motor1_speed / 1000;
      motor1_speed = 60 / motor1_speed;
      motor1_speed = motor1_speed / 50;

		//************************************************************
		//PID Setpoint Ayarlanıyor
		//************************************************************
      if (motor1_speed > motor2_speed)
      {
        SetpointMotor1 = map(motor2_speed, 0, 420, 0, 1023);
      }
      else
      {
        SetpointMotor1 = motor_speed_theoric;
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


	pinMode(motor1_dir, OUTPUT);
	pinMode(motor2_dir, OUTPUT);
	pinMode(motor1_pwm, OUTPUT);
	pinMode(motor2_pwm, OUTPUT);
	pinMode(rgb_red, OUTPUT);
	pinMode(rgb_green, OUTPUT);
	pinMode(rgb_blue, OUTPUT);

	Kp = 0.002;
	Ki = 3;
	Kd = 0.001;

	myPID1 = new PID(&InputMotor1, &OutputMotor1, &SetpointMotor1, Kp, Ki, Kd, DIRECT);
	myPID2 = new PID(&InputMotor2, &OutputMotor2, &SetpointMotor2, Kp, Ki, Kd, DIRECT);

	myPID1->SetMode(AUTOMATIC);
	myPID2->SetMode(AUTOMATIC);

	myPID1->SetOutputLimits(0, 255);
	myPID2->SetOutputLimits(0, 255);

	initINT();

	level = 0;
	distanceHcsr04 = 0;
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
	distanceHcsr04 = duration / 58.2;
	delay(50);
	return distanceHcsr04;
}

/* linesensor()************************************************************
*Robotun üzerinde bulunan iki adet Cny70 çizgi sensorleri ile, çizgi izleme
yapılabilirinir.
*Sınır değeri olarak 100 ayarlanmıştır.
*Metod Donus degerleri
	-iki sensor siyah cizgide = 3
	-sag sensor beyazda = 2
	-sol sensor beyazda = 1
	-ikisi de beyazda = 0
******************************************************************************/
int EtkinClass::linesensor(){
	int threshold_value  = 110;
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
void EtkinClass::move(int direction, int speed)
{
/*
	direction = 1 --> ileri(forward)
	direction = 2 --> geri(backward)
	direction = 3 --> sola dön(left)
	direction = 4 --> sağa dön(right)
*/
	if (direction == 1)
	{
		digitalWrite(motor1_dir, HIGH);
		digitalWrite(motor2_dir, LOW);
	}
	else if(direction == 2)
	{
		digitalWrite(motor1_dir, LOW);
		digitalWrite(motor2_dir, HIGH);
	}
	else if(direction == 3){

		digitalWrite(motor1_dir, LOW);
		digitalWrite(motor2_dir, LOW);
	}
	else if(direction == 4)
	{
		digitalWrite(motor1_dir, HIGH);
		digitalWrite(motor2_dir, HIGH);
	}
	analogWrite(motor1_pwm, speed);
  	analogWrite(motor2_pwm, speed);
}
/*movePID()*************************************************************
*move metodundan farkli olarak bu metod motorlarin hiz ve konum kontrolunu
saglayarak dogrusal olarak hareket etmesi saglanmistir.
*/
void EtkinClass::movePid(uint8_t _direction, int _speed, int distance)
{

	motor_speed_theoric = map(_speed, 0, 255, 0, 1023);
	SetpointMotor1 = motor_speed_theoric;
	SetpointMotor2 = motor_speed_theoric;
	EtkinClass::move(_direction, _speed);
	digitalWrite(rgb_red, LOW);
	digitalWrite(rgb_green, LOW);
	digitalWrite(rgb_blue, LOW);
  isFinished = 0;
	distance1 = 0;
	distance2 = 0;
	distancein = distance;
  distancein = distancein / 2.953;
	isFinished = 0;
	timer0	= micros();
	timer3	= micros();
	uint8_t _direction_1, _direction_2;

  if(_direction == 1)
  {
    _direction_1 = 1;
    _direction_2 = 1;
  }
  else if(_direction == 2)
  {
    _direction_1 = 2;
    _direction_2 = 2;
  }
  else if(_direction == 3)
  {
    _direction_1 = 2;
    _direction_2 = 1;
  }
  else if(_direction == 4)
  {
    _direction_1 = 1;
    _direction_2 = 2;
  }

	while (1)
  {
    InputMotor1 = map(motor1_speed, 0, 420, 0, 1023);
    InputMotor2 = map(motor2_speed, 0, 420, 0, 1023);
    myPID1->Compute();
    myPID2->Compute();
	if (one_loop == 0)
	{
		analogWrite(motor1_pwm, 150);
		analogWrite(motor2_pwm, 150);
		delay(50);
		one_loop = 1;
	}
    pwmWriteDistance_1(OutputMotor1, _direction_1, distancein );
    pwmWriteDistance_2(OutputMotor2, _direction_2, distancein );
    speed_control();
    if(isFinished == 1)
    {
      break;
    }
  }
	digitalWrite(motor1_pwm, LOW);
	digitalWrite(motor2_pwm, LOW);
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
	// DDRD &= ~(1 << 2);
	// DDRD &= ~(1 << 3);
	//Pin 2 ve 3 pullup
	// PORTD |= (1 << 2);
	// PORTD |= (1 << 3);

	pinMode(2, INPUT);
	pinMode(3, INPUT);
	//Int0 ve Int1 icin dusen kenar tetiklemesi
	EICRA |= (1 << ISC01);
	EICRA |= (1 << ISC11);
	//Int0 ve Int1 aktif edildi
	EIMSK |= (1<< INT0);
	EIMSK |= (1<< INT1);
	//global kesme aktif et
	sei();
}

/*pwmWriteDistance()*****************************************
*PID fonksiyonun ciktilarini motorlara pwm sinyalini verir
*/
void EtkinClass::pwmWriteDistance_1(int out, int dir, int dist)
{
  if (distance1 < dist)
  {
    if (dir == 1)
    {
      digitalWrite(motor1_dir, HIGH);
      analogWrite(motor1_pwm, out);
    }
    else if (dir == 2)
    {
      digitalWrite(motor1_dir, LOW);
      analogWrite(motor1_pwm, out);
    }
  }
  else
  {
    stopp(0);
  }
}

void EtkinClass::pwmWriteDistance_2(int out, int dir, int dist)
{
  if (distance2 < dist)
  {
    if (dir == 1)
    {
      digitalWrite(motor2_dir, LOW);
      analogWrite(motor2_pwm, out);
    }
    else if (dir == 2)
    {
      digitalWrite(motor2_dir, HIGH);
      analogWrite(motor2_pwm, out);
    }
  }
  else
  {
    stopp(0);
  }
}
/*stopp()**********************************************************************
* urgentStop degiskeni ile motorlarlar eger speed_control() metodu tarafindan
durduruldu ise Kirmizi led yanacaktir.
*eger pwmWriteDistance metodlarindan durduruldu ise Yesil led yanacaktır.
*/
void EtkinClass::stopp(bool urgentStop)
{
		digitalWrite(motor1_pwm, LOW);
		digitalWrite(motor2_pwm, LOW);

	if(urgentStop)
	{
		digitalWrite(rgb_red, HIGH);
		digitalWrite(rgb_green, LOW);
		digitalWrite(rgb_blue, LOW);
		digitalWrite(motor1_pwm, LOW);
		digitalWrite(motor2_pwm, LOW);
		delay(200);
	}
	else
	{
		digitalWrite(rgb_red, LOW);
		digitalWrite(rgb_green, HIGH);
		digitalWrite(rgb_blue, LOW);
		isFinished = 1;
		digitalWrite(motor1_pwm, LOW);
		digitalWrite(motor2_pwm, LOW);
		delay(200);
	}
}

/*speed_control()***************************************************************
*motorlardan herhangi biri dışarıdan müdehale ile durdugunda her iki motoruda
durdurur.
*/
void EtkinClass::speed_control()
{
  if (((micros() - timer0) > 80000) || ((micros() - timer3) > 80000))
  {
    stopp(1);
  }
}
