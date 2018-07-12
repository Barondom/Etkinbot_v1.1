//MAX motor Duty si 200 olacak. PID, çıkış sinyalini Setpointe yaklaştırmak stediğinde max motor dutysinden biraz fazla duty verebilmeli.
//myPID1: Robota arkadan bakınca sol motor
//myPID2: Robota arkadan bakınca sağ motor

//Tekerlek Çapı = 47mm
//Tekerlek çevresi = 1 tur: 2*pi*(47/2) = 147.65mm
//İç şaftta 1 tur, tekerlekte 2,953mm mesafeye eşit.


#include <PID_v1.h>

//3:Motor1 Enkoder
//2:Motor2 Enkoder
//A2:Motor1 İleri
//A2':Motor1 Geri
//4:Motor2 İleri
//4':Motor2 Geri
//10:Motor1 EN
//11:Motor2 EN
//7: BUTTON

const int EN1 = 10;
const int EN2 = 11;
const int Ileri1 = A2;
const int Ileri2 = 4;
const int Enkoder1 = 3;
const int Enkoder2 = 2;
const int button = 7;

int counter = 0;
float distance1 = 0, distance2 = 0, distancein1 = 0, distancein2 = 0;
int counter1 = 0;
int motor_speed_theoric = 800;
bool looping = 0;
double timer2, timer1, timer0, motor_speed = 0;
double timer5, timer4, timer3, motor_speed1 = 0;
double timer6, timer7;
double Setpoint, Input, Output;
double Kp = 0.002, Ki = 3 , Kd = 0.001;
double Setpoint1, Input1, Output1;
double Kp1 = 0.002, Ki1 = 3 , Kd1 = 0.001;

PID myPID1(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

void interrupt1()
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
      motor_speed = timer2 / 1000;
      motor_speed = motor_speed / 1000;
      motor_speed = 60 / motor_speed;
      motor_speed = motor_speed / 50;

      ////////////////////////////////////////////////////////////////////// PID Setpoint Ayarlanıyor
      if (motor_speed > motor_speed1)
      {
        Setpoint = map(motor_speed1, 0, 420, 0, 1023);
      }
      else
      {
        Setpoint = motor_speed_theoric;
      }
    }
    counter = 0;
    distance1++;
  }
}

void interrupt2()
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
      motor_speed1 = timer5 / 1000;
      motor_speed1 = motor_speed1 / 1000;
      motor_speed1 = 60 / motor_speed1;
      motor_speed1 = motor_speed1 / 50;

      ////////////////////////////////////////////////////////////////////// PID Setpoint Ayarlanıyor
      if (motor_speed1 > motor_speed)
      {
        Setpoint1 = map(motor_speed, 0, 420, 0, 1023);
      }
      else
      {
        Setpoint1 = motor_speed_theoric;
      }
    }
    counter1 = 0;
    distance2++;
  }
}

void setup()
{

  Serial.begin(250000);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(0, 255);
  myPID2.SetOutputLimits(0, 255);
  
  ////////////////////////////////////////////////////////////////////// Giriş Çıkış Ayarları
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(A2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(7, INPUT);

  attachInterrupt(1, interrupt1, FALLING);
  attachInterrupt(0, interrupt2, FALLING);
  ////////////////////////////////////////////////////////////////////// Pinler LOW'a Çekiliyor
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(Ileri1, LOW);
  digitalWrite(Ileri2, LOW);
}

void loop()
{
  if (digitalRead(button) == 1)
  {
    delay(200);
    looping = 1;
    distance1 = 0;
    distance2 = 0;
    Setpoint = motor_speed_theoric;
    Setpoint1 = motor_speed_theoric;
    digitalWrite(Ileri1, HIGH);
    digitalWrite(Ileri2, LOW);
    analogWrite(EN1, map(motor_speed_theoric, 0, 1023, 0, 255));
    analogWrite(EN2, map(motor_speed_theoric, 0, 1023, 0, 255));
    digitalWrite(3, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    delay(20);
  }
  while (looping)
  {
    Input1 = map(motor_speed1, 0, 420, 0, 1023);
    Input = map(motor_speed, 0, 420, 0, 1023);
    myPID1.Compute();
    myPID2.Compute();

    motor_speed_theoric = 800;
    distancein1 = 75; //(mm)
    distancein2 = 75; //(mm)
    distancein1 = distancein1 / 2.953;
    distancein2 = distancein2 / 2.953;
    //pwmWrite_1(Output, 2);
    //pwmWrite_2(Output1, 2);
    pwmWriteDistance_1(Output, 1, distancein1);
    pwmWriteDistance_2(Output1, 2, distancein2);

    Serial.print(distancein1);
    Serial.print("\t");
    Serial.println(distance1);


    speed_control();
  }
  stopp();
}

void pwmWrite_1(int out, int dir)
{
  if (dir == 1)
  {
    digitalWrite(Ileri1, LOW);
    analogWrite(EN1, out);
  }
  else if (dir == 2)
  {
    digitalWrite(Ileri1, HIGH);
    analogWrite(EN1, out);
  }
}

void pwmWrite_2(int out, int dir)
{
  if (dir == 1)
  {
    digitalWrite(Ileri2, HIGH);
    analogWrite(EN2, out);
  }
  else if (dir == 2)
  {
    digitalWrite(Ileri2, LOW);
    analogWrite(EN2, out);
  }
}

void pwmWriteDistance_1(int out, int dir, int dist)
{
  if (distance1 < dist)
  {
    if (dir == 1)
    {
      digitalWrite(Ileri1, LOW);
      analogWrite(EN1, out);
    }
    else if (dir == 2)
    {
      digitalWrite(Ileri1, HIGH);
      analogWrite(EN1, out);
    }
  }
  else
  {
    stopp();
  }
}

void pwmWriteDistance_2(int out, int dir, int dist)
{
  if (distance2 < dist)
  {
    if (dir == 1)
    {
      digitalWrite(Ileri2, HIGH);
      analogWrite(EN2, out);
    }
    else if (dir == 2)
    {
      digitalWrite(Ileri2, LOW);
      analogWrite(EN2, out);
    }
  }
  else
  {
    stopp();
  }
}

void speed_control()
{
  if (((micros() - timer0) > 80000) || ((micros() - timer3) > 80000))
  {
    stopp();
  }
}

void stopp()
{
  looping = 0;
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(3, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
}








