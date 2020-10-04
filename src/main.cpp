/*
 *  Code for robot gathering contest
 *  
 *
 *
 *
*/

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

int DEBUG = 0; // Make it 0 in main run

#define trigPin A8  // Must connect to an analog pin
#define echoPin A10 // Must connect to an analog pin
#define STBY 5
#define rMin1 7
#define rMin2 6
#define lMin1 4
#define lMin2 3
#define pwL 2
#define pwR 12
#define btn1 23
#define btn2 29
#define btn3 22
#define btn4 28
#define btn5 34
#define led1 24
#define led2 26
#define led3 32
#define servo1_pin 8
#define servo2_pin 9
#define sda 20 // Important! cannot change this pin
#define scl 21 // Important! cannot change this pin

Servo servo1;
Servo servo2;
LiquidCrystal_I2C lcd(0x27, 16, 2);

//-------------------- IR SENSOR RELATED VARIABLES-----------------------------------------
boolean firstData[8]; /*to eliminate effect of noise*/
int v = 0;
unsigned int sensor[8];                              /*sensor readings are saved here*/
int sensorPin[8] = {A7, A6, A5, A4, A3, A2, A1, A0}; /*arduino pins to read sensors*/
byte NumOfSensors = 8;
byte i; /*just to run for loop!!*/
unsigned int MaxWaitTime = 1024;
byte sensorData;
//---------------------------------------------------------------------------------------------

char directions[3][100]; // memory of the track to follow -> have to be defined according to the track

double Const1 = 12.4;
double Const2 = 0;
double Const3 = 6.1;
double Shomakolon = 0;
double motorspeed = 150;
double velocity = 0;
double Vul = 0;
double PIDvalue, RSpeed, LSpeed;
double Parthokko = 0;
double AgerVul = 0;
double threshold[8] = {80, 80, 80, 80, 80, 80, 80, 80}; //Array for holding sensor threshold values
double t1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double t2[8] = {1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
unsigned long int memory[300]; // Memory of the path
int memory_length = 300;
int sumation;
int ct[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
int x[8];
int reading;
int sm = 0;
//----------------------Functions used in this code-------------------------------
void readSensors();
void generateBinary();
void generateThreshold();
void deviation();
void PIDval();
void doura();
void Forward(double del, int vel);
void Backward(double del, int vel);
void Right(double del, int vel);
void Left(double del, int vel);
void Stop(double del);
void Tleft();
void Tright();
void BreakR();
void BreakL();
void BreakF();
void pick_object();
void release_object();
double search();
void shift_right(int value);
void detection();

//-----------------------------Starting point------------------
void setup()
{
  Serial.begin(9600);
  pinMode(rMin1, OUTPUT);
  pinMode(rMin2, OUTPUT);
  pinMode(lMin1, OUTPUT);
  pinMode(lMin2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn5, INPUT_PULLUP);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  pinMode(pwR, OUTPUT);
  pinMode(pwL, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(STBY, HIGH);

  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);

  lcd.init();
  lcd.backlight();

  for (int load_memory = 0; load_memory < memory_length; load_memory++)
  {
    memory[load_memory] = 0;
  }
  while (true)
  {
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Calibrate?");
    lcd.setCursor(3, 1);
    lcd.print("btn4 = OK");
    delay(100);
    if (digitalRead(btn4) == LOW)
      break;
  }
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Calibrating...");
  generateThreshold();
  lcd.clear();
  while (true)
  {
    Stop(10);
    for (int threshold_iterator = 0; threshold_iterator < 4; threshold_iterator++)
    {
      lcd.setCursor(threshold_iterator * 3, 0);
      lcd.print((int)threshold[threshold_iterator] < 1000 ? threshold[threshold_iterator] : 999);
      lcd.setCursor(threshold_iterator * 3, 1);
      lcd.print((int)threshold[threshold_iterator + 4] < 1000 ? threshold[threshold_iterator + 4] : 999);
    }
    delay(10);
    if (digitalRead(btn4) == LOW)
      break;
  }
}
//------------------------------Main Loop--------------------
void loop()
{
  if (digitalRead(btn1) == LOW)
  {
    setup();
  }
  readSensors();
  generateBinary();
  deviation();
  PIDval();
  doura();
  if (sumation > 4)
  {
    detection();
  }
}
//-------------------------------------------------------------------
void shift_right(int value)
{
  memory[value] = memory[value - 1];
  if (value == 1)
    return;
  shift_right(value - 1);
}
//-------------------------------------------------------------------
void readSensors()
{
  /*
  * Code to read sensor data -> identical to the documentation with some simple modifications 
  * 
  */
  for (i = 0; i < NumOfSensors; i++)
  {
    digitalWrite(sensorPin[i], HIGH);
    pinMode(sensorPin[i], OUTPUT);
  }
  delayMicroseconds(20);
  for (i = 0; i < NumOfSensors; i++)
  {
    pinMode(sensorPin[i], INPUT);
    digitalWrite(sensorPin[i], LOW);
    sensor[i] = MaxWaitTime;
    firstData[i] = false;
  }
  unsigned long startTime = micros();

  while (micros() - startTime < MaxWaitTime)
  {
    unsigned int time = micros() - startTime;
    for (i = 0; i < NumOfSensors; i++)
    {
      if ((digitalRead(sensorPin[i]) == LOW) && (firstData[i] == false))
      {
        sensor[i] = time;
        firstData[i] = true;
      }
    }
  }
  if (DEBUG)
  {
    for (int sensor_iterator = 0; sensor_iterator < NumOfSensors; sensor_iterator++)
    {
      Serial.print(" ");
      Serial.print(sensor[sensor_iterator]);
    }
  }
}
//---------------------------------------------------------------------------------------
void generateBinary()
{
  if (DEBUG)
    lcd.clear();

  sumation = 0;
  for (int cx = 0; cx < NumOfSensors; cx++)
  {
    if (sensor[cx] > threshold[cx])
    {
      x[cx] = 1;
    }
    else
    {
      x[cx] = 0;
    }
    sumation += x[cx];
  }
  sensorData = 0;
  for (int cxx = 0; cxx < NumOfSensors; cxx++)
  {
    sensorData = (sensorData << 1) | x[cxx];
    if (DEBUG)
    {
      lcd.setCursor(cxx + 4, 0);
      lcd.print(x[cxx]);
    }
  }
  shift_right(memory_length - 1);
  memory[0] = sensorData;

  if (DEBUG)
    lcd.clear();
}
//--------------------------------------------------------------------------------
void generateThreshold()
{
  for (int th = 0; th < 500; th++)
  {
    Forward(1, 100);
    readSensors();
    for (int sense = 0; sense < NumOfSensors; sense++)
    {
      if (sensor[sense] > t1[sense])
        t1[sense] = sensor[sense];
      if (sensor[sense] < t2[sense])
        t2[sense] = sensor[sense];
    }
  }
  for (int thr = 0; thr < NumOfSensors; thr++)
  {
    threshold[thr] = (t1[thr] + t2[thr]) / 2;
  }
}
//----------------------------------THE MAIN CALCULATION&EXECUTION------------------------------------------------
void deviation()
{
  if (sensorData == B00000000)
    Vul = Vul;
  else if (sensorData == B00011000)
    Vul = 0; //0001 1000
  else if (sensorData == B00111100)
    Vul = 0; //0011 1100
  //------------------------TWO SENSOR------------------------
  else if (sensorData == B00001100) //12
    Vul = 7.5;                      //0000 1100
  else if (sensorData == B00110000)
    Vul = -7.5; //0011 0000
  else if (sensorData == B00000110)
    Vul = 12.5; //0000 0110
  else if (sensorData == B01100000)
    Vul = -12.5; //0110 0000
  else if (sensorData == B00000011)
    Vul = 20; //0000 0011
  else if (sensorData == B11000000)
    Vul = -20; //1100 0000
  else if (sensorData == B00000001)
    Vul = 25; //0000 0001
  else if (sensorData == B10000000)
    Vul = -25; //1000 0000
  //-----------------------------three sensor---------------
  else if (sensorData == B00011100)
    Vul = 4; //0001 1100
  else if (sensorData == B00111000)
    Vul = -4; //0011 1000
  else if (sensorData == B00001110)
    Vul = 10; //0000 1110
  else if (sensorData == B01110000)
    Vul = -10; //0111 0000
  else if (sensorData == B00000111)
    Vul = 16; //0000 0111
  else if (sensorData == B11100000)
    Vul = -16; //1110 0000
  //----------------------four sensor-------------------------
  else if (sensorData == B00011110)
    Vul = 7.5; //0001 1110
  else if (sensorData == B01111000)
    Vul = -7.5; //0111 1000
  else if (sensorData == B00001111)
    Vul = -12.5; //0000 1111
  else if (sensorData == B11110000)
    Vul = -12.5; //1111 0000
  else
  {
    Vul = 0;
    sm = 0;
    for (int cnt = 0; cnt < NumOfSensors; cnt++)
    {
      Vul += (x[cnt] * 5 * ct[cnt]);
      sm += x[cnt];
    }
    Vul = Vul / sm;
  }
}
//-----------------------------------------------------------------------------------
void PIDval()
{
  Shomakolon = Shomakolon + Vul;
  Parthokko = Vul - AgerVul;
  PIDvalue = (Const1 * Vul) + (Const3 * Parthokko) + (Shomakolon * Const2);
  AgerVul = Vul;
  if (DEBUG)
  {
    Serial.print(" ");
    Serial.print(Vul);
  }
}
//------------------------------------------------------------------------------------
void doura()
{
  if (Vul > 0)
  {
    RSpeed = motorspeed - PIDvalue;
    LSpeed = motorspeed;
  }
  else if (Vul < 0)
  {
    LSpeed = motorspeed + PIDvalue;
    RSpeed = motorspeed;
  }
  else
  {
    RSpeed = motorspeed;
    LSpeed = motorspeed;
  }

  if (RSpeed < 5)
    RSpeed = 5;
  if (LSpeed < 5)
    LSpeed = 5;

  analogWrite(pwR, RSpeed);
  analogWrite(pwL, LSpeed);
  digitalWrite(rMin1, HIGH);
  digitalWrite(lMin1, HIGH);
  digitalWrite(rMin2, LOW);
  digitalWrite(lMin2, LOW);
}
//--------------------------------Driving Functions--------------------------------------
void Forward(double del, int vel)
{
  analogWrite(pwR, vel);
  analogWrite(pwL, vel);
  digitalWrite(rMin1, HIGH);
  digitalWrite(rMin2, LOW);
  digitalWrite(lMin1, HIGH);
  digitalWrite(lMin2, LOW);
  delay(del);
}
//--------------------------------------------------------------------------------------
void Backward(double del, int vel)
{
  analogWrite(pwR, vel);
  analogWrite(pwL, vel);
  digitalWrite(rMin2, HIGH);
  digitalWrite(rMin1, LOW);
  digitalWrite(lMin2, HIGH);
  digitalWrite(lMin1, LOW);
  delay(del);
}
//--------------------------------------------------------------------------------------
void Right(double del, int vel)
{
  analogWrite(pwR, vel);
  analogWrite(pwL, vel);
  digitalWrite(rMin1, LOW);
  digitalWrite(lMin1, HIGH);
  digitalWrite(rMin2, HIGH);
  digitalWrite(lMin2, LOW);
  delay(del);
}
//--------------------------------------------------------------------------------------
void Left(double del, int vel)
{
  analogWrite(pwR, vel);
  analogWrite(pwL, vel);
  digitalWrite(rMin1, HIGH);
  digitalWrite(lMin1, LOW);
  digitalWrite(rMin2, LOW);
  digitalWrite(lMin2, HIGH);
  delay(del);
}
//---------------------------------------------------------------------------------------
void Stop(double del)
{
  analogWrite(pwR, 0);
  analogWrite(pwL, 0);
  digitalWrite(rMin1, LOW);
  digitalWrite(lMin1, LOW);
  digitalWrite(rMin2, LOW);
  digitalWrite(lMin2, LOW);
  delay(del);
}
//---------------------------------Breaking functions--------------------------------------
void BreakF()
{
  Stop(20);
  Backward(30, 240);
  Stop(20);
  Backward(30, 160);
  Stop(10);
}
//-----------------------------------------------------------------------------------------
void BreakL()
{
  Stop(20);
  Right(30, 230);
  Stop(20);
  Right(40, 200);
  Stop(10);
}
//-----------------------------------------------------------------------------------------
void BreakR()
{
  Stop(20);
  Left(30, 230);
  Stop(20);
  Left(40, 200);
  Stop(10);
}
//----------------------------------------------------------------------------------------
void Tleft()
{
  BreakF();
  while (1)
  {
    Left(5, 160);
    readSensors();
    generateBinary();
    if (x[1] == 1 || x[2] == 1)
    {
      BreakL();
      break;
    }
  }
}
//----------------------------------------------------------------------------------------
void Tright()
{
  BreakF();
  while (1)
  {
    Right(5, 160);
    readSensors();
    generateBinary();
    if (x[6] == 1 || x[5] == 1)
    {
      BreakR();
      break;
    }
  }
}
//----------------------------------Searching For Object------------------------------------
double search()
{
  double duration = 0.00;
  double CM = 0.00;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 1700);
  CM = (duration / 58.82);
  if (CM > 10 && CM < 25)
    pick_object();
  return CM;
}
//-------------------------------------------------------------------------------------------
void pick_object()
{
  BreakF();
  while (search() > 5)
  {
    Forward(10, 50);
  }
  BreakF();
  for (int servo_angle = 0; servo_angle < 90; servo_angle++)
  {
    servo1.write(servo_angle);
    servo2.write(servo_angle);
    delay(10);
  }
}
//--------------------------------------------------------------------------------------------
void release_object()
{
  BreakF();
  for (int servo_angle = 90; servo_angle > 0; servo_angle--)
  {
    servo1.write(servo_angle);
    servo2.write(servo_angle);
    delay(10);
  }
  while (search() < 8)
  {
    Backward(10, 50);
  }
}
//------------------------------------Case Detection-------------------------------------------
void detection()
{
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  for (int detect = 0; detect < 200; detect++)
  {
    readSensors();
    generateBinary();
    deviation();
    PIDval();
    doura();
  }
  unsigned int memory_value = 0;
  for (int memory_iterator = 0; memory_iterator < memory_length; memory_iterator++)
  {
    memory_value = memory_value + memory[memory_iterator];
  }
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  /*
  *Desicion making code will go here -> depends on on field values
  * 
  */
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(memory_value);
  while (true)
  {
    Stop(10);
    if (digitalRead(btn4) == LOW)
      break;
  }
}