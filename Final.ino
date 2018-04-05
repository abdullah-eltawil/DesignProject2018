/*
  Doolz & Joy
*/

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_StopperMotor1;
Servo servo_StopperMotor2;
Servo servo_ArmMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_MOTOR_CALIBRATION
int pyramidCounter = 0;
boolean bt_Motors_Enabled = true;

//Port pin constants
const int ci_UltrasonicF_Ping = 4;   //output plug (trig)
const int ci_UltrasonicF_Data = 5;   //input plug (echo)
const int ci_UltrasonicR_Ping = 6;   //output plug (trig)
const int ci_UltrasonicR_Data = 7;   //input plug (echo)

const int ci_UltrasonicT_Echo = 2; //echo
const int ci_UltrasonicT_Trig = 3; //trig

long durationFront, durationBack, durationT;
double distanceFront, distanceBack, distanceT;

const int ci_Mode_Button = 10;

const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Stopper_Motor1 = A2;
const int ci_Stopper_Motor2 = A3;
const int ci_Arm_Motor =13;

const int ci_Motor_Enable_Switch = 12;

const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow


//Constants

//EEPROM addresses
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;

const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//Variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;

unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

const int ci_distance_from_wall = 20;
const int ci_distance_ahead = 14;

boolean isDistanceOk, isWallAhead, isParallel, isPyramidAhead;

int parallelCounter(0), turnCounter(0);
int interruptPin = 2;

void setup()
{
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up ultrasonic
  pinMode (ci_UltrasonicF_Data, OUTPUT);
  pinMode (ci_UltrasonicR_Data, OUTPUT);
  pinMode (ci_UltrasonicF_Ping, INPUT);
  pinMode (ci_UltrasonicR_Ping, INPUT);

  pinMode (ci_UltrasonicT_Trig, OUTPUT);
  pinMode (ci_UltrasonicT_Echo, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);

  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  pinMode (interruptPin, LOW);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), pyramidISR, HIGH);

  // set up motor enable switch
  //pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (digitalRead(ci_Mode_Button))
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = true;

  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {

        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);

        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);

        ui_Mode_Indicator_Index = 0;

        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          

          readUltrasonicT();
          if (isWallAhead)
          {
            goReverse();
            delay(800);
            ui_Robot_State_Index = 2;
            turnCounter++;
            break;
          }

          if(turnCounter >=4)
            ui_Robot_State_Index = 3;

          stayParallel();

          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
          
#ifdef DEBUG_MOTORS
          Serial.print("Motors enabled: ");
          Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif
          ui_Mode_Indicator_Index = 1;
        }
        break;
      }


    case 2:
      {
        turnACW();

        readUltrasonic();


        if (parallelCounter >= 45)
        {
          ui_Robot_State_Index = 1;
          parallelCounter = 0;
          break;
        }
        //Serial.println(parallelCounter);

      }

    case 3: //search for pyramid
      {
        findPyramid();
        
      if(digitalRead(A5)){
        ui_Robot_State_Index = 1;
      }
      else
        ui_Robot_State_Index = 3;
        
       break;
      
  }

      
  }
}
void readUltrasonic()
{
  delay(10);

  digitalWrite (ci_UltrasonicF_Data, LOW);//clear output
  delayMicroseconds(2);

  digitalWrite (ci_UltrasonicF_Data, HIGH);//send 8 cycle sonic burst for 10 microseconds
  delayMicroseconds (10);
  digitalWrite (ci_UltrasonicF_Data, LOW);

  durationFront = pulseIn(ci_UltrasonicF_Ping, HIGH); //times the echo input and calculates distance
  distanceFront = durationFront * 0.034 / 2; //0.034 speed of sound, 2 for signal reflection

  delay(5);

  digitalWrite (ci_UltrasonicR_Data, LOW);
  delayMicroseconds(2);

  digitalWrite (ci_UltrasonicR_Data, HIGH);
  delayMicroseconds (10);
  digitalWrite (ci_UltrasonicR_Data, LOW);

  durationBack = pulseIn(ci_UltrasonicR_Ping, HIGH);
  distanceBack = durationBack * 0.034 / 2;

  isDistanceOk = abs(distanceFront - ci_distance_from_wall) <= 3;
  isParallel = abs(distanceFront - distanceBack) < .5;
}

int readUltrasonicT()
{
  delay(5);
  digitalWrite (ci_UltrasonicT_Trig, LOW);//clear output
  delayMicroseconds(2);

  digitalWrite (ci_UltrasonicT_Trig, HIGH);//send 8 cycle sonic burst for 10 microseconds
  delayMicroseconds (10);
  digitalWrite (ci_UltrasonicT_Trig, LOW);

  durationT = pulseIn(ci_UltrasonicT_Echo, HIGH); //times the echo input and calculates distance
  distanceT = durationT * 0.034 / 2; //0.034 speed of sound, 2 for signal reflection

  isWallAhead = distanceT <= ci_distance_ahead;
  isPyramidAhead = distanceT <= 10;

  //Serial.println(distanceT);
}

void veerRight()
{
  ui_Left_Motor_Speed = 1975;
  ui_Right_Motor_Speed = 1600;
  Serial.println("Veering Right");
}
void veerLeft()
{
  ui_Left_Motor_Speed = 1600;
  ui_Right_Motor_Speed = 2025;
  Serial.println("Veering Left");
}

void goStraight()
{
  ui_Left_Motor_Speed = 1800;
  ui_Right_Motor_Speed = 1800;
  Serial.println("Going straight");
}

void goReverse()
{
  ui_Left_Motor_Speed = 1200;
  ui_Right_Motor_Speed = 1200;

  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void stopMotors()
{
  servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
  servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
}

void turnACW ()
{
  ui_Left_Motor_Speed = 1550;
  ui_Right_Motor_Speed = 2100;

  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);

  Serial.println("ACW");

  if(isParallel)
    parallelCounter++;

  Serial.print("Parallel Counter: ");
  Serial.println(parallelCounter);

  
}

void stayParallel()
{
  readUltrasonic();

  if ((distanceFront - distanceBack) > 0.5) //facing away from wall
    veerRight();

  else if ((distanceBack - distanceFront) > 0.5) //facing toward wall
    veerLeft();

  else //parallel to wall
    goStraight();
}

void findPyramid(){
    ui_Left_Motor_Speed = 1200;
    ui_Right_Motor_Speed = 1800;
  
    servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
    servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
    pyramidCounter++;
    Serial.println(pyramidCounter);
}
