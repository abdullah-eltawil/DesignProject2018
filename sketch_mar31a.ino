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

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;
boolean turnSwitch = LOW;
boolean isDistanceOk, isWallAhead, isParallel, isOvershoot;

//Port pin constants
const int ci_UltrasonicF_Ping = 4;   //input plug (trig)
const int ci_UltrasonicF_Data = 5;   //output plug (echo)
const int ci_UltrasonicR_Ping = 6;   //input plug (trig)
const int ci_UltrasonicR_Data = 7;   //output plug (echo)

const int ci_UltrasonicT_Echo = 2; //echo
const int ci_UltrasonicT_Trig = 3; //trig

long durationFront, durationBack, durationT;
double distanceFront, distanceBack, distanceT, avgDistance;

const int ci_Mode_Button = 10;

const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;

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
const int ci_distance_from_wall = 20;
int state = 0 ;


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

void setup() {
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
  bt_Motors_Enabled = true; //digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate motor speeds to drive straight.
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      { Serial.println(ui_Robot_State_Index);
        //readUltrasonicT();
        //readUltrasonic();
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);

        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);

        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;
        //  Serial.println("0");
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {

#ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);
#endif
          Serial.println(ui_Robot_State_Index);
          goStraight();

          //keepDistance();

          readUltrasonicT();
          if (isWallAhead)
          {
            ui_Robot_State_Index = 2;
            break;
          }

          readUltrasonic();
          if (isDistanceOk)
          {
            stayParallel();
          }

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
        // Serial.println("1");
        break;
      }


    case 2:
      {
        Serial.println(ui_Robot_State_Index);
        //readUltrasonicT();
        ui_Left_Motor_Speed = 1200;
        ui_Right_Motor_Speed = 1800;

        servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
        servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);

        readUltrasonic();

        if (isParallel)
        {
          ui_Robot_State_Index = 1;

          Serial.print ("Parallel: ");
          Serial.println(isParallel);
          Serial.print ("Overshoot: ");
          Serial.println(isOvershoot);

        }

        //Serial.println("2");
      }

    case 3:    //Calibrate motor straightness after 3 seconds.
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();

            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);

            l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (l_Left_Motor_Position > l_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
              ui_Left_Motor_Offset = 0;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Left = ");
            Serial.print(ui_Left_Motor_Offset);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Motor_Offset);
#endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
          ui_Mode_Indicator_Index = 2;
        }

        break;
      }
  }
  //Serial.println(ui_Robot_State_Index);
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

  delay(10);

  digitalWrite (ci_UltrasonicR_Data, LOW);
  delayMicroseconds(2);

  digitalWrite (ci_UltrasonicR_Data, HIGH);
  delayMicroseconds (10);
  digitalWrite (ci_UltrasonicR_Data, LOW);

  durationBack = pulseIn(ci_UltrasonicR_Ping, HIGH);
  distanceBack = durationBack * 0.034 / 2;

  avgDistance = (distanceFront + distanceBack) / 2;
  isDistanceOk = abs(avgDistance - ci_distance_from_wall) <= 2;
  isParallel = abs(distanceFront - distanceBack) < 1;
  isOvershoot = (distanceFront - distanceBack) > 1;
}

/*if (((distanceFront+distanceBack/2) <= ci_distance_from_wall + 2) && ((distanceFront+distanceBack/2) >= ci_distance_from_wall - 2))
  {
  if ((distanceFront - distanceBack) > 0.5)
    return 1;

  else if (distanceBack - distanceFront > 0.5)
    return 2;

  else if (ui_Robot_State_Index == 2)
    return 4;

  else
    return 3;
  }

  else if ((distanceFront+distanceBack/2) < ci_distance_from_wall - 2)
  {
  if (distanceBack - distanceFront > 0.5)
    return 2;

  else if ((distanceFront - distanceBack) > 0.5)
    return 3;

  else if (ui_Robot_State_Index == 2)
    return 4;

  else
    return 2;
  }

  else    //too far away
  {
  if ((distanceFront - distanceBack) > 0.5)
    return 1;

  else if (distanceBack - distanceFront > 0.5)
    return 3;

  else if (ui_Robot_State_Index == 2)
    return 4;

  else
    return 1;
  }

  }*/

int readUltrasonicT ()
{
  delay(10);
  digitalWrite (ci_UltrasonicT_Trig, LOW);//clear output
  delayMicroseconds(2);

  digitalWrite (ci_UltrasonicT_Trig, HIGH);//send 8 cycle sonic burst for 10 microseconds
  delayMicroseconds (10);
  digitalWrite (ci_UltrasonicT_Trig, LOW);

  durationT = pulseIn(ci_UltrasonicT_Echo, HIGH); //times the echo input and calculates distance
  distanceT = durationT * 0.034 / 2; //0.034 speed of sound, 2 for signal reflection
  //Serial.println (distanceT);

  isWallAhead = distanceT <= 30;
}

void turnRight()
{
  ui_Left_Motor_Speed = 2100;
  ui_Right_Motor_Speed = 1550;

  //servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  //servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void turnLeft ()
{
  ui_Left_Motor_Speed = 1550;
  ui_Right_Motor_Speed = 2100;

  //servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  //servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void goStraight()
{
  //Serial.println("going straight dude");
  ui_Left_Motor_Speed = 1800;
  ui_Right_Motor_Speed = 1800;

  //servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  //servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void stayParallel ()
{
  readUltrasonic();

  if ((distanceFront - distanceBack) > 1)
    turnRight();

  else if ((distanceBack - distanceFront) > 1)
    turnLeft();

  else
    goStraight();
}

void keepDistance()
{
  readUltrasonic();

  if (abs(avgDistance - ci_distance_from_wall) <= 1) //just right
  {
    goStraight();
    Serial.println("going straight dude");
  }

  else if (avgDistance > (ci_distance_from_wall + 1)) //too far
    turnRight();

  else //too close
    turnLeft();
}

