#include "Motors.h"
#include "encoders.h"
#include "PID.h"
#include "LineSensors.h"
#include "Kinematics.h"
#include "Magnetometer.h"

// 定义结构体
Motors_c motors;
LineSensors_c line_sensors;
Kinematics_c pose;
LIS3MDL mag;
Magnetometer_c magnetometer;
PID_c left_pid;
PID_c right_pid;

// 宏定义
#define BUZZER_PIN 6
#define LEFT 1  // 左转
#define RIGHT 2 // 右转
#define GOXY_THRESHOLD 40

// FSM state
#define STATE_CALIBRATE 0
#define STATE_TASK 2
#define STATE_CHECK 3
#define STATE_STOP 4
int state = STATE_TASK;

// CHECK部分的变量定义
#define TURN_CHECK 0
#define GOXY_CHECK 1
int check_flag = TURN_CHECK;

// 姿态控制需要的全局变量
float l_pwm, r_pwm;
float target_angle; // 用于checkTurnAngle内检验是否转到目标角度
int target_turn_direction;
float target_x, target_y; // 用于checkGOXY内检测是否到目标点

// 编码器相关变量
long last_e0;
long last_e1;
long Encoder_prevTime = 0;
float speed_e0 = 0.0;
float speed_e1 = 0.0;
float St0 = 0.0;
float St1 = 0.0;
float previSt0 = 0.0;
float previSt1 = 0.0;
float a = 0.1; // parameter of low pass filter
float demand;  // velocity demand

// 时间参数
unsigned long elapsed_time;

unsigned long beep_stop_time;
unsigned long turn_stop_time;

// debug调试所需要的变量
int mode = 1;

void setup()
{
  // 编码器初始化
  setupEncoder0();
  setupEncoder1();
  last_e0 = count_e0;
  last_e1 = count_e1;
  Wire.begin();
  // PID
  left_pid.initialise(5.5, 0.05, 0.5);  // pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
  right_pid.initialise(5.5, 0.05, 0.5); // pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
  left_pid.reset();
  right_pid.reset();
  // BUZZER
  pinMode(BUZZER_PIN, OUTPUT);

  // Motor LineSensors Pose
  motors.initialise();
  // line_sensors.initialiseForADC();
  pose.initialise(0, 0, 0);
  // Mag
  // magnetometer.initialise();
  // mag.enableDefault();
  // Serial
  Serial.begin(9600);
  delay(2000);
  Serial.println(" *** READY *** ");

  // 初始化一些时间变量
  elapsed_time = millis();
}

void loop()
{
  // 传感器数据更新
  pose.update();
  // mag.read();
  // line_sensors.calcReadingADC();
  // magnetometer.calcReadingMeg();

  // 编码器计算
  calcEncoder();

  switch (state)
  {
  case STATE_CHECK:
    if (check_flag == TURN_CHECK)
    {
      if (checkTurn() == false)
      {
        state = STATE_TASK;
      }
    }
    if (check_flag == GOXY_CHECK)
    {
      if (checkTurnAngle() == false) // 转角度完成了
      {
        demand = 5;
        l_pwm = left_pid.update(demand, St1 * 10);
        r_pwm = right_pid.update(demand, St0 * 10);
        motors.setPWM(25, 25);    // 这里我先不加PID，我尝试了加上去，但参数不对走的很怪
        if (checkGoXY() == false) // 这个动作完成了
        {
          state = STATE_TASK;
        }
      }
      else // 未完成角度旋转或者going xy的同时偏离了目标角度，都会在这里修正
      {
        setTurnAngle(target_angle);
      }
    }

    break;
  case STATE_TASK:
    if (mode == 1)
    {
      target_x = 300;
      target_y = -300;
      setGoXY(target_x, target_y);
      mode = 2;
      setBeep(500);
    }
    else if (mode == 2)
    {
      target_x = 500;
      target_y = -100;
      setGoXY(target_x, target_y);
      mode = 3;
      setBeep(500);
    }
    else if (mode == 3)
    {
      target_x = 0;
      target_y = 0;

      mode = 1;
      setGoXY(target_x, target_y);
      setBeep(500);
    }

    if ((millis() - elapsed_time) / 1000 > 60)
    {
      state = STATE_STOP;
    }
    break;
  case STATE_STOP:
    motors.setPWM(0, 0);
    break;
  }
  checkBeep(); // 蜂鸣器的set和check跟我们的FSM无关，所以把check放外面了

  Serial.print(demand);
  Serial.print(",");
  Serial.print(St0 * 10);
  Serial.print(",");
  Serial.print(St1 * 10);
  Serial.print(",");
  Serial.print(l_pwm);
  Serial.print(",");
  Serial.println(r_pwm);
}

void setBeep(unsigned long duration_ms)
{
  beep_stop_time = millis() + duration_ms;
  analogWrite(BUZZER_PIN, 150);
}
bool checkBeep()
{
  if (millis() < beep_stop_time)
    return true;
  else
  {
    analogWrite(BUZZER_PIN, 0);
    return false;
  }
}

void calcEncoder()
{
  if (millis() - Encoder_prevTime > 10)
  {
    long dt = millis() - Encoder_prevTime;
    if (dt == 0)
    {
      return;
    }
    long count_difference0 = count_e0 - last_e0;
    long count_difference1 = count_e1 - last_e1;
    last_e0 = count_e0;
    last_e1 = count_e1;
    speed_e0 = (float)count_difference0 / (float)dt;
    speed_e1 = (float)count_difference1 / (float)dt;
    St0 = (float)(a * speed_e0) + (float)((1.0 - a) * previSt0);
    St1 = (float)(a * speed_e1) + (float)((1.0 - a) * previSt1);
    Encoder_prevTime = millis();
    previSt0 = St0;
    previSt1 = St1;
    // Serial.print( speed_e0 * 60000 / 358.3 , 4 ); // chaging speed unit to rpm
    // Serial.print( "," );
    // Serial.println( St * 60000 / 358.3 , 4 ); //rpm // chaging speed unit to rpm
  }
}

void setTurn(int direction, float speed, unsigned long duration_ms)
{
  turn_stop_time = millis() + duration_ms;
  if (direction == LEFT)
    motors.setPWM(-speed, speed);
  else if (direction == RIGHT)
    motors.setPWM(speed, -speed);
  state = STATE_CHECK;
  check_flag = TURN_CHECK;
}
bool checkTurn()
{
  if (millis() < turn_stop_time)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void setTurnAngle(float target)
{
  float angle_diff = target - pose.theta;
  target_angle = target;
  if (angle_diff > 0)
    target_turn_direction = RIGHT;
  else
    target_turn_direction = LEFT;
}
bool checkTurnAngle()
{
  float turn_speed = 17;
  if (target_turn_direction == LEFT)
    motors.setPWM(turn_speed, -turn_speed);
  else if (target_turn_direction == RIGHT)
    motors.setPWM(-turn_speed, turn_speed);

  if (abs(pose.theta - target_angle) < 0.1)
    return false;
  else
    return true;
}

void setGoXY(float tar_x, float tar_y)
{
  float dgree;
  target_x = tar_x;
  target_y = tar_y;
  float diff_x = tar_x - pose.x;
  float diff_y = tar_y - pose.y;

  setTurnAngle(atan2(diff_y, diff_x));
  state = STATE_CHECK;
  check_flag = GOXY_CHECK;
}
bool checkGoXY()
{
  float thres;
  float diff_x = pose.x - target_x;
  float diff_y = pose.y - target_y;
  float goXY_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

  if (goXY_distance < GOXY_THRESHOLD)
  {
    return false;
  }
  else
    return true;
}