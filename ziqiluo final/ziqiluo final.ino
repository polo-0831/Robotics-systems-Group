#include "encoders.h"
#include "Motors.h"      // Labsheet 1
#include "PID.h"         // Labsheet 1 - Advanced
#include "LineSensors.h" // Labsheet 2
#include "Kinematics.h"  // Labsheet 4
#include "Magnetometer.h"
#include <PololuOLED.h>
#include <PololuHD44780.h>
#include <math.h>
#include <Wire.h>
#include <LIS3MDL.h>

#define BUZZER_PIN 6
#define PI 3.1415926
#define X_SCALLING 0.6
#define Y_SCALLING 0.44
#define MAGNET_THRESHOLD 4.5
#define DISTANCE_THRESHOLD 25
#define HOME_X 0 - 2.0
#define HOME_Y 0

PololuSH1106 display(1, 30, 0, 17, 13);  // oled
PololuHD44780 lcd(0, 1, 14, 17, 13, 30); // lcd
uint8_t savedUDIEN;
uint8_t savedUENUM;
uint8_t savedUEIENX0;

Motors_c motors;
LineSensors_c line_sensors;
Kinematics_c pose;
PID_c left_pid;
LIS3MDL mag;
Magnetometer_c magnetometer;

#define LEFT 1
#define RIGHT 2

// FSM state flag
#define CALIBRATE 0
#define SETOFF 1
#define TASK 2
#define CHECK 3
#define GOINGXY 4
#define STOP 5
#define HOME 6
#define GOINGMAG 7
#define DEBUG 8
#define TURN_ANGLE_AND_MOVE 9
int state = CALIBRATE;
int back_state = TASK; // use for state CHECK, point out what state should go back after check function

// check state. used for what thing I should check
#define MOVE_CHECK 0
#define TURN_CHECK 1
#define TURN_ANGLE_CHECK 2
#define GOXY_CHECK 3
#define TURN_ANGLE_AND_MOVE_CHECK 4
int check_flag = MOVE_CHECK;

// check flag. 作为退出check状态的标志
bool reset_coord_flag = true;
bool beep_flag = false;
bool move_flag = false;
bool turn_flag = false;
bool turnAngle_flag = false;
bool goxy_flag = false;

// encoder
long last_e0;
float speed_rads_e0;

// 一些时间变量的定义
unsigned long elapsed_time;   // for timer code
unsigned long searching_time; // counting searching time
#define SEARCHING_TIME 120

// 某些check用时间作为退出check的标志
unsigned long beep_stop_time = 0;
unsigned long move_stop_time = 0;
unsigned long turn_stop_time = 0;
bool goXY_changeCheckError_flag = false; // 随着时间增长会有pose x y的误差，跑长距离的误差越大，用这个flag来判定然后硬性修正
int calcGoxyStopTime_flag = 0;           //

float l_demand_speed = 30.0;

float target_angle = 0.0;         // target angle
int target_turn_direction = LEFT; // target direction for turnAngle(LEFT RIGHT)
float target_x, target_y;         // target cord
float goXY_distance;
float new_origin_x, new_origin_y; // right map origin
float mag_x, mag_y;               // magnet cord
int mode = 1;                     // for test
float debug1, debug2, debug3, debug4;
int call_once_flag = 0;
void setup()
{
    setupEncoder0();
    setupEncoder1();
    Wire.begin();
    // PID
    left_pid.initialise(0.0, 0.0, 0.0);
    left_pid.reset();

    // encoder
    last_e0 = count_e0;
    speed_rads_e0 = 0.0;

    pinMode(BUZZER_PIN, OUTPUT);

    motors.initialise();
    line_sensors.initialiseForADC();
    pose.initialise(0, 0, 0);

    Serial.begin(9600);
    delay(2000);
    Serial.println(" *** READY *** ");

    magnetometer.initialise();
    mag.enableDefault();

    elapsed_time = millis();
}

void loop()
{
    pose.update();
    mag.read();
    line_sensors.calcReadingADC();
    magnetometer.calcReadingMeg();

    disableUSB();
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print(state, 1);
    lcd.gotoXY(2, 0);
    lcd.print(check_flag, 1);
    lcd.gotoXY(4, 0);
    lcd.print(((millis() - searching_time) / 1000), 1);
    lcd.gotoXY(0, 1);
    lcd.print(goXY_distance, 2);
    enableUSB();

    switch (state)
    {
    case CHECK:
        if (check_flag == MOVE_CHECK)
        {
            move_flag = checkMove();
            if (move_flag == false)
                state = back_state;
        }
        if (check_flag == TURN_CHECK)
        {
            turn_flag = checkTurn();
            if (turn_flag == false)
                state = back_state;
        }
        if (check_flag == TURN_ANGLE_CHECK)
        {
            turnAngle_flag = checkTurnAngle();
            if (turnAngle_flag == false)
                state = back_state;
        }
        if (check_flag == GOXY_CHECK)
        {
            turnAngle_flag = checkTurnAngle();
            if (turnAngle_flag == false)
            {
                motors.setPWM(20, 22);
                goxy_flag = checkGoXY();
                if (goxy_flag == false)
                {
                    state = back_state;
                    if (goXY_distance > 450)
                    {
                        motors.setPWM(17, 17);
                        delay(400);
                    }
                    else if (goXY_distance > 350 && goXY_distance <= 450)
                    {
                        motors.setPWM(17, 17);
                        delay(250);
                    }
                    else
                    {
                    }
                    if (reset_coord_flag == true)
                    {
                        new_origin_x = pose.x;
                        new_origin_y = pose.y;
                        reset_coord_flag = false;
                    }
                }
            }
            else
            {
                setTurnAngle(target_angle);
            }
        }
        if (check_flag == TURN_ANGLE_AND_MOVE_CHECK)
        {
            turnAngle_flag = checkTurnAngle();
            if (turnAngle_flag == false)
            {
                setMove(20, 20, 500, back_state);
            }
        }
        break;
    case DEBUG:
        motors.setPWM(0, 0);
        break;
    case CALIBRATE:
        motors.setPWM(17, -17);
        line_sensors.CalibratedADC();
        magnetometer.CalibratedMag();

        if ((millis() - elapsed_time) > 4000)
        {
            searching_time = millis();
            state = SETOFF;
        }
        break;
    case SETOFF:
        motors.setPWM(0, 0);
        delay(500);
        // turn right 90 dgree (270 dgree), set off to right map
        target_x = 0;
        target_y = -4.3; // go to right map coord (1,1)
        setGoXY(target_x, target_y, TASK);
        break;
    case TURN_ANGLE_AND_MOVE:
        setMove(20, 20, 500, back_state);
        break;
    case STOP:
        motors.setPWM(0, 0);
        break;
    case HOME:
        motors.setPWM(0, 0);
        delay(500); // stop for a moment while detecting magnet

        target_x = HOME_X;
        target_y = HOME_Y;                           // go back home
        mag_x = (pose.x / (100 * X_SCALLING)) - 1.0; // record currunt coord
        mag_y = (pose.y / (100 * Y_SCALLING)) - 2.0;

        setGoXY(target_x, target_y, GOINGMAG);
        break;
    case GOINGMAG:
        target_x = mag_x;
        target_y = mag_y;
        setGoXY(target_x, target_y, STOP);
        break;
    case TASK:
        float l_pwm, r_pwm;
        float l_random_speed = getRand(0, 7);
        float r_random_speed = getRand(0, 7);

        if (millis() - elapsed_time > 1000)
        {
            elapsed_time = millis();
            l_pwm = 16 + l_random_speed;
            r_pwm = 16 + r_random_speed;
        }

        if (((millis() - searching_time) / 1000) >= SEARCHING_TIME) // restrict searching time
        {
            setBeep(2000);
            state = STOP;
            break;
        }

        if (line_sensors.DetectBlackLine() && turn_flag == false) // detecte line
            setTurn(line_sensors.DetectBlackLine(), 18, 500, TASK);
        else if ((pose.y > new_origin_y - 50) && turnAngle_flag == false) // restrict on right map
            setTurnAngleAndMove(-PI / 2, TASK);
        else if (magnetometer.meg_distance > MAGNET_THRESHOLD) // detect magnet
        {
            setMove(0, 17, 1000, HOME);
            setBeep(500);
            // mag_x = pose.x / (100 * X_SCALLING);
            // mag_y = pose.y / (100 * Y_SCALLING);
            // back_state = GOINGMAG;
            // state = HOME;
        }
        else // random searching
            motors.setPWM(l_pwm, r_pwm);
        break;
    }
    checkBeep();
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

void setMove(float l_pwm, float r_pwm, unsigned long duration_ms, int bk_state)
{
    move_stop_time = millis() + duration_ms;
    motors.setPWM(l_pwm, r_pwm);
    state = CHECK;
    check_flag = MOVE_CHECK;
    back_state = bk_state;
}
bool checkMove()
{
    if (millis() < move_stop_time)
        return true;
    else
    {
        return false;
    }
}

void setTurn(int direction, float speed, unsigned long duration_ms, int bk_state)
{
    turn_stop_time = millis() + duration_ms;
    if (direction == LEFT)
        motors.setPWM(-speed, speed);
    else if (direction == RIGHT)
        motors.setPWM(speed, -speed);
    state = CHECK;
    check_flag = TURN_CHECK;
    back_state = bk_state;
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
void setTurnAngleAndMove(float target, int bk_state)
{
    int direction = LEFT;

    float angle_diff = target - pose.theta;
    target_angle = target;
    if (angle_diff > 0)
        target_turn_direction = RIGHT;
    else
        target_turn_direction = LEFT;

    state = CHECK;
    check_flag = TURN_ANGLE_AND_MOVE_CHECK;
    back_state = bk_state;
}

void setGoXY(float tar_x, float tar_y, int bk_state)
{
    float dgree;
    target_x = tar_x; // 0.4 is wheel speed diff error
    target_y = tar_y;
    tar_x = tar_x * 100 * X_SCALLING;
    tar_y = tar_y * 100 * Y_SCALLING;
    float diff_x = tar_x - pose.x;
    float diff_y = tar_y - pose.y;

    setTurnAngle(atan2(diff_y, diff_x));
    state = CHECK;
    check_flag = GOXY_CHECK;
    back_state = bk_state;
}
bool checkGoXY()
{
    float thres;
    float diff_x = pose.x - 100 * X_SCALLING * target_x;
    float diff_y = pose.y - 100 * Y_SCALLING * target_y;
    goXY_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

    debug1 = diff_x;
    debug2 = diff_y;

    if (goXY_changeCheckError_flag == false)
    {
        if (goXY_distance > 450)
            thres = DISTANCE_THRESHOLD + 30;
        else if (goXY_distance > 350 && goXY_distance <= 450)
            thres = DISTANCE_THRESHOLD + 20;
        else
            thres = DISTANCE_THRESHOLD;
        goXY_changeCheckError_flag = true;
    }

    if (goXY_distance < thres)
    {
        goXY_changeCheckError_flag = false;
        return false;
    }
    else
        return true;
}

int getRand(int min, int max)
{
    return (rand() % (max - min + 1)) + min;
}

void disableUSB()
{
    savedUDIEN = UDIEN;
    UDIEN = 0;
    savedUENUM = UENUM;
    UENUM = 0;
    savedUEIENX0 = UEIENX;
    UEIENX = 0;
}
void enableUSB()
{
    UENUM = 0;
    UEIENX = savedUEIENX0;
    UENUM = savedUENUM;
    UDIEN = savedUDIEN;
}
