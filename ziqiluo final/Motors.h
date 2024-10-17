/************************************
,        .       .           .      ,
|        |       |           |     '|
|    ,-: |-. ,-. |-. ,-. ,-. |-     |
|    | | | | `-. | | |-' |-' |      |
`--' `-` `-' `-' ' ' `-' `-' `-'    '
*************************************/

#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

#define FWD LOW
#define BCK HIGH
#define MAX_PWM 40.0

class Motors_c
{

public:
  Motors_c()
  {
    // Leave empty. Ensure initialise() is called
    // instead.
  }

  void initialise()
  {
    pinMode(L_PWM, OUTPUT);
    pinMode(L_DIR, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(R_DIR, OUTPUT);

    digitalWrite(L_DIR, FWD);
    digitalWrite(R_DIR, FWD);

    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
  }

  void setPWM(float left_pwr, float right_pwr)
  {

    if (left_pwr < 0)
    {
      digitalWrite(L_DIR, BCK);
      left_pwr = abs(left_pwr);
    }
    else
    {
      digitalWrite(L_DIR, FWD);
    }

    if (right_pwr < 0)
    {
      digitalWrite(R_DIR, BCK);
      right_pwr = abs(right_pwr);
    }
    else
    {
      digitalWrite(R_DIR, FWD);
    }

    // maximum pwm protection
    left_pwr > MAX_PWM ? left_pwr = MAX_PWM : left_pwr;
    right_pwr > MAX_PWM ? right_pwr = MAX_PWM : right_pwr;
    // write
    analogWrite(L_PWM, left_pwr);
    analogWrite(R_PWM, right_pwr);

    return;
  }
};

#endif
