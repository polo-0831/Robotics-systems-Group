#include "Motors.h"
#include "encoders.h"'
#include "PID.h"

Motors_c motors; 
PID_c left_pid; 
PID_c right_pid;

#define SPEED_EST_MS 10 // estimate speed every 10ms
#define TEST_MS 2000
unsigned long speed_est_ts; // timestamp for speed estimation
bool isTurning = false; 
unsigned long test_ts;
unsigned long pid_update_TIME;

long last_e0;
long last_e1;
long previTime = 0; 

float speed_e0;
float speed_e1;
float St0;
float St1;
float previSt0;
float previSt1;
float a = 0.1; // parameter of low pass filter
float demand = 10; // velocity demand

void setup() {
  Serial.begin(9600);
  setupEncoder0();
  setupEncoder1();
  last_e0 = count_e0;
  last_e1 = count_e1;
  speed_e0 = 0.0;
  speed_e1 = 0.0;
  St0 = 0.0;
  St1 = 0.0;
  previSt0 = 0.0;
  previSt1 = 0.0;
  left_pid.initialise( 5.5, 0.05, 0.5 ); //pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
  right_pid.initialise( 5.5, 0.05, 0.5 ); //pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
  test_ts = millis(); // Prepare our testing timestamp
  left_pid.reset();
  right_pid.reset();
}

void loop() {

  
  pid_update_TIME = 0;
  unsigned long currenTime = millis();
  if( currenTime - previTime > 10 ) {
    long dt = currenTime - previTime;
    if (dt == 0) {
      return;
    }
    long count_difference0 = count_e0 - last_e0; 
    long count_difference1 = count_e1 -last_e1;
    last_e0 = count_e0;
    last_e1 = count_e1;
    speed_e0 = (float)count_difference0 / (float)dt;
    speed_e1 = (float)count_difference1 / (float)dt;
    St0 = (float)(a * speed_e0) + (float)((1.0 - a) * previSt0);
    St1 = (float)(a * speed_e1) + (float)((1.0 - a) * previSt1);
    previTime = currenTime;
    previSt0 = St0;
    previSt1 = St1;
    //Serial.print( speed_e0 * 60000 / 358.3 , 4 ); // chaging speed unit to rpm
    //Serial.print( "," );
    //Serial.println( St * 60000 / 358.3 , 4 ); //rpm // chaging speed unit to rpm
  }



//   if( millis() - test_ts > TEST_MS ) { 
//    test_ts = millis();
//    demand = demand * -1.0;
//  } // changing damand to positive or netative to test the performance PID controller




    if(millis() - pid_update_TIME > 30){
      float l_pwm = left_pid.update( demand, St1*10 ); 
      float r_pwm = right_pid.update( demand, St0*10 );
      motors.setPWM( l_pwm, r_pwm );
      pid_update_TIME = millis();
      Serial.print( demand );
      Serial.print( "," );
      Serial.print( St0*10 );
      Serial.print( "," );
      Serial.print( St1*10 );
      Serial.print( "\n" );
    }
}
