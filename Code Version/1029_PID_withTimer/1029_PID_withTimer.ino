#include "Motors.h"
#include "encoders.h"'
#include "PID.h"
#include "Kinematics.h"

Motors_c motors; 
PID_c left_pid; 
PID_c right_pid;

#define SPEED_EST_MS 10 // estimate speed every 10ms
#define TEST_MS 2000

//Timer
volatile boolean DEBUG_LED_STATE = false;

//PID
unsigned long speed_est_ts;
unsigned long test_ts;
unsigned long PID_update_previTime = 0;
float a = 0.5; // parameter of low pass filter
float demand = 0; // velocity demand
bool PID_Turning = false;

//Encoder
unsigned long Encoder_dt_previTime = 0; 
long last_e0;
long last_e1;
float speed_e0;
float speed_e1;
float St0;
float St1;
float previSt0;
float previSt1;

//Pose
unsigned long Pose_update_previTime = 0;
Kinematics_c pose;

//FSM
enum states { STATE_CALIBRATE, STATE_PUSH, STATE_STOP };
int state;

void setup() {
  Serial.begin(9600);
  //Encoder initialise
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

  //pid initialise
  left_pid.initialise( 5.5, 0.025, 0.5 ); //pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
  right_pid.initialise( 5.5, 0.025, 0.5 ); //pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
//  PID_Turning = true;
  test_ts = millis(); // Prepare our testing timestamp

  //pose initialise
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);

  //Timer initialise
  pinMode(13,OUTPUT );
  digitalWrite( 13, DEBUG_LED_STATE );
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");
  setupTimer3(); 

  // must put this at the end of setup
  delay(2000);
  left_pid.reset();
  right_pid.reset();
}

void loop() {
  switch (state) {
    case STATE_CALIBRATE:
      demand = 5;
      state = STATE_PUSH;
      PID_Turning = true;
      break;
    case STATE_PUSH:
      
      if (pose.x > 1000){
        state = STATE_STOP;
      }
      break;
    case STATE_STOP:
      PID_Turning = false;
//    left_pid.initialise( 0, 0, 0 );
//    right_pid.initialise( 0, 0, 0 );
      demand = 0;
      motors.setPWM(0, 0);
      break;
  }
  
//  pose_Update();
 
//  encoder_Cal();

//  if (PID_Turning){
//    pid();
//  }
    
}
void encoder_Cal(){

    long count_difference0 = count_e0 - last_e0; 
    long count_difference1 = count_e1 -last_e1;
    last_e0 = count_e0;
    last_e1 = count_e1;
    speed_e0 = (float)count_difference0 / 10.0;
    speed_e1 = (float)count_difference1 / 10.0;
    St0 = (float)(a * speed_e0) + (float)((1.0 - a) * previSt0);
    St1 = (float)(a * speed_e1) + (float)((1.0 - a) * previSt1);
    previSt0 = St0;
    previSt1 = St1;
    //Serial.print( speed_e0 * 60000 / 358.3 , 4 ); // chaging speed unit to rpm
    //Serial.print( "," );
    //Serial.println( St * 60000 / 358.3 , 4 ); //rpm // chaging speed unit to rpm
  
}

void pid(){

      float l_pwm = left_pid.update( demand, St1*10 ); 
      float r_pwm = right_pid.update( demand, St0*10 );
      motors.setPWM( l_pwm, r_pwm );
      Serial.print( demand );
      Serial.print( "," );
      Serial.print( St0*10 );
      Serial.print( "," );
      Serial.print( St1*10 );
      Serial.print( "\n" );
    
}

void setupTimer3() {
  
  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual.
  TCCR3B = TCCR3B | (1 << CS32);
  
  
  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  OCR3A = 625; //10ms
  
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();
  
}

int count = 0;
// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {
  
  count++;

  // call encoder
  encoder_Cal();

  // PID 
  if (count % 3 == 0){
    if (PID_Turning){
      pid();
      }
  }

  // position update
  if (count % 2 == 0){
    pose.update();
  }

  if (count >= 30000){
    count = 0;
  }
}
