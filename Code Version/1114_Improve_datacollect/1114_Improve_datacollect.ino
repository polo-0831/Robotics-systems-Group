#include "Motors.h"
#include "encoders.h"
#include "PID.h"
#include "Kinematics.h"

Motors_c motors;
PID_c left_pid;
PID_c right_pid;

#define SPEED_EST_MS 10 // estimate speed every 10ms
#define TEST_MS 2000
#define EMIT_PIN 11
#define BUZZER_PIN 6
#define BL_PIN 4
#define BR_PIN 5

// Data collection
#define MAX_RESULTS 100
float bump_data[MAX_RESULTS][2];
float AddSpeed_data[MAX_RESULTS][2];
bool StartCollectingData_flag = false;
int results_index = 0;

// Bump sensor
unsigned long L_MaxBumpSensorTime = 1;
unsigned long R_MaxBumpSensorTime = 1;
unsigned long L_MinBumpSensorTime = 9999;
unsigned long R_MinBumpSensorTime = 9999;
unsigned long BLElapsedTime;
unsigned long BRElapsedTime;
float leftBumpValue;
float rightBumpValue;
float LeftBump_Max;
float LeftBump_Min;
float RightBump_Max;
float RightBump_Min;
float LeftCollide_thres;
float RightCollide_thres;

// Timer
volatile boolean DEBUG_LED_STATE = false;

// PID
unsigned long speed_est_ts;
unsigned long test_ts;
unsigned long PID_update_previTime = 0;
float a = 0.5; // parameter of low pass filter
#define INIT_SPEED_DEMAND 5
float l_demand = 0; // velocity demand
float r_demand = 0;
bool PID_Turning = false;

// Encoder
unsigned long Encoder_dt_previTime = 0;
long last_e0;
long last_e1;
float speed_e0;
float speed_e1;
float St0;
float St1;
float previSt0;
float previSt1;

// Pose
unsigned long Pose_update_previTime = 0;
Kinematics_c pose;

// FSM
enum states
{
  STATE_CALIBRATE,
  STATE_PUSH,
  STATE_STOP
};
int state = STATE_CALIBRATE;

// Time var
unsigned long beep_stop_time;
unsigned long CalibrateTime;
unsigned long TaskTime;
int diff_count, sync_count;

// Control
float accumulate_speed = 0;

void setup()
{
  Serial.begin(9600);
  // Bump sensor
  pinMode(BL_PIN, INPUT);
  pinMode(BR_PIN, INPUT);
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);

  // Encoder initialise
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

  // pid initialise
  left_pid.initialise(5.5, 0.025, 0.5);  // pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
  right_pid.initialise(5.5, 0.025, 0.5); // pid initialise ( 13.0, 0.2, 0.0 ) respond would be faster but too strong
  //  PID_Turning = true;
  test_ts = millis(); // Prepare our testing timestamp

  // pose initialise
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);

  // Timer initialise
  pinMode(13, OUTPUT);
  digitalWrite(13, DEBUG_LED_STATE);
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");
  setupTimer3();

  // must put it at the end
  delay(2000);

  setBeep(200);
  //  delay(500);

  CalibrateTime = millis();
}

void loop()
{
  checkBeep();
  BLElapsedTime = MeasureBumpSensor(BL_PIN, L_MaxBumpSensorTime, L_MinBumpSensorTime);
  leftBumpValue = 1023 - map(BLElapsedTime, L_MinBumpSensorTime, L_MaxBumpSensorTime, 0, 1023); // Inverse Mapping

  BRElapsedTime = MeasureBumpSensor(BR_PIN, R_MaxBumpSensorTime, R_MinBumpSensorTime);
  rightBumpValue = 1023 - map(BRElapsedTime, R_MinBumpSensorTime, R_MaxBumpSensorTime, 0, 1023); // Mapping digital value to discrete value

  leftBumpValue < LeftBump_Min ? LeftBump_Min = leftBumpValue : LeftBump_Min;
  leftBumpValue > LeftBump_Max ? LeftBump_Max = leftBumpValue : LeftBump_Max;
  rightBumpValue < RightBump_Min ? RightBump_Min = rightBumpValue : RightBump_Min;
  rightBumpValue > RightBump_Max ? RightBump_Max = rightBumpValue : RightBump_Max;

  LeftCollide_thres = LeftBump_Min + (LeftBump_Max - LeftBump_Min) * 0.9;
  RightCollide_thres = RightBump_Min + (RightBump_Max - RightBump_Min) * 0.9;
  switch (state)
  {
  case STATE_CALIBRATE:

    if (millis() - CalibrateTime > 3000) // 3s for calibrate bump sensor
    {
      left_pid.reset();
      right_pid.reset();
      PID_Turning = true;
      state = STATE_PUSH;
      delay(10);
      TaskTime = millis();
    }
    break;
  case STATE_PUSH:
    StartCollectingData_flag = true;
    l_demand = INIT_SPEED_DEMAND + Bumpvalue2Speed(rightBumpValue) + accumulate_speed;
    r_demand = INIT_SPEED_DEMAND + Bumpvalue2Speed(leftBumpValue) + accumulate_speed;

    /******************* set accumulate_speed ******************/
    //    if (diff_count > 100) // 1s
    //    {
    //      accumulate_speed = 3;
    //      diff_count = 0;
    //    }

    if (abs(pose.theta) < M_PI / 15 && accumulate_speed != 0) // back to normal
    {
      accumulate_speed = 0;
      //      state = STATE_STOP;
    }
    /************************** end *****************************/

    if (millis() - TaskTime > 10000) // 10s box pushing Task
    {
      state = STATE_STOP;
    }
    break;
  case STATE_STOP:
    StartCollectingData_flag = false;
    PID_Turning = false;
    //    left_pid.initialise( 0, 0, 0 );
    //    right_pid.initialise( 0, 0, 0 );
    l_demand = 0;
    r_demand = 0;
    motors.setPWM(0, 0);

    int result;
    Serial.println("LeftBumpValue & RightBumpValue : \n");
    for (result = 0; result < MAX_RESULTS; result++)
    {
      Serial.print(bump_data[result][0]);
      Serial.print(",");
      Serial.println(bump_data[result][1]);
    }
    Serial.println("LeftAddSpeed & RightAddSpeed : \n");
    for (result = 0; result < MAX_RESULTS; result++)
    {
      Serial.print(AddSpeed_data[result][0]);
      Serial.print(",");
      Serial.println(AddSpeed_data[result][1]);
    }

    delay(5000);
    setBeep(200);

    break;
  }
}

float bumpValueMap(unsigned long ElapsedTime, unsigned long MinBumpSensorTime, unsigned long MaxBumpSensorTime, float MinValue, float MaxValue)
{
  float BumpValue = (float)(ElapsedTime * (MaxValue - MinValue) / (MinBumpSensorTime - MaxBumpSensorTime));
  return BumpValue;
}

float Bumpvalue2Speed(float bump_value)
{
  float diff_speed = -0.02 * bump_value + 18;
  if (diff_speed > 5)
    diff_speed = 5;
  return diff_speed;
}

void encoder_Cal()
{

  long count_difference0 = count_e0 - last_e0;
  long count_difference1 = count_e1 - last_e1;
  last_e0 = count_e0;
  last_e1 = count_e1;
  speed_e0 = (float)count_difference0 / 10.0;
  speed_e1 = (float)count_difference1 / 10.0;
  St0 = (float)(a * speed_e0) + (float)((1.0 - a) * previSt0);
  St1 = (float)(a * speed_e1) + (float)((1.0 - a) * previSt1);
  previSt0 = St0;
  previSt1 = St1;
  // Serial.print( speed_e0 * 60000 / 358.3 , 4 ); // chaging speed unit to rpm
  // Serial.print( "," );
  // Serial.println( St * 60000 / 358.3 , 4 ); //rpm // chaging speed unit to rpm
}
void pid()
{

  float l_pwm = left_pid.update(l_demand, St1 * 10);
  float r_pwm = right_pid.update(r_demand, St0 * 10);
  motors.setPWM(l_pwm, r_pwm);
  // Serial.print(demand);
  // Serial.print(",");
  // Serial.print(St0 * 10);
  // Serial.print(",");
  // Serial.print(St1 * 10);
  // Serial.print("\n");
}

void setupTimer3()
{

  // disable global interrupts
  cli();

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0; // set entire TCCR3A register to 0
  TCCR3B = 0; // set entire TCCR3B register to 0

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
  OCR3A = 625; // 10ms

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
ISR(TIMER3_COMPA_vect)
{

  count++;

  encoder_Cal();

  if (state == STATE_PUSH && abs(pose.theta) > 0)
  {
    if (pose.theta > 0)
    {
      if (accumulate_speed < 0)
        accumulate_speed = 0;
      accumulate_speed += 0.01;
      if (accumulate_speed > 2)
        accumulate_speed = 2;
    }
    else if (pose.theta < 0)
    {
      if (accumulate_speed > 0)
        accumulate_speed = 0;
      accumulate_speed -= 0.01;
      if (accumulate_speed < -2)
        accumulate_speed = -2;
    }
  }

  if (count % 3 == 0)
  {
    if (PID_Turning)
    {
      pid();
    }
  }

  if (count % 2 == 0)
  {
    pose.update();
  }

  if (count % 10 == 0) // 100ms
  {
    if (StartCollectingData_flag == true)
      if (results_index < MAX_RESULTS)
      {

        bump_data[results_index][0] = leftBumpValue;  // save leftBumpValue
        bump_data[results_index][1] = rightBumpValue; // save rightBumpValue
        AddSpeed_data[results_index][0] = Bumpvalue2Speed(rightBumpValue);
        AddSpeed_data[results_index][1] = Bumpvalue2Speed(leftBumpValue);
        results_index++;
      }
      else
      {
        StartCollectingData_flag = false;
      }
  }

  if (count >= 30000)
  {
    count = 0;
  }
}

unsigned long MeasureBumpSensor(int SensorPin, unsigned long &MaxBumpSensorTime, unsigned long &MinBumpSensorTime)

// unsigned long MeasureBumpSensor(int SensorPin)
{

  pinMode(SensorPin, OUTPUT);
  digitalWrite(SensorPin, HIGH);
  delay(10);

  pinMode(SensorPin, INPUT);

  unsigned long StartTime = micros();
  while (digitalRead(SensorPin) == HIGH)
  {
  }

  unsigned long EndTime = micros();
  unsigned long ElapsedTime = EndTime - StartTime;

  ElapsedTime < MinBumpSensorTime ? MinBumpSensorTime = ElapsedTime : MinBumpSensorTime;
  ElapsedTime > MaxBumpSensorTime ? MaxBumpSensorTime = ElapsedTime : MaxBumpSensorTime;

  return ElapsedTime;
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
