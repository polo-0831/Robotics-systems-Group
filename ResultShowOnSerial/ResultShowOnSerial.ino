# define MAX_RESULTS 100
# define VARIABLES   2
float results[ MAX_RESULTS ][ VARIABLES ];

int results_index;

# define STATE_RUNNING_EXPERIMENT  0
# define STATE_FINISHED_EXPERIMENT 1
int state;

unsigned long experiment_start_ts;
# define EXPERIMENT_END_MS 10000       // expect the time of experiement (10 seconds)

 
unsigned long record_results_ts;       // Hou much second to do saving result 
unsigned long results_interval_ms;


void setup() {

  results_interval_ms = ( EXPERIMENT_END_MS / MAX_RESULTS );

  state = STATE_RUNNING_EXPERIMENT;

  results_index = 0;

  Serial.begin(9600);
  delay(1000);
  
  /*
  analogWrite( BUZZ_PIN, 120 );     //Beep to know when robot has activated or reset.
  delay( 200 );
  analogWrite( BUZZ_PIN, 0 );
  */
  
  experiment_start_ts = millis();     // Record start time into timestamps
  record_results_ts = millis();
}


void loop() {

  if( state == STATE_RUNNING_EXPERIMENT ) {

        // Primary code type in here................
    
    
        unsigned long elapsed_time;
        elapsed_time = millis() - record_results_ts;
        if( elapsed_time > results_interval_ms ) {
    
            record_results_ts = millis();
    
            if( results_index < MAX_RESULTS ) {
    
              results[ results_index ][0] = leftBumpValue;   // save leftBumpValue 
              results[ results_index ][1] = rightBumpValue;  // save rightBumpValue
    
              results_index++;
            } else {
              state = STATE_FINISHED_EXPERIMENT;
              return;
            }
    
        }
    
        elapsed_time = millis() - experiment_start_ts;
        if( elapsed_time > EXPERIMENT_END_MS ) {
          state = STATE_FINISHED_EXPERIMENT;
          return;
        }


  } else if ( state == STATE_FINISHED_EXPERIMENT) {

     // Type Finish & Stop State Code in here....... Such as (motor.setPWM(0,0)) 

    int result;
    Serial.println("LeftBumpValue & RightBumpValue : \n");
    for( result = 0; result < MAX_RESULTS; result++ ) {
      Serial.print( results[ result ][0] );                  // LeftBumpValue
      Serial.print(",");
      Serial.println( results[ result ][1] );                // RightBumpValue
    }

    delay(5000);   //delays 5 seconds to copy-paste from serial monitor 
  }

}
