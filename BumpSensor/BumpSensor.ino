#define EMIT_PIN   11
#define BL 4
#define BR 5

unsigned long MaxBumpSensorTime = 0;


void setup() {
  Serial.begin(9600);
  
  pinMode (BL , INPUT);
  pinMode (BR , INPUT);
  
 

  pinMode(EMIT_PIN, OUTPUT);  
  digitalWrite(EMIT_PIN, LOW);
  
  //CalibrateBumpSensors();
}

void loop() {

  unsigned long BLElapsedTime = MeasureBumpSensor(BL);
  int leftBumpValue = 1023 - map(BLElapsedTime, 0, MaxBumpSensorTime, 0 ,1023);
  
  unsigned long BRElapsedTime = MeasureBumpSensor(BR);
  int rightBumpValue = 1023 - map(BRElapsedTime, 0, MaxBumpSensorTime, 0 ,1023); //Mapping digital value to discrete value
  
   
  Serial.print("Left Right Bump Sensor: ");
  Serial.print(leftBumpValue);
  Serial.print(",");
  Serial.println(rightBumpValue);
 
  delay(100); 
  
}

unsigned long MeasureBumpSensor(int SensorPin){
  
  pinMode(SensorPin , OUTPUT);
  digitalWrite(SensorPin , HIGH);
  delay(10);

  pinMode(SensorPin , INPUT);

  unsigned long StartTime = micros();
  while( digitalRead(SensorPin) == HIGH){
    
  }
  
  unsigned long EndTime = micros();
  unsigned long ElapsedTime = EndTime - StartTime;

  if(ElapsedTime >= MaxBumpSensorTime){
    MaxBumpSensorTime = ElapsedTime;
  }

  return ElapsedTime;
}

/*
void CalibrateBumpSensors() {
  
  int minLeft = 1023, maxLeft = 0;
  int minRight = 1023, maxRight = 0;

  unsigned long startCalibrationTime = millis();
  while (millis() - startCalibrationTime < 2000) { 
    int leftValue = analogRead(BL);
    int rightValue = analogRead(BR);

   
    if (leftValue > maxLeft) maxLeft = leftValue;
    if (leftValue < minLeft) minLeft = leftValue;

  
    if (rightValue > maxRight) maxRight = rightValue;
    if (rightValue < minRight) minRight = rightValue;

    delay(10);
  }

  Serial.println("Calibration complete.");
  Serial.print("Left Bump Min: "); Serial.println(minLeft);
  Serial.print("Left Bump Max: "); Serial.println(maxLeft);
  Serial.print("Right Bump Min: "); Serial.println(minRight);
  Serial.print("Right Bump Max: "); Serial.println(maxRight);
}
*/
