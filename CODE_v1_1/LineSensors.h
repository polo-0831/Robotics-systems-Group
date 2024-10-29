/***************************************
 ,        .       .           .     ,-.
 |        |       |           |        )
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /
 |    | | | | `-. | | |-' |-' |      /
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--'
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#define LEFT 1
#define RIGHT 2

#define NUM_SENSORS 5
#define BLACK_THRES 0.6

const int sensor_pins[NUM_SENSORS] = {A11, A0, A2, A3, A4};

#define EMIT_PIN 11

class LineSensors_c
{

public:
  // Store your readings into this array.
  // You can then access these readings elsewhere
  // by using the syntax line_sensors.readings[n];
  // Where n is a value [0:4]
  float readings[NUM_SENSORS];

  // Variables to store calibration constants.
  // Make use of these as a part of the exercises
  // in labsheet 2.
  float minimum[NUM_SENSORS] = {2000, 2000, 2000, 2000, 2000};
  float maximum[NUM_SENSORS] = {0, 0, 0, 0, 0};
  float scaling[NUM_SENSORS] = {0, 0, 0, 0, 0};

  // Variable to store the calculated calibrated
  // (corrected) readings. Needs to be updated via
  // a function call, which is completed in
  // labsheet 2.
  float calibrated[NUM_SENSORS];

  // Constructor, must exist.
  LineSensor_c()
  {
    // leave this empty
  }

  // Refer to Labsheet 2: Approach 1
  // Fix areas marked ????
  // Use this function to setup the pins required
  // to perform an read of the line sensors using
  // the ADC.
  void initialiseForADC()
  {

    // Ensure that the IR LEDs are on
    // for line sensing
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);

    // Configure the line sensor pins
    // DN1, DN2, DN3, DN4, DN5.
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++)
    {
      pinMode(sensor_pins[sensor], INPUT_PULLUP);
    }

  } // End of initialiseForADC()

  // Refer to Labsheet 2: Approach 1
  // Fix areas marked ????
  // This function is as simple as using a call to
  // analogRead()
  void readSensorsADC()
  {

    // First, initialise the pins.
    // You need to complete this function (above).
    initialiseForADC();

    for (int sensor = 0; sensor < NUM_SENSORS; sensor++)
    {
      readings[sensor] = analogRead(sensor_pins[sensor]);
    }

  } // End of readSensorsADC()

  // Use this function to apply the calibration values
  // that were captured in your calibration routine.
  // Therefore, you will need to write a calibration
  // routine (see Labsheet 2)
  void CalibratedADC()
  {

    // Get latest readings (raw values)
    readSensorsADC();

    // Apply calibration values, store in calibrated[]
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++)
    {
      readings[sensor] < minimum[sensor] ? minimum[sensor] = readings[sensor] : minimum[sensor];
      readings[sensor] > maximum[sensor] ? maximum[sensor] = readings[sensor] : maximum[sensor];
      scaling[sensor] = maximum[sensor] - minimum[sensor];
    }

  } // End of calcCalibratedADC()
  void calcReadingADC()
  {
    readSensorsADC();
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++)
    {
      calibrated[sensor] = (readings[sensor] - minimum[sensor]) / scaling[sensor];
    }
  }
  // Part of the Advanced Exercises for Labsheet 2
  void initialiseForDigital()
  {

    // Ensure that the IR LEDs are on
    // for line sensing
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);

  } // End of initialiseForDigital()

  // Part of the Advanced Exercises for Labsheet 2
  void readSensorsDigital()
  {
    //  ???
  } // End of readSensorsDigital()

  int DetectBlackLine()
  {
    if (calibrated[0] > BLACK_THRES || calibrated[1] > BLACK_THRES)
      return RIGHT;
    else if (calibrated[2] > BLACK_THRES || calibrated[3] > BLACK_THRES || calibrated[4] > BLACK_THRES)
      return LEFT;
    else
      return 0;
  }
};

#endif
