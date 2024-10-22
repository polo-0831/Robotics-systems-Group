/***************************************
 ,        .       .           .     ,--,
 |        |       |           |       /
 |    ,-: |-. ,-. |-. ,-. ,-. |-     `.
 |    | | | | `-. | | |-' |-' |        )
 `--' `-` `-' `-' ' ' `-' `-' `-'   `-'
***************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3
extern LIS3MDL mag;
class Magnetometer_c
{

public:
  // Instance of the LIS3MDL class used to
  // interact with the magnetometer device.
  // LIS3MDL mag;

  // A place to store the latest readings
  // from the magnetometer
  volatile float readings[MAX_AXIS] = {0, 0, 0};

  float minimum[MAX_AXIS] = {10000, 10000, 10000};
  float maximum[MAX_AXIS] = {-10000, -10000, -10000};
  float range[MAX_AXIS] = {0, 0, 0};
  float offset[MAX_AXIS] = {0, 0, 0};
  float normalising[MAX_AXIS] = {0, 0, 0};

  float calibrated[MAX_AXIS];
  float meg_distance;
  // Constructor, must exist.
  Magnetometer_c()
  {
    // Leave this empty.
    // If you put Wire.begin() into this function
    // it will crash your microcontroller.
  }

  // Call this function witin your setup() function
  // to initialise the I2C protocol and the
  // magnetometer axis
  bool initialise()
  {

    // Start the I2C protocol
    Wire.begin();

    // Try to connect to the magnetometer
    if (!mag.init())
    {
      return false;
    }
    else
    {
      return true;
    }
  } // End of initialise()

  // Function to update readings array with
  // latest values from the axis over i2c
  void getReadings()
  {
    mag.read();
    readings[0] = mag.m.x;
    readings[1] = mag.m.y;
    readings[2] = mag.m.z;
  } // End of getReadings()

  void CalibratedMag()
  {
    getReadings();
    for (int axis = 0; axis < MAX_AXIS; axis++)
    {
      readings[axis] < minimum[axis] ? minimum[axis] = readings[axis] : minimum[axis];
      readings[axis] > maximum[axis] ? maximum[axis] = readings[axis] : maximum[axis];
      range[axis] = maximum[axis] - minimum[axis];
      offset[axis] = minimum[axis] + (range[axis] / 2.0);
      normalising[axis] = 1.0 / (range[axis] / 2.0);
    }
  }

  void calcReadingMeg()
  {
    getReadings();
    for (int axis = 0; axis < MAX_AXIS; axis++)
    {
      calibrated[axis] = (readings[axis] - offset[axis]) * normalising[axis];
    }
    meg_distance = sqrt(pow(calibrated[0], 2) + pow(calibrated[1], 2) + pow(calibrated[2], 2));
  }
}; // End of Magnetometer_c class definition

#endif
