#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
// #include <time.h>

#define SETUP_SUCCESS 0
#define SETUP_FAIL -1

#define GIMBAL_IDLE_THRESH 0.1

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

// Resistance ranged used in Calibration Unflexed/Flexed states
#define RESISTANCE_RANGE_THRESH 10000

#define INIT_TIME_SEC 2
#define CAL_TIME_SEC 5
typedef enum {INIT, CAL_UNFLEXED, CAL_FLEXED, MODE_STANDARD, MODE_ADVANCED} states;

/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
  union {
    struct {
      float x; ///< X component of vector
      float y; ///< Y component of vector
      float z; ///< Z component of vector
    };         ///< Struct for holding XYZ component
  };                 ///< Union that can hold 3D vector array, XYZ components or
} sensors_vec_t;

/** struct sensor_event_s is used to provide a single sensor event in a common format. 
 *  accel range: -156.8 to 156.8
 *  magnetic range: 
 *  gyro range: -34.91 to 34.91
*/
typedef struct {
  union {
    sensors_vec_t acceleration; /**< acceleration values are in meter per second
                                   per second (m/s^2) */
    sensors_vec_t
        magnetic; /**< magnetic vector values are in micro-Tesla (uT) */
    sensors_vec_t gyro;        /**< gyroscope values are in rad/s */
  };                       ///< Union for the wide ranges of data we can carry
} sensors_event_t;

float max(float, float);
float min(float, float);
float clamp(float, float, float);
float lerp(float, float, float);
float map(float, float, float, float, float);
float convert_period_to_freq(float);
float convert_freq_to_period(float);

#endif
