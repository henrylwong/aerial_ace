//=====================================================================================================
#ifndef CalculateGestures_h
#define CalculateGestures_h

#include "utils.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration

#define VCC 3.3
#define ADC_NUM_BITS 12
#define RESISTANCE_FLEXED 45 * 1000
#define RESISTANCE_UNFLEXED 10 * 1000
#define RESISTANCE_PULLDOWN 20 * 1000

#define FINGER_INDEX_IDX 3
#define FINGER_MIDDLE_IDX 2
#define FINGER_RING_IDX 1
#define FINGER_PINKY_IDX 0

#define ANGLE_THRESH 30

//---------------------------------------------------------------------------------------------------
// External declarations



//---------------------------------------------------------------------------------------------------
// Function declarations

int analog_read(int);
void calculate_gestures();
int detect_gestures(float*);
float calculate_finger_angle(int);
float calculate_finger_resistance(int);
void calibrate_init();

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
