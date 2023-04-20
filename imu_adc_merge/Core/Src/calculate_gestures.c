//---------------------------------------------------------------------------------------------------
// Header files

#include "calculate_gestures.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define VCC 3.3
#define ADC_NUM_BITS 12
#define RESISTANCE_FLEXED 30 * 1000
#define RESISTANCE_UNFLEXED 10 * 1000
#define RESISTANCE_PULLDOWN 20 * 1000

#define FINGER_INDEX_IDX 3
#define FINGER_MIDDLE_IDX 2
#define FINGER_RING_IDX 1
#define FINGER_PINKY_IDX 0

#define ANGLE_THRESH 30

//---------------------------------------------------------------------------------------------------
// Variable definitions

extern float gimbal_yaw;
extern float gimbal_throttle;
extern int ADC_vals[4];
short gesture_key = 0;
float finger_angles[4] = {0, 0, 0, 0};
int ADC_max_val = 2 << (ADC_NUM_BITS - 1) - 1;
extern float resistance_min[4];
extern float resistance_max[4];

//====================================================================================================
// Functions

void calculate_gestures() {
    for (int i = 0; i < 4; i++) {
        finger_angles[i] = calculate_finger_angle(i);
    }

    int gesture_key = detect_gestures(finger_angles);
    switch (gesture_key) {
        case 0b0001: // yaw right
            gimbal_yaw = 0.5 + lerp(0, 0.5, finger_angles[FINGER_INDEX_IDX] / 90);
            gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
            break;
        case 0b1000: // yaw left
            gimbal_yaw = 0.5 - lerp(0, 0.5, finger_angles[FINGER_PINKY_IDX] / 90);
            gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
            break;
        case 0b0110: // throttle
            gimbal_yaw = 0;
            gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
            break;
        default: // no-op
        	gimbal_yaw = 0.5;
        	gimbal_throttle = 0;
            break;
    }
    if (gesture_key >> 2 && gesture_key >> 1) {
    	gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
    }
}

int detect_gestures(float* finger_angles) {
    short gesture_key = 0;
    for (int i = 0; i < 4; i++) {
        if (finger_angles[i] > ANGLE_THRESH) {
            gesture_key |= 1 << (3 - i);
        }
    }
    return gesture_key;
}

/*
 * Calculate finger angles
 * ADC_flex read by "analogRead(flexPin)" // analogRead later defined in flex sensor interface
 */
float calculate_finger_angle(int finger_num) {
    float resistance_flex = calculate_finger_resistance(finger_num);
    float angle = 90 - map(resistance_flex, resistance_min[finger_num], resistance_max[finger_num], 0, 90);
    return angle;
}

float calculate_finger_resistance(int finger_num) {
	float voltage_flex = ADC_vals[finger_num] * VCC / ADC_max_val;
	float resistance_flex = (RESISTANCE_PULLDOWN * voltage_flex) / (VCC - voltage_flex);
	return resistance_flex;
}

void calibrate_init() {
	for (int i = 0; i < 4; i++) {
		resistance_min[i] = 0;
		resistance_max[i] = RESISTANCE_FLEXED;
	}
}

//====================================================================================================
// END OF CODE
//====================================================================================================
