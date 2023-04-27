//---------------------------------------------------------------------------------------------------
// Header files

#include "calculate_gestures.h"

//---------------------------------------------------------------------------------------------------
// Definitions

//---------------------------------------------------------------------------------------------------
// Variable definitions

extern float gimbal_yaw;
extern float gimbal_throttle;
extern int ADC_vals[4];
short gesture_key = 0;
float finger_angles[4] = {0, 0, 0, 0};
int ADC_MAX_VAL = (2 << ADC_NUM_BITS - 1) - 1;
extern float resistance_min[4];
extern float resistance_max[4];

//====================================================================================================
// Functions

void calculate_gestures() {
    for (int i = 0; i < 4; i++) {
        finger_angles[i] = calculate_finger_angle(i);
    }

    int gesture_key = detect_gestures(finger_angles);
    gimbal_throttle = 0;
//     if (gesture_key && 1 << 0) { // yaw right
//             gimbal_yaw = 0.5 + lerp(0, 0.5, finger_angles[3] / 90);
// //            gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
//     } else if (gesture_key && 1 >> 3) {
// 		gimbal_yaw = 0.5 - lerp(0, 0.5, finger_angles[0] / 90);
// //		gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
// 	}
    gimbal_yaw = 0.5 - lerp(0, 0.5, finger_angles[3] / 90) + lerp(0, 0.5, finger_angles[0] / 90);

    if (gesture_key >> 2 && gesture_key >> 1) {
    	gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
    }
}

int detect_gestures(float* finger_angles) {
    short gesture_key = 0;
    for (int i = 0; i < 4; i++) {
        if (finger_angles[i] > ANGLE_THRESH) {
            gesture_key |= 1 << (3 - i);
        } else {
            finger_angles[i] = 0;
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
	float voltage_flex = ADC_vals[finger_num] * VCC / ADC_MAX_VAL;
	float resistance_flex = (RESISTANCE_PULLDOWN * voltage_flex) / (VCC - voltage_flex);
	return resistance_flex;
}

void calibrate_init() {
	for (int i = 0; i < 4; i++) {
		resistance_min[i] = RESISTANCE_FLEXED;
		resistance_max[i] = RESISTANCE_UNFLEXED;
	}
}

//====================================================================================================
// END OF CODE
//====================================================================================================
