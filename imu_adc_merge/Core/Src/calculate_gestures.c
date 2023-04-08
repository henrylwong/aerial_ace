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

#define FINGER_INDEX_PIN 1
#define FINGER_MIDDLE_PIN 2
#define FINGER_RING_PIN 3
#define FINGER_PINKY_PIN 4

#define FINGER_INDEX_IDX 3
#define FINGER_MIDDLE_IDX 2
#define FINGER_RING_IDX 1
#define FINGER_PINKY_IDX 0

#define ANGLE_THRESH 30

//---------------------------------------------------------------------------------------------------
// Variable definitions

extern float gimbal_yaw;
extern float gimbal_throttle;
float finger_pins[4] = {FINGER_PINKY_PIN, FINGER_RING_PIN, FINGER_MIDDLE_PIN, FINGER_INDEX_PIN}; // index: 3, middle: 2, ring: 1, pinky: 0
short gesture_key = 0;

//====================================================================================================
// Functions

/* Returns randomized ADC values for testing purposes */
int analog_read(int pin) {
    int min = (VCC * RESISTANCE_UNFLEXED/(RESISTANCE_UNFLEXED + RESISTANCE_PULLDOWN)) * (pow(2, ADC_NUM_BITS) - 1)/VCC;
    int max = (VCC * RESISTANCE_FLEXED/(RESISTANCE_FLEXED + RESISTANCE_PULLDOWN)) * (pow(2, ADC_NUM_BITS) - 1)/VCC;
    int res = rand() % (max - min) + min;
    printf("Analog val: %d ", res);
    return res;
}

void calculate_gestures() {
    float finger_angles[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        printf("Finger: %d; ", i);
        finger_angles[i] = calculate_finger_angle(finger_pins[i]);
        printf("Angle: %f\n", finger_angles[i]);
    }

    int gesture_key = detect_gestures(finger_angles);
    switch (gesture_key) {
        case 0b0111: // yaw right
            printf("Performing Yaw Right\n");
            gimbal_yaw = 0.5 + lerp(0, 0.5, finger_angles[FINGER_INDEX_IDX] / 90);
            gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
            break;
        case 0b1110: // yaw left
            printf("Performing Yaw Left\n");
            gimbal_yaw = 0.5 - lerp(0, 0.5, finger_angles[FINGER_PINKY_IDX] / 90);
            gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
            break;
        case 0b0110: // throttle
            printf("Performing Throttle\n");
            gimbal_yaw = 0;
            gimbal_throttle = lerp(0, 1, max(finger_angles[FINGER_MIDDLE_IDX], finger_angles[FINGER_RING_IDX]) / 90);
            break;
        default: // no-op
            printf("Performing No-Op\n");
            break;
    }
}

int detect_gestures(float* finger_angles) {
    short gesture_key = 0;
    for (int i = 0; i < 4; i++) {
        if (finger_angles[i] > ANGLE_THRESH) {
            gesture_key |= 1 << i;
        }
    }
    return gesture_key;
}

/*
 * Calculate finger angles
 * ADC_flex read by "analogRead(flexPin)" // analogRead later defined in flex sensor interface"
 */
float calculate_finger_angle(int flex_pin) {
    float ADC_flex = analog_read(flex_pin);
    float voltage_flex = ADC_flex * VCC / (pow(2, ADC_NUM_BITS) - 1);
    // float resistance_flex = RESISTANCE_PULLDOWN * (VCC / voltage_flex - 1);
    float resistance_flex = (RESISTANCE_PULLDOWN * voltage_flex) / (VCC - voltage_flex);
    float angle = map(resistance_flex, RESISTANCE_UNFLEXED, RESISTANCE_FLEXED, 0, 90);
    return angle;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
