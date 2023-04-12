//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h


//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

#define atan2_offset M_PI
#define asin_offset M_PI / 2

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// External declarations

void calculate_orientation(float delta_time);
void convert_quaternion_to_euler();
void remap_angles_to_gimbals();
void reset_aux_frame();

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float sample_freq);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float sample_freq);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
