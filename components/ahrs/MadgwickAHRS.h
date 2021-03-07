//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRS_init(float sampleFreqDef, float betaDef);
void MadgwickAHRS_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRS_update_IMU(float gx, float gy, float gz, float ax, float ay, float az);
void Madgwick_get_euler_angles_degrees(float *heading, float *pitch, float *roll);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
