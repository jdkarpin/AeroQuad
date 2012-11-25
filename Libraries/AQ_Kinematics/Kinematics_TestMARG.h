/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AQ_KINEMATICS_ARG_
#define _AQ_KINEMATICS_ARG_

//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================


////////////////////////////////////////////////////////////////////////////////
// ARG - Accelerometer, Rate Gyro
////////////////////////////////////////////////////////////////////////////////


#include "Kinematics.h"

#include <AQMath.h>

float kpAcc = 0.0;                					// proportional gain governs rate of convergence to accelerometer
float kiAcc = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float kpMag = 0.0;                					// proportional gain governs rate of convergence to magnetometer
float kiMag = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float halfT = 0.0;                					// half the sample period
float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;       // quaternion elements representing the estimated orientation
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		// scaled integral error
  
float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;


////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void margUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt) {
  
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy;//, wz;
  float q0i, q1i, q2i, q3i;
  float exAcc, eyAcc, ezAcc;
  float ezMag;
    
  halfT = G_Dt/2;
    
  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;         

  // compute reference direction of flux
  hx = mx * 2*(0.5 - q2*q2 - q3*q3) + my * 2*(q1*q2 - q0*q3)       + mz * 2*(q1*q3 + q0*q2);
  hy = mx * 2*(q1*q2 + q0*q3)       + my * 2*(0.5 - q1*q1 - q3*q3) + mz * 2*(q2*q3 - q0*q1);
  hz = mx * 2*(q1*q3 - q0*q2)       + my * 2*(q2*q3 + q0*q1)       + mz * 2*(0.5 - q1*q1 - q2*q2);
    
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;


  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
      
  wx = bx * 2*(0.5 - q2*q2 - q3*q3) + bz * 2*(q1*q3 - q0*q2);
  wy = bx * 2*(q1*q2 - q0*q3)       + bz * 2*(q0*q1 + q2*q3);
  //wz = bx * 2*(q0*q2 + q1*q3)       + bz * 2*(0.5 - q1*q1 - q2*q2);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  exAcc = (vy*az - vz*ay);
  eyAcc = (vz*ax - vx*az);
  ezAcc = (vx*ay - vy*ax);
    
  //exMag = (my*wz - mz*wy);
  //eyMag = (mz*wx - mx*wz);
  ezMag = (mx*wy - my*wx);
    
  // integral error scaled integral gain
  exInt = exInt + exAcc*kiAcc;
  if (isSwitched(previousEx,exAcc)) {
    exInt = 0.0;
  }
  previousEx = exAcc;
	
  eyInt = eyInt + eyAcc*kiAcc;
  if (isSwitched(previousEy,eyAcc)) {
    eyInt = 0.0;
  }
  previousEy = eyAcc;

  ezInt = ezInt + ezAcc*kiAcc;
  if (isSwitched(previousEz,ezAcc)) {
    ezInt = 0.0;
  }
  previousEz = ezAcc;
	
  // adjusted gyroscope measurements
  gx = gx + kpAcc*exAcc + exInt;
  gy = gy + kpAcc*eyAcc + eyInt;
  //gz = gz + Kp*ez + ezInt;
  gz = gz + kpAcc*ezAcc + ezMag*kpMag + ezInt;

  
  // integrate quaternion rate and normalise
  q0i = (-q1*gx - q2*gy - q3*gz) * halfT;
  q1i = ( q0*gx + q2*gz - q3*gy) * halfT;
  q2i = ( q0*gy - q1*gz + q3*gx) * halfT;
  q3i = ( q0*gz + q1*gy - q2*gx) * halfT;
  q0 += q0i;
  q1 += q1i;
  q2 += q2i;
  q3 += q3i;
    
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
}
  
void eulerAngles()
{
  kinematicsAngle[XAXIS]  = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  kinematicsAngle[YAXIS] = asin(2 * (q0*q2 - q1*q3));
  kinematicsAngle[ZAXIS]   = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}

////////////////////////////////////////////////////////////////////////////////
// Initialize ARG
////////////////////////////////////////////////////////////////////////////////

void initializeKinematics() 
{
  initializeBaseKinematicsParam();
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
	
  previousEx = 0;
  previousEy = 0;
  previousEz = 0;

  kpAcc = 0.2;
  kiAcc = 0.0005;
    
  kpMag = 2.0;//2.0;
  kiMag = 0.005;//0.005;
}

void initializeKinematics(float ax, float ay, float az, float hdgX, float hdgY)  
{
  float norm = 1, rollAngle, pitchAngle, pi, tmp;
  float yawAngle = atan2(hdgY, hdgX);

  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  tmp = atan2(ay, sqrt(ax*ax+az*az));
  pi = radians(180);

  if (az < 0) { //board up
	tmp = -tmp;
  } else {		//board upside down
	if (ay >= 0) {
	  tmp -= pi;
	} else {
	  tmp += pi;
	}
  }
  
  rollAngle = tmp;
  pitchAngle = atan2(ax, sqrt(ay*ay+az*az));

  initializeBaseKinematicsParam(rollAngle, pitchAngle, yawAngle);

  q0 = cos(yawAngle/2);
  q1 = 0;
  q2 = 0;
  q3 = sin(yawAngle/2);
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
	
  previousEx = 0;
  previousEy = 0;
  previousEz = 0;

  kpAcc = 0.2;
  kiAcc = 0.0005;
    
  kpMag = 2.0;//2.0;
  kiMag = 0.005;//0.005;
}
////////////////////////////////////////////////////////////////////////////////
// Calculate ARG
////////////////////////////////////////////////////////////////////////////////
void calculateKinematics(float rollRate,          float pitchRate,    float yawRate,  
                 float longitudinalAccel, float lateralAccel, float verticalAccel, 
                 float measuredMagX,      float measuredMagY, float measuredMagZ,
				 float G_Dt) {
    
  margUpdate(rollRate,          pitchRate,    yawRate, 
             longitudinalAccel, lateralAccel, verticalAccel,  
             measuredMagX,      measuredMagY, measuredMagZ,
		     G_Dt);
  eulerAngles();
}
  
float getGyroUnbias(byte axis) {
  return correctedRateVector[axis];
}
  
void calibrateKinematics() {}


#endif

