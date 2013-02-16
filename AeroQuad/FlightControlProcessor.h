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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)


#ifndef _AQ_PROCESS_FLIGHT_CONTROL_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_H_

#define ATTITUDE_SCALING (0.75 * PWM2RAD)


/**
 * calculateFlightError
 *
 * Calculate roll/pitch axis error with gyro/accel data to
 * compute motor command thrust so used command are executed
 */
void calculateFlightError()
{
  #if defined (UseGPSNavigator)
    if (navigationState == ON || positionHoldState == ON) {
      float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS] + gpsRollAxisCorrection) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
      float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS] + gpsPitchAxisCorrection) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
      motorAxisCommandRoll   = updatePID(rollAttitudeCmd, gyroRate[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
      motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -gyroRate[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
    }
    else
  #endif
  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS]) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
    float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS]) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
    motorAxisCommandRoll   = updatePID(rollAttitudeCmd, gyroRate[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
    motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -gyroRate[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
  }
  else {
    motorAxisCommandRoll = updatePID(getReceiverSIData(XAXIS), gyroRate[XAXIS]*rotationSpeedFactor, &PID[RATE_XAXIS_PID_IDX]);
    motorAxisCommandPitch = updatePID(getReceiverSIData(YAXIS), -gyroRate[YAXIS]*rotationSpeedFactor, &PID[RATE_YAXIS_PID_IDX]);
  }
}

/**
 * processBatteryMonitorThrottleAdjustment
 *
 * Check battery alarm and if in alarm, increment a counter
 * When this counter reach BATTERY_MONITOR_MAX_ALARM_COUNT, then
 * we are now in auto-descent mode.
 *
 * When in auto-descent mode, the user can pass throttle keep when the
 * alarm was reach, and the throttle is slowly decrease for a minute til
 * batteryMonitorThrottle that is configurable with the configurator
 */
#if defined BattMonitor && defined BattMonitorAutoDescent
  void processBatteryMonitorThrottleAdjustment() {
    
    if (batteryMonitorAlarmCounter < BATTERY_MONITOR_MAX_ALARM_COUNT) {
      if (batteryAlarm) {
        batteryMonitorAlarmCounter++;
      }
    }
    else {
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        if (altitudeHoldState == ON) {
          #if defined AltitudeHoldBaro
            baroAltitudeToHoldTarget -= 0.01;
          #endif
          #if defined AltitudeHoldRangeFinder
            if (sonarAltitudeToHoldTarget != INVALID_RANGE) {
              sonarAltitudeToHoldTarget -= 0.01;
            }
          #endif
        }
        else {
      #endif
          if (batteryMonitorStartThrottle == 0) {  // init battery monitor throttle correction!
            batteryMonitorStartTime = millis();
            if (throttle < batteryMonitorThrottleTarget) {
              batteryMonitorStartThrottle = batteryMonitorThrottleTarget;
            }
            else {
              batteryMonitorStartThrottle = throttle; 
            }
          }
          int batteryMonitorThrottle = map(millis()-batteryMonitorStartTime, 0, batteryMonitorGoingDownTime, batteryMonitorStartThrottle, batteryMonitorThrottleTarget);
          if (batteryMonitorThrottle < batteryMonitorThrottleTarget) {
            batteryMonitorThrottle = batteryMonitorThrottleTarget;
          }
          if (throttle < batteryMonitorThrottle) {
            batteyMonitorThrottleCorrection = 0;
          }
          else {
            batteyMonitorThrottleCorrection = batteryMonitorThrottle - throttle;
          }
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        }
      #endif
    }
  }
#endif  

/**
 * processFlightControl
 *
 * Main flight control processos function
 */
void processFlightControl() {
  
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();
  
  if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  // 50hz task
    
    // ********************** Process position hold or navigation **************************
    #if defined (UseGPS)
      #if defined (UseGPSNavigator)
        processGpsNavigation();
      #endif  
    #endif
        
    // ********************** Process Battery monitor hold **************************
    #if defined BattMonitor && defined BattMonitorAutoDescent
      processBatteryMonitorThrottleAdjustment();
    #endif
  }

  // ********************** Calculate Motor Commands *************************
  if (motorArmed && safetyCheck) {
    applyMotorCommand();
  } 
 
  // Apply limits to motor commands
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
  }

  // *********************** Command Motors **********************
  if (motorArmed == ON && safetyCheck == ON) {
    writeMotors();
  }
}

#endif //#define _AQ_PROCESS_FLIGHT_CONTROL_H_

