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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad by
   editing UserConfiguration.h.

   If you need additional assistance go to http://www.aeroquad.com/forum.php
   or talk to us live on IRC #aeroquad
*****************************************************************************/

#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

//
// Define Security Checks
//

#if defined(UseGPS_NMEA) || defined(UseGPS_UBLOX) || defined(UseGPS_MTK) || defined(UseGPS_406)
 #define UseGPS
#endif 

#if defined(UseGPSNavigator) && !defined(AltitudeHoldBaro)
  #error "GpsNavigation NEED AltitudeHoldBaro defined"
#endif

#if defined(ReceiverSBUS) && defined(SlowTelemetry)
  #error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
#endif

#if defined (CameraTXControl) && !defined (CameraControl)
  #error "CameraTXControl need to have CameraControl defined"
#endif 

#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>
#include <FourtOrderFilter.h>
#ifdef BattMonitor
  #include <BatteryMonitorTypes.h>
#endif

//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

#ifdef AeroQuadSTM32
  #include "AeroQuad_STM32.h"
#endif

// default to 10bit ADC (AVR)
#ifndef ADC_NUMBER_OF_BITS
#define ADC_NUMBER_OF_BITS 10
#endif

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
  #if defined(HeadingMagHold)
	#include "Kinematics_TestMARG.h"
  #else
    #include "Kinematics_ARG.h"
  #endif

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined(ReceiverHWPPM)
  #include <Receiver_HWPPM.h>
#elif defined(ReceiverPPM)
  #include <Receiver_PPM.h>
#elif defined(AeroQuad_Mini) && (defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config))
  #include <Receiver_PPM.h>
#elif defined(RemotePCReceiver)
  #include <Receiver_RemotePC.h>
#elif defined(ReceiverSBUS)
  #include <Receiver_SBUS.h>
#elif defined(RECEIVER_328P)
  #include <Receiver_328p.h>
#elif defined(RECEIVER_MEGA)
  #include <Receiver_MEGA.h>
#elif defined(RECEIVER_APM)
  #include <Receiver_APM.h>
#elif defined(RECEIVER_STM32PPM)
  #include <Receiver_STM32PPM.h>  
#elif defined(RECEIVER_STM32)
  #include <Receiver_STM32.h>  
#endif

#if defined(UseAnalogRSSIReader) 
  #include <AnalogRSSIReader.h>
#elif defined(UseEzUHFRSSIReader)
  #include <EzUHFRSSIReader.h>
#elif defined(UseSBUSRSSIReader)
  #include <SBUSRSSIReader.h>
#endif



//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined(triConfig)
  #if defined (MOTOR_STM32)
    #define MOTORS_STM32_TRI
    #include <Motors_STM32.h>    
  #else
    #include <Motors_Tri.h>
  #endif
#elif defined(MOTOR_PWM)
  #include <Motors_PWM.h>
#elif defined(MOTOR_PWM_Timer)
  #include <Motors_PWM_Timer.h>
#elif defined(MOTOR_APM)
  #include <Motors_APM.h>
#elif defined(MOTOR_I2C)
  #include <Motors_I2C.h>
#elif defined(MOTOR_STM32)
  #include <Motors_STM32.h>    
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined(HMC5843)
  #include <HeadingFusionProcessorCompFilter.h>
  #include <Magnetometer_HMC5843.h>
#elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L)
  #include <HeadingFusionProcessorCompFilter.h>
  #include <Magnetometer_HMC5883L.h>
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
  #include <BarometricSensor_BMP085.h>
#elif defined(MS5611)
 #include <BarometricSensor_MS5611.h>
#endif
#if defined(XLMAXSONAR)
  #include <MaxSonarRangeFinder.h>
#endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#ifdef BattMonitor
  #include <BatteryMonitor.h>
  #ifndef BattCustomConfig
    #define BattCustomConfig BattDefaultConfig
  #endif
  struct BatteryData batteryData[] = {BattCustomConfig};
#endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#if defined(CameraControl_STM32)
  #include <CameraStabilizer_STM32.h>
#elif defined(CameraControl)
  #include <CameraStabilizer_Aeroquad.h>
#endif

#if defined (CameraTXControl)
  #include <CameraStabilizer_TXControl.h>
#endif

//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
#include "DriveControlCar.h"

//********************************************************
//****************** GPS DECLARATION *********************
//********************************************************
#if defined(UseGPS)
  #if !defined(HeadingMagHold)
    #error We need the magnetometer to use the GPS
  #endif 
  #include <GpsAdapter.h>
  #include "GpsNavigator.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION **************
//********************************************************
#ifdef MAX7456_OSD     // only OSD supported for now is the MAX7456
  #include <Device_SPI.h>
  #include "OSDDisplayController.h"
  #include "MAX7456.h"
#endif

#if defined(SERIAL_LCD)
  #include "SerialLCD.h"
#endif

#ifdef OSD_SYSTEM_MENU
  #if !defined(MAX7456_OSD) && !defined(SERIAL_LCD)
    #error "Menu cannot be used without OSD or LCD"
  #endif
  #include "OSDMenu.h"
#endif


//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#if defined(WirelessTelemetry) 
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define SERIAL_PORT Serial3
  #else    // force 328p to use the normal port
    #define SERIAL_PORT Serial
  #endif
#else  
  #if defined(SERIAL_USES_USB)   // STM32 Maple
    #define SERIAL_PORT SerialUSB
    #undef BAUD
    #define BAUD
  #else
    #define SERIAL_PORT Serial
  #endif
#endif  

#ifdef SlowTelemetry
  #include <AQ_RSCode.h>
#endif

#ifdef SoftModem
  #include <AQ_SoftModem.h>
#endif


// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#if defined(UseGPS) || defined(BattMonitor)
  #include "LedStatusProcessor.h"
#endif  

#if defined(MavLink)
  #include "MavLink.h"
#else
  #include "SerialCom.h"
#endif



/*******************************************************************
 * Main setup function, called one time at bootup
 * initialize all system and sub system of the
 * Aeroquad
 ******************************************************************/
void setup() {
  SERIAL_BEGIN(BAUD);

  pinMode(LED_Green, OUTPUT);
  digitalWrite(LED_Green, LOW);

  initCommunication();

  readEEPROM(); // defined in DataStorage.h
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
    initializeEEPROM();
    writeEEPROM();
  }

  initPlatform();

  
  #if defined(CarConfig)
     initializeMotors(FOUR_Motors);
  #elif defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config) || defined(triConfig)
     initializeMotors(FOUR_Motors);
  #elif defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config)
     initializeMotors(SIX_Motors);
  #elif defined(octoX8Config) || defined(octoXConfig) || defined(octoPlusConfig)
     initializeMotors(EIGHT_Motors);
  #endif

  // Setup receiver pins for pin change interrupts
  initializeReceiver(LASTCHANNEL);
  initReceiverFromEEPROM();


  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
  PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;
  
  // Optional Sensors
  #ifdef AltitudeHoldBaro
    initializeBaro();
    vehicleState |= ALTITUDEHOLD_ENABLED;
  #endif
  #ifdef AltitudeHoldRangeFinder
    inititalizeRangeFinders();
    vehicleState |= RANGE_ENABLED;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].P = PID[BARO_ALTITUDE_HOLD_PID_IDX].P*2;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].I = PID[BARO_ALTITUDE_HOLD_PID_IDX].I;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].D = PID[BARO_ALTITUDE_HOLD_PID_IDX].D;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].windupGuard = PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard;
  #endif

  #ifdef BattMonitor
    initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
    vehicleState |= BATTMONITOR_ENABLED;
  #endif
  
  #if defined(CameraControl)
    initializeCameraStabilization();
    vehicleState |= CAMERASTABLE_ENABLED;
  #endif

  #if defined(MAX7456_OSD)
    initializeSPI();
    initializeOSD();
  #endif
  
  #if defined(SERIAL_LCD)
    InitSerialLCD();
  #endif

  #if defined(BinaryWrite) || defined(BinaryWritePID)
    #ifdef OpenlogBinaryWrite
      binaryPort = &Serial1;
      binaryPort->begin(115200);
      delay(1000);
    #else
     binaryPort = &Serial;
    #endif
  #endif
  
  #if defined(UseGPS)
    initializeGps();
  #endif 

  #ifdef SlowTelemetry
     initSlowTelemetry();
  #endif

  
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first

  // Initialize gyro
  initializeGyro(); // defined in Gyro.h
  // Calibrate gyro
  if (calibrateGyro2() == true) {
	 //If board is in motion, gyroZero will have wrong values.
	 //If board was motionless, save values to EEPROM.
	 //storeSensorsZeroToEEPROM();
  } 
  
  // Initialize accel
  initializeAccel(); // defined in Accel.h
  initSensorsZeroFromEEPROM();

  delay(2000);

  measureAccel();
  setupFourthOrder(meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS]); //should be intialized after accelerometer.

  #ifdef HeadingMagHold
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeMagnetometer(); //assumes that quad x/y angle = 0.
    //initializeHeadingFusion();
  #endif

  #ifdef HeadingMagHold
	initializeKinematics(meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], hdgX, hdgY); //Testing ARG with heading compensation from magnetometer.
  #else
	initializeKinematics(); //
  #endif



  previousTime = micros();
  digitalWrite(LED_Green, HIGH);
  safetyCheck = 0;
}


/*******************************************************************
 * 100Hz task
 ******************************************************************/
void process100HzTask() {
  
  G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
  hundredHZpreviousTime = currentTime;
  
  evaluateGyroRate();
  evaluateMetersPerSec();

  for (int axis = XAXIS; axis <= ZAXIS; axis++) {
    filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
  }
  #if defined(HeadingMagHold)
	calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 
						filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], 
						measuredMag[XAXIS], measuredMag[YAXIS], measuredMag[ZAXIS],
						G_Dt);
  trueNorthHeading = kinematicsAngle[ZAXIS];
  #else
	calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 
						filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], 
						G_Dt);
  #endif
  

    

  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    zVelocity = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS] - runtimeZBias;
    if (!runtimaZBiasInitialized) {
      runtimeZBias = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS];
      runtimaZBiasInitialized = true;
    }
    estimatedZVelocity += zVelocity;
    estimatedZVelocity = (velocityCompFilter1 * zVelocity) + (velocityCompFilter2 * estimatedZVelocity);
  #endif    

  #if defined(AltitudeHoldBaro)
    measureBaroSum(); 
    if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  //  50 Hz tasks
      evaluateBaroAltitude();
    }
  #endif
        
  processFlightControl();
  
  
  #if defined(BinaryWrite)
    if (fastTransfer == ON) {
      // write out fastTelemetry to Configurator or openLog
      fastTelemetry();
    }
  #endif      
  
  #ifdef SlowTelemetry
    updateSlowTelemetry100Hz();
  #endif

  #if defined(UseGPS)
    updateGps();
  #endif      
  
  #if defined(CameraControl)
    moveCamera(kinematicsAngle[YAXIS],kinematicsAngle[XAXIS],kinematicsAngle[ZAXIS]);
    #if defined CameraTXControl
      processCameraTXControl();
    #endif
  #endif       

}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
void process50HzTask() {
  G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
  fiftyHZpreviousTime = currentTime;

  // Reads external pilot commands and performs functions based on stick configuration
  readPilotCommands(); 
  
  #if defined(UseAnalogRSSIReader) || defined(UseEzUHFRSSIReader) || defined(UseSBUSRSSIReader)
    readRSSI();
  #endif

  #ifdef AltitudeHoldRangeFinder
    updateRangeFinders();
  #endif

  #if defined(UseGPS)
    if (haveAGpsLock() && !isHomeBaseInitialized()) {
      initHomeBase();
    }
  #endif      
}

/*******************************************************************
 * 10Hz task
 ******************************************************************/
void process10HzTask1() {
  #if defined(HeadingMagHold)
    G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
    tenHZpreviousTime = currentTime;
     
    measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);
    
    //calculateHeading();
    
  #endif
}

/*******************************************************************
 * low priority 10Hz task 2
 ******************************************************************/
void process10HzTask2() {
  G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
  lowPriorityTenHZpreviousTime = currentTime;
  
  #if defined(BattMonitor)
    measureBatteryVoltage(G_Dt*1000.0);
  #endif

  // Listen for configuration commands and reports telemetry
  readSerialCommand();
  sendSerialTelemetry();
}

/*******************************************************************
 * low priority 10Hz task 3
 ******************************************************************/
void process10HzTask3() {
    G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
    lowPriorityTenHZpreviousTime2 = currentTime;

    #ifdef OSD_SYSTEM_MENU
      updateOSDMenu();
    #endif

    #ifdef MAX7456_OSD
      updateOSD();
    #endif
    
    #if defined(UseGPS) || defined(BattMonitor)
      processLedStatus();
    #endif
    
    #ifdef SlowTelemetry
      updateSlowTelemetry10Hz();
    #endif
}

/*******************************************************************
 * 1Hz task 
 ******************************************************************/
void process1HzTask() {
  #ifdef MavLink
    G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
    oneHZpreviousTime = currentTime;
    
    sendSerialHeartbeat();   
  #endif
}

/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () {
  
  currentTime = micros();
  deltaTime = currentTime - previousTime;

  measureCriticalSensors();

  // ================================================================
  // 100Hz task loop
  // ================================================================
  if (deltaTime >= 10000) {
    
    frameCounter++;
    
    process100HzTask();

    // ================================================================
    // 50Hz task loop
    // ================================================================
    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
      process50HzTask();
    }

    // ================================================================
    // 10Hz task loop
    // ================================================================
    if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
      process10HzTask1();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) {
      process10HzTask2();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) {
      process10HzTask3();
    }
    
    // ================================================================
    // 1Hz task loop
    // ================================================================
    if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
      process1HzTask();
    }
    
    previousTime = currentTime;
  }
  
  if (frameCounter >= 100) {
      frameCounter = 0;
  }
}



