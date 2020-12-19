/* BiasCorrection -- Normalize and calibrate Raw data from MPU 
 *
 * Copyright 2020-2021 Clément Sambuc
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*****************************************************************************************/
/*                               BiasCorrection                                          */
/*                                                                                       */
/*  Ver     Date                                                                         */
/*  1.0     18/11/20     Created from old vertaccel library                              */
/*                                                                                       */
/*****************************************************************************************/

#ifndef BIAS_CORRECTION_H
#define BIAS_CORRECTION_H

#include <Arduino.h>
#include <VarioSettings.h>
#include <HardwareConfig.h>

//to know about the magnetometer 
#include <LightInvensense.h>

/* use mag sensitivity adjustement */
#define BIAS_CORRECTION_USE_MAG_SENS_ADJ

/* enable EEPROM functionnalities */
#define BIAS_CORRECTION_ENABLE_EEPROM

/*########################################################*/
/* You can compile BiasCorrection with static             */
/* calibration coefficients.                              */
/* For this define  :                                     */
/*                                                        */
/* #define BIAS_CORRECTION_STATIC_CALIBRATION             */
/*                                                        */
/* And set the values with :                              */
/*                                                        */
/* #define BIAS_CORRECTION_GYRO_CAL_BIAS                  */
/* #define BIAS_CORRECTION_ACCEL_CAL_BIAS                 */
/* #define BIAS_CORRECTION_ACCEL_CAL_SCALE                */
/* #define BIAS_CORRECTION_MAG_CAL_BIAS                   */
/* #define BIAS_CORRECTION_MAG_CAL_PROJ_SCALE             */
/*                                                        */
/*########################################################*/

#if defined(BIAS_CORRECTION_STATIC_CALIBRATION) && \
  ( !defined(BIAS_CORRECTION_GYRO_CAL_BIAS) || \
    !defined(BIAS_CORRECTION_ACCEL_CAL_BIAS) || \
    !defined(BIAS_CORRECTION_ACCEL_CAL_SCALE) || \
    (defined(AK89xx_SECONDARY) && !defined(BIAS_CORRECTION_MAG_CAL_BIAS)) || \
    (defined(AK89xx_SECONDARY) && !defined(BIAS_CORRECTION_MAG_CAL_PROJ_SCALE)) )
#error Static calibration is enabled but static values not given !
#endif

/* base struct */
struct BiasCorrectionCalibration {
  int16_t bias[3];
  int16_t scale;
};

/* calibration settings */
struct BiasCorrectionSettings {
  uint8_t gyroCal[12]; //stored in machine representation
  BiasCorrectionCalibration accelCal;
#ifdef AK89xx_SECONDARY
  BiasCorrectionCalibration magCal;
#endif
};

/* default settings */
#define BIAS_CORRECTION_DEFAULT_MAG_PROJ_COEFF 0.745346
#define BIAS_CORRECTION_DEFAULT_MAG_CAL_PROJ_SCALE -16689

/* enable static or default settings */
#ifdef BIAS_CORRECTION_STATIC_CALIBRATION

constexpr BiasCorrectionSettings biasCorrectionSettings = {
  BIAS_CORRECTION_GYRO_CAL_BIAS
  ,{BIAS_CORRECTION_ACCEL_CAL_BIAS, BIAS_CORRECTION_ACCEL_CAL_SCALE}
#ifdef AK89xx_SECONDARY
  ,{BIAS_CORRECTION_MAG_CAL_BIAS, BIAS_CORRECTION_MAG_CAL_PROJ_SCALE}
#endif //AK89xx_SECONDARY
};

#else
  
constexpr BiasCorrectionSettings defaultBiasCorrectionSettings = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  ,{ {0, 0, 0}, 0 }
#ifdef AK89xx_SECONDARY
  ,{ {0, 0, 0}, BIAS_CORRECTION_DEFAULT_MAG_CAL_PROJ_SCALE}
#endif //AK89xx_SECONDARY
};

#endif //BIAS_CORRECTION_STATIC_CALIBRATION


/************************/
/* calibration settings */
/************************/

/* this is default values, it can be overwrited */

/* Bias is multiplied by 2^multiplier                 */
/* maximum bias correction value is 2^(15-multiplier) */
#ifndef BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER
#define BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER 6
#endif

#ifndef BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER
#define BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER 4
#endif

/* Measure is scaled by (1 + scale/2^multiplier) */
/* minimum scale is 1 - 2^(15-mutiplier) and maximum scale 1 + 2^(15-mutiplier) */
/* !! A default scale is applied before (see LightInvensense.h) !! */
#ifndef BIAS_CORRECTION_CAL_SCALE_MULTIPLIER
#define BIAS_CORRECTION_CAL_SCALE_MULTIPLIER 16
#endif

#ifdef BIAS_CORRECTION_ENABLE_EEPROM
/* read settings from EEPROM */
// see  BiasCorrection::readEEPROMSettings()
#define BIAS_CORRECTION_GYRO_CAL_EEPROM_ADDR 0x00
#define BIAS_CORRECTION_GYRO_CAL_EEPROM_TAG 0xf4e2
#define BIAS_CORRECTION_ACCEL_CAL_EEPROM_ADDR 0x0C
#define BIAS_CORRECTION_ACCEL_CAL_EEPROM_TAG 0xee54
#define BIAS_CORRECTION_MAG_CAL_EEPROM_ADDR 0x14
#define BIAS_CORRECTION_MAG_CAL_EEPROM_TAG 0x49f2
#define BIAS_CORRECTION_MAG_PROJ_EEPROM_ADDR 0x1C
#define BIAS_CORRECTION_MAG_PROJ_EEPROM_TAG 0x67fa
#endif //BIAS_CORRECTION_ENABLE_EEPROM


/*************************/
/*                       */
/*    The main class     */
/*                       */
/*************************/
class BiasCorrection{
  
 public:
#ifndef BIAS_CORRECTION_STATIC_CALIBRATION
 BiasCorrection(BiasCorrectionSettings baseSettings = defaultBiasCorrectionSettings) : settings(baseSettings) { }
#endif
  
  /* init device */
  void init(void);

  /* access to sensors */
  static boolean readRawAccelQuat(int16_t* accel, int32_t* quat);
	static boolean readRawGyroQuat(int16_t* gyro, int32_t* quat);
	static boolean readRawSensor(int16_t* gyro, int16_t* accel, int32_t* quat);

#ifdef AK89xx_SECONDARY
  static boolean readRawMag(int16_t* mag);
#endif //AK89xx_SECONDARY

  /* compute bias correction values*/
  void stabilizeAccel(int16_t* imuAccel, double* stableAccel);
	void scaleQuat(int32_t* imuQuat, double* scaledQuat);

#ifdef AK89xx_SECONDARY
  /* compute bias correction values*/
  void stabilizeMag(int16_t* mag, double* stableMag);
  
  void computeNorthVector(double* vertVector, int16_t* mag, double* northVector);
#endif
  
  /* calibration class methods */
  static void readCurrentDMPGyroCalibration(unsigned char* gyroCal);

#ifdef BIAS_CORRECTION_ENABLE_EEPROM
  /* EEPROM methods */
  static BiasCorrectionSettings readEEPROMSettings(void);
  static void saveGyroCalibration(const uint8_t* gyroCal);
  static void readGyroCalibration(uint8_t* gyroCal);
  static void saveAccelCalibration(const BiasCorrectionCalibration& accelCal);
  static void readAccelCalibration(BiasCorrectionCalibration& accelCal);
#ifdef AK89xx_SECONDARY
  static void saveMagCalibration(const BiasCorrectionCalibration& magCal);
  static void readMagCalibration(BiasCorrectionCalibration& magCal);
#endif //AK89xx_SECONDARY
#endif //BIAS_CORRECTION_ENABLE_EEPROM
 
  
 private:
#ifdef BIAS_CORRECTION_STATIC_CALIBRATION
  static constexpr BiasCorrectionSettings settings = biasCorrectionSettings;
  static constexpr uint8_t gyroCalArray[12] = BIAS_CORRECTION_GYRO_CAL_BIAS; //need to be passed as pointer
  static constexpr int32_t accelCalArray[3] = { (int32_t)biasCorrectionSettings.accelCal.bias[0] * ((int32_t)1 << (15 - BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER)),
						(int32_t)biasCorrectionSettings.accelCal.bias[1] * ((int32_t)1 << (15 - BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER)),
						(int32_t)biasCorrectionSettings.accelCal.bias[2] * ((int32_t)1 << (15 - BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER)) }; //passed as pointer
#else
  BiasCorrectionSettings settings;
#endif 
};


#endif

