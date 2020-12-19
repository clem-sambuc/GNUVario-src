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

#include <BiasCorrection.h>

#include <Arduino.h>
#include <LightInvensense.h>

#ifdef BIAS_CORRECTION_ENABLE_EEPROM
#include <eepromHAL.h>
#endif

/* static variables */
#ifdef BIAS_CORRECTION_STATIC_CALIBRATION
constexpr uint8_t BiasCorrection::gyroCalArray[12];
constexpr int32_t BiasCorrection::accelCalArray[3];
#endif


/*-------------*/
/* calibration */
/*-------------*/
void BiasCorrection::readCurrentDMPGyroCalibration(uint8_t* gyroCal) {

  fastMPUReadGyroBias(gyroCal);
}


#ifdef BIAS_CORRECTION_ENABLE_EEPROM
/******************/
/* EEPROM methods */
/******************/
void readEEPROMValues(int address, uint16_t eepromTag, int length, uint8_t* data) {

  /* read tag */
  uint16_t tag = EEPROMHAL.read(address);
  address++;
  tag = (tag<<8) + EEPROMHAL.read(address);
  address++;

  /* read values */
  for( uint8_t i = 0; i<length; i++) {
    if( tag == eepromTag ) {
      *data = EEPROMHAL.read(address);
    } else {
      *data = 0;
    }
    data++;
    address++;
  }
}


void writeEEPROMValues(int address, uint16_t eepromTag, int length, const uint8_t* data) {

  /* write tag */
  EEPROMHAL.write(address, (eepromTag>>8) & 0xff);
  EEPROMHAL.write(address + 0x01, eepromTag & 0xff);
  address += 2;
  
  /* write values */
  for( uint8_t i = 0; i<length; i++) {
    EEPROMHAL.write(address, *data);
    data++;
    address++;
  }
}


BiasCorrectionSettings BiasCorrection::readEEPROMSettings(void) {

  BiasCorrectionSettings eepromSettings;
  readGyroCalibration(eepromSettings.gyroCal);
  readAccelCalibration(eepromSettings.accelCal);
#ifdef AK89xx_SECONDARY 
  readMagCalibration(eepromSettings.magCal);
#endif //AK89xx_SECONDARY

  return eepromSettings;
}


void BiasCorrection::saveGyroCalibration(const uint8_t* gyroCal) {

  writeEEPROMValues(BIAS_CORRECTION_GYRO_CAL_EEPROM_ADDR, BIAS_CORRECTION_GYRO_CAL_EEPROM_TAG, 12, gyroCal);
}

void BiasCorrection::readGyroCalibration(uint8_t* gyroCal) {

  readEEPROMValues(BIAS_CORRECTION_GYRO_CAL_EEPROM_ADDR, BIAS_CORRECTION_GYRO_CAL_EEPROM_TAG, 12, gyroCal);
}

void BiasCorrection::saveAccelCalibration(const BiasCorrectionCalibration& accelCal) {

  writeEEPROMValues(BIAS_CORRECTION_ACCEL_CAL_EEPROM_ADDR, BIAS_CORRECTION_ACCEL_CAL_EEPROM_TAG, sizeof(BiasCorrectionCalibration), (uint8_t*)(&accelCal));
}

void BiasCorrection::readAccelCalibration(BiasCorrectionCalibration& accelCal) {

  readEEPROMValues(BIAS_CORRECTION_ACCEL_CAL_EEPROM_ADDR, BIAS_CORRECTION_ACCEL_CAL_EEPROM_TAG, sizeof(BiasCorrectionCalibration), (uint8_t*)(&accelCal));
}

#ifdef AK89xx_SECONDARY
void BiasCorrection::saveMagCalibration(const BiasCorrectionCalibration& magCal) {

  writeEEPROMValues(BIAS_CORRECTION_MAG_CAL_EEPROM_ADDR, BIAS_CORRECTION_MAG_CAL_EEPROM_TAG, sizeof(BiasCorrectionCalibration), (uint8_t*)(&magCal));
}

void BiasCorrection::readMagCalibration(BiasCorrectionCalibration& magCal) {

  readEEPROMValues(BIAS_CORRECTION_MAG_CAL_EEPROM_ADDR, BIAS_CORRECTION_MAG_CAL_EEPROM_TAG, sizeof(BiasCorrectionCalibration), (uint8_t*)(&magCal));
}
#endif //AK89xx_SECONDARY
#endif //BIAS_CORRECTION_ENABLE_EEPROM
  


/***************/
/* init device */
/***************/
void BiasCorrection::init(void) {

  /* init MPU */
  fastMPUInit(false);

#ifndef BIAS_CORRECTION_STATIC_CALIBRATION
  uint8_t gyroCalArray[12] = {GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_00,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_01,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_02,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_03,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_04,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_05,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_06,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_07,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_08,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_09,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_10,
			      GnuSettings.VARIO_BIAS_CORRECTION_GYRO_CAL_BIAS_11 };
#endif
  /* set gyro calibration in the DMP */
  fastMPUSetGyroBias(gyroCalArray);
  

#ifndef BIAS_CORRECTION_STATIC_CALIBRATION
  /* set accel calibration in the DMP */
  int32_t accelBias[3] = { GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_00 << (15 - GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER),
			   GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_01 << (15 - GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER),
			   GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_02 << (15 - GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER)};

  fastMPUSetAccelBiasQ15(accelBias);
#else
  /* set accel calibration in the DMP */
  fastMPUSetAccelBiasQ15(accelCalArray);
#endif
    
  /* start DMP */
  fastMPUStart();
}

void BiasCorrection::stabilizeAccel(int16_t* imuAccel, double* stableAccel)
{
  /**************************************/
  /* stabilize using static calibration */
  /**************************************/
  
#ifndef BIAS_CORRECTION_STATIC_CALIBRATION
  
  int64_t calibratedAccel = (int64_t)imuAccel[0] << GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_00;
  calibratedAccel *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableAccel[0] = ((double)calibratedAccel)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

  calibratedAccel = (int64_t)imuAccel[1] << GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_01;
  calibratedAccel *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableAccel[1] = ((double)calibratedAccel)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

  calibratedAccel = (int64_t)imuAccel[2] << GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_02;
  calibratedAccel *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableAccel[2] = ((double)calibratedAccel)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

#else
  /* inline for optimization */
  int64_t calibratedAccel;
  calibratedAccel = (int64_t)imuAccel[0] << BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= (int64_t)settings.accelCal.bias[0];
  calibratedAccel *= ((int64_t)settings.accelCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableAccel[0] = ((double)calibratedAccel)/((double)((int64_t)1 << (BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

  calibratedAccel = (int64_t)imuAccel[1] << BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= (int64_t)settings.accelCal.bias[1];
  calibratedAccel *= ((int64_t)settings.accelCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableAccel[1] = ((double)calibratedAccel)/((double)((int64_t)1 << (BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

  calibratedAccel = (int64_t)imuAccel[2] << BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= (int64_t)settings.accelCal.bias[2];
  calibratedAccel *= ((int64_t)settings.accelCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableAccel[2] = ((double)calibratedAccel)/((double)((int64_t)1 << (BIAS_CORRECTION_ACCEL_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));
#endif
}

void BiasCorrection::scaleQuat(int32_t* imuQuat, double* scaledQuat)
{
  for(int i = 0; i<4; i++)
    scaledQuat[i] = ((double)imuQuat[i])/LIGHT_INVENSENSE_QUAT_SCALE;
}

#ifdef AK89xx_SECONDARY
void BiasCorrection::stabilizeMag(int16_t* mag, double* stableMag)
{
  /**************************************/
  /* stabilize using static calibration */
  /**************************************/

#ifndef BIAS_CORRECTION_STATIC_CALIBRATION
  int64_t calibratedMag;
  calibratedMag = ((int64_t)mag[0]) << GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_00;
  calibratedMag *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_PROJ_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableMag[0] = ((double)calibratedMag)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[1]) << GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_01;
  calibratedMag *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_PROJ_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableMag[1] = ((double)calibratedMag)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[2]) << GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_02;
  calibratedMag *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_PROJ_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableMag[2] = ((double)calibratedMag)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));
  
#else
  /* inline for optimization */
  int64_t calibratedMag;
  calibratedMag = ((int64_t)mag[0]) << BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)settings.magCal.bias[0];
  calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableMag[0] = ((double)calibratedMag)/((double)((int64_t)1 << (BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[1]) << BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)settings.magCal.bias[1];
  calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableMag[1] = ((double)calibratedMag)/((double)((int64_t)1 << (BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[2]) << BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)settings.magCal.bias[2];
  calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  stableMag[2] = ((double)calibratedMag)/((double)((int64_t)1 << (BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));
#endif
}

void BiasCorrection::computeNorthVector(double* vertVector, int16_t* mag, double* northVector) {

  /*-------------------------------*/
  /*   north vector computation    */
  /*-------------------------------*/
  
  double n[3];

#ifndef BIAS_CORRECTION_STATIC_CALIBRATION
  int64_t calibratedMag;
  calibratedMag = ((int64_t)mag[0]) << GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_00;
  calibratedMag *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_PROJ_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  n[0] = ((double)calibratedMag)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[1]) << GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_01;
  calibratedMag *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_PROJ_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  n[1] = ((double)calibratedMag)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[2]) << GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_02;
  calibratedMag *= ((int64_t)GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_PROJ_SCALE + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  n[2] = ((double)calibratedMag)/((double)((int64_t)1 << (GnuSettings.VARIO_BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));
  
#else
  /* inline for optimization */
  int64_t calibratedMag;
  calibratedMag = ((int64_t)mag[0]) << BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)settings.magCal.bias[0];
  calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  n[0] = ((double)calibratedMag)/((double)((int64_t)1 << (BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[1]) << BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)settings.magCal.bias[1];
  calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  n[1] = ((double)calibratedMag)/((double)((int64_t)1 << (BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));

  calibratedMag = ((int64_t)mag[2]) << BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER;
  calibratedMag -= (int64_t)settings.magCal.bias[2];
  calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << BIAS_CORRECTION_CAL_SCALE_MULTIPLIER));
  n[2] = ((double)calibratedMag)/((double)((int64_t)1 << (BIAS_CORRECTION_MAG_CAL_BIAS_MULTIPLIER + BIAS_CORRECTION_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));
#endif
        
  /* compute north vector by applying rotation from v to z to vector n */
  vertVector[2] = -1.0 - vertVector[2];
  northVector[0] = (1+vertVector[0]*vertVector[0]/vertVector[2])*n[0] + (vertVector[0]*vertVector[1]/vertVector[2])*n[1] - vertVector[0]*n[2];
  northVector[1] = (vertVector[0]*vertVector[1]/vertVector[2])*n[0] + (1+vertVector[1]*vertVector[1]/vertVector[2])*n[1] - vertVector[1]*n[2];
}

#endif


/* direct access to sensors */
boolean BiasCorrection::readRawAccelQuat(int16_t* accel, int32_t* quat) {

  boolean haveValue = false;

  while( fastMPUReadFIFO(NULL, accel, quat) >= 0 ) {
    haveValue = true;
  }

  return haveValue;
}

boolean BiasCorrection::readRawGyroQuat(int16_t* gyro, int32_t* quat) {

  boolean haveValue = false;

  while( fastMPUReadFIFO(gyro, NULL, quat) >= 0 ) {
    haveValue = true;
  }

  return haveValue;
}

boolean BiasCorrection::readRawSensor(int16_t* gyro, int16_t* accel, int32_t* quat) {

  boolean haveValue = false;

  while( fastMPUReadFIFO(gyro, accel, quat) >= 0 ) {
    haveValue = true;
  }

  return haveValue;
}
  
#ifdef AK89xx_SECONDARY
boolean BiasCorrection::readRawMag(int16_t* mag) {

  boolean haveValue = false;

  if( fastMPUMagReady() ) {
#ifdef BIAS_CORRECTION_USE_MAG_SENS_ADJ
    fastMPUReadMag(mag);
#else
    fastMPUReadRawMag(mag);
#endif //BIAS_CORRECTION_USE_MAG_SENS_ADJ
    haveValue = true;
  }

  return haveValue;
}  
#endif //AK89xx_SECONDARY
