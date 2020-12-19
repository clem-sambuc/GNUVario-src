/* VarioImuTwoWire -- 
 *
 * Copyright 2020 MichelPa / Jpg63
 * 
 * This file is part of GnuVario-E.
 *
 * ToneHAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ToneHAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/* 
 *********************************************************************************
 *                                                                               *
 *                          VarioImuTwoWire                                         *
 *                                                                               *
 *  version    Date     Description                                              *
 *    1.0    22/03/20                                                            *
 *    1.0.1  25/03/20   Ajout haveMeasure(void)																	 *
 *                                                                               *
 *********************************************************************************
 */

#ifndef VARIOIMUTWOWIRE_H
#define VARIOIMUTWOWIRE_H

#include <HardwareConfig.h>

#ifdef TWOWIRESCHEDULER

#include <IntTW.h>
#include <ms5611TW.h>
#include <BiasCorrection.h>
#include <LightInvensense.h>
#include <TwoWireScheduler.h>

class VarioImuTwoWire
{

public:
  VarioImuTwoWire();
  void init();
  bool havePressure(void);
  bool updateData();
  void updateAlti();
  double getAlti();
  double getTemp();

  bool haveAccel();
	void getRawAccelQuat(int16_t* accel, int32_t* quat);
  void getStableAccelQuat(double* stableAccel, double* quaternions);

  bool haveMag();
	void getRawMag(int16_t* mag);

  void disableAcquisition();

private:
  double Alti;
  double Temp;
  double Accel[3];
  double Quat[4];
	int	CompteurAccel = 0;

  TWScheduler twScheduler;
};

#endif //TWOWIRESCHEDULER

#endif //VARIOIMUTWOWIRE_H