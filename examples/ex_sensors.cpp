
/***************************************************************************
 *  ex_sensors.cpp - KinDrv example - Connect to tactile sensors and read data
 *
 *  Created: Tue Mar 10 13:58:00 2015
 *  Copyright  2015  Bahram Maleki-Fard
 ****************************************************************************/

/*  This file is part of libkindrv.
 *
 *  libkindrv is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  libkindrv is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with libkindrv.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libkindrv/jaco_sensors.h>

#include <stdio.h>

#define NUM_READS 20

using namespace KinDrv;

int main()
{
  printf("KinDriv library sensors example \n");

  printf("Create a JacoSensors \n");
  JacoSensors *sensors;
  try {
    sensors = new JacoSensors();
  } catch( KinDrvException &e ) {
    printf("error %i: %s \n", e.error(), e.what());
    return 0;
  }

  printf("Continuously fetching data \n");
  jaco_sensor_pressure_t pr;
  unsigned short vibr;

  for(unsigned int read=0; read<NUM_READS; ++read) {

    // get data for each sensor separately
    for(unsigned int i=0; i<7; ++i) {
      try {
        // get pressure information
        pr = sensors->get_pressure(i);
        printf("\n");
        printf("[%u] N %.5hu  %.5hu  %.5hu  %.5hu \n", i+1, pr.normalized[0], pr.normalized[3], pr.normalized[6], pr.normalized[9]);
        printf("[%u] N %.5hu  %.5hu  %.5hu  %.5hu \n", i+1, pr.normalized[1], pr.normalized[4], pr.normalized[7], pr.normalized[10]);
        printf("[%u] N %.5hu  %.5hu  %.5hu  %.5hu \n", i+1, pr.normalized[2], pr.normalized[5], pr.normalized[8], pr.normalized[11]);
        printf("[%u] R %.5hu  %.5hu  %.5hu  %.5hu \n", i+1, pr.raw[0], pr.raw[3], pr.raw[6], pr.raw[9]);
        printf("[%u] R %.5hu  %.5hu  %.5hu  %.5hu \n", i+1, pr.raw[1], pr.raw[4], pr.raw[7], pr.raw[10]);
        printf("[%u] R %.5hu  %.5hu  %.5hu  %.5hu \n", i+1, pr.raw[2], pr.raw[5], pr.raw[8], pr.raw[11]);

        // get vibration information
        vibr = sensors->get_vibration(i);
        printf("[%u]  V %.5hu Hz \n", vibr);

      } catch(KinDrvException &e) {
        printf("ERROR: %s\n", e.what());
      }
      usleep(100e3);
    }
    usleep(500e3);
  }

  delete sensors;

  return 0;
}
