
/***************************************************************************
 *  ex_simple.cpp - KinDrv example - connect arm and read information
 *
 *  Created: Fri Oct 11 00:031:00 2013
 *  Copyright  2013  Bahram Maleki-Fard
 ****************************************************************************/

/*  This file is part of libkindrv.
 *
 *  libkindrv is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Foobar is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with libkindrv.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libkindrv/kindrv.h>

#include <stdio.h>

using namespace KinDrv;

int main()
{
  printf("KinDriv library usage example \n");

  // explicitly initialize a libusb context
  KinDrv::init_usb();

  printf("Create a JacoArm \n");
  JacoArm *arm;
  try {
    arm = new JacoArm();
  } catch( KinDrvException &e ) {
    printf("error %i: %s \n", e.error(), e.what());
    return 0;
  }

  printf("Get joint values. \n");
  jaco_position_t pos = arm->get_ang_pos();
  printf("joints: %f  %f  %f  %f  %f  %f \n",
         pos.joints[0], pos.joints[1], pos.joints[2],
         pos.joints[3], pos.joints[4], pos.joints[5]);


  printf("Get cartesian values + fingers. \n");
  pos = arm->get_cart_pos();
  printf("pos: %f  %f  %f \nrot: %f  %f  %f \nfingers: %f  %f  %f ",
         pos.position[0], pos.position[1], pos.position[2],
         pos.rotation[0], pos.rotation[1], pos.rotation[2],
         pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);


  // explicitly close libusb context
  KinDrv::close_usb();
  return 0;
}
