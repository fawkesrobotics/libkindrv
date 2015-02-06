
/***************************************************************************
 *  ex_simple.cpp - KinDrv example - connect arm and read information
 *
 *  Created: Fri Oct 11 00:31:00 2013
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
  printf("pos: %f  %f  %f \nrot: %f  %f  %f \nfingers: %f  %f  %f \n",
         pos.position[0], pos.position[1], pos.position[2],
         pos.rotation[0], pos.rotation[1], pos.rotation[2],
         pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);


  printf("Get client configuration: \n");
  jaco_client_config_t config = arm->get_client_config();
  printf(" id:%s \n name:%s \n organizaion:%s \n sn:%s \n model_no:%s \n",
         config.id, config.name, config.organization, config.sn, config.model_no);


  printf("Get firmware information:\n");
  jaco_firmware_t firmware = arm->get_firmware();
  printf("      DSP = %u.%u.%u.%u \n", firmware.dsp[0], firmware.dsp[1], firmware.dsp[2], firmware.dsp[3]);
  for(unsigned int i=0; i<6; ++i)
    printf("  joint %u = %u.%u.%u \n", i, firmware.joint[i][0], firmware.joint[i][1], firmware.joint[i][2]);
  for(unsigned int i=0; i<3; ++i)
    printf(" finger %u = %u.%u.%u \n", i, firmware.finger[i][0], firmware.finger[i][1], firmware.finger[i][2]);
  for(unsigned int i=0; i<2; ++i)
    printf("    CAN %u = %u.%u.%u \n", i, firmware.can[i][0], firmware.can[i][1], firmware.can[i][2]);

 try {
    printf("Get status: \n");
    jaco_status_t status = arm->get_status();
    printf(" finger initialized : [%i, %i, %i] \n",
           status.finger_initialized[0], status.finger_initialized[1], status.finger_initialized[2]);
    printf(" retract mode       : %i \n", status.retract);
    printf(" controller         : %i \n", status.controller);
    printf(" force_control      : %i \n", status.force_control);
    printf(" current_limitation : %i \n", status.current_limitation);
    printf(" torque_sensors     : %i \n", status.torque_sensor);
  } catch(KinDrvException &e) {
    printf("ERROR: %s\n", e.what());
  }

  // explicitly close libusb context
  KinDrv::close_usb();
  return 0;
}
