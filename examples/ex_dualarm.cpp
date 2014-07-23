
/***************************************************************************
 *  ex_dualarm.cpp - KinDrv example - control two arms
 *
 *  Created: Wed Jul 23 02:19:00 2014
 *  Copyright  2014  Bahram Maleki-Fard
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
  printf("OpenJaco example, showing how to connect to more than 1 arm \n");

  // explicitly initialize a libusb context
  KinDrv::init_usb();

  JacoArm *arm;
  JacoArm *arm2;

  printf("\nCreate 1st JacoArm \n");
  try {
    arm = new JacoArm();
    printf("successfully connected arm '%s' \n", arm->get_client_config(false).name);

    printf("\nCreate 2nd JacoArm \n");
    arm2 = new JacoArm();
    printf("successfully connected 2nd arm '%s' \n", arm2->get_client_config(false).name);
  } catch( KinDrvException &e ) {
    printf("error %i: %s \n", e.error(), e.what());
    return 0;
  }

  // gain API control over the arms
  arm->start_api_ctrl();
  arm2->start_api_ctrl();

  // move them around a little, asynchronely. upon initialization, this should move towards HOME
  printf("move arms around a little \n");
  arm->push_joystick_button(2);
  usleep(1000*2000);
  arm2->push_joystick_button(2);
  usleep(1000*1000);
  arm2->release_joystick();
  usleep(1000*1000);
  arm->release_joystick();
  arm2->push_joystick_button(2);
  usleep(1000*1000);
  arm->push_joystick_button(2);
  usleep(1000*1000);
  arm->release_joystick();
  arm2->release_joystick();
  usleep(1000*50);

  // move to HOME , very simple with 'sleep', just assuming this works for now.
  // Check the ex_ctrl example to see how to handle proper HOME/RETRACT movement
  printf("move arms to HOME \n");
  sleep(3);
  arm->push_joystick_button(2);
  arm2->push_joystick_button(2);
  usleep(1000*10000);
  arm->release_joystick();
  arm2->release_joystick();
  usleep(1000*50);
  // now they should be at HOME.

  // move back to RETRACT
  printf("move arms to RETRACT \n");
  arm->push_joystick_button(2);
  arm2->push_joystick_button(2);
  usleep(1000*10000);
  arm->release_joystick();
  arm2->release_joystick();
  usleep(1000*50);



  // explicitly close libusb context
  KinDrv::close_usb();
  return 0;
}
