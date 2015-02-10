
/***************************************************************************
 *  ex_joystick.cpp - KinDrv example - simulate joystick movement
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

int
goto_retract(JacoArm *arm)
{
  // this can only be achieved from HOME position. Otherwise the arm
  // will move to HOME. You'll probably need to uncomment the gripper movements
  // in order for this to work. Or even better, implement moving to HOME position,
  // which could be triggered before going to RETRACT ;)
  jaco_retract_mode_t mode = arm->get_status();
  switch( mode ) {
    case MODE_READY_TO_RETRACT:
      // is currently on the way to RETRACT. Need 2 button presses,
      // 1st moves towards HOME, 2nd brings it back to its way to RETRACT
      arm->push_joystick_button(2);
      arm->release_joystick();
      arm->push_joystick_button(2);
      break;

    case MODE_READY_STANDBY:
    case MODE_RETRACT_TO_READY:
      // just 1 button press needed
      arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      printf("cannot go from NORMAL/NOINIT to RETRACT \n");
      return 0;
      break;

    case MODE_ERROR:
      printf("some error?! \n");
      return 0;
      break;

    case MODE_RETRACT_STANDBY:
      printf("nothing to do here \n");
      return 1;
      break;
  }

  while( mode != MODE_RETRACT_STANDBY ) {
    usleep(1000*10); // 10 ms
    mode = arm->get_status();
  }
  arm->release_joystick();

  return 1;
}


int
goto_home(JacoArm *arm)
{
  // going to HOME position is possible from all positions. Only problem is,
  // if there is some kinfo of error
  jaco_retract_mode_t mode = arm->get_status();
  switch( mode ) {
    case MODE_RETRACT_TO_READY:
      // is currently on the way to HOME. Need 2 button presses,
      // 1st moves towards RETRACT, 2nd brings it back to its way to HOME
      arm->push_joystick_button(2);
      arm->release_joystick();
      arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_READY_TO_RETRACT:
    case MODE_RETRACT_STANDBY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      // just 1 button press needed
      arm->push_joystick_button(2);
      break;

    case MODE_ERROR:
      printf("some error?! \n");
      return 0;
      break;

    case MODE_READY_STANDBY:
      printf("nothing to do here \n");
      return 1;
      break;
  }

  while( mode != MODE_READY_STANDBY ) {
    usleep(1000*10); // 10 ms
    mode = arm->get_status();
    if( mode == MODE_READY_TO_RETRACT ) {
      arm->release_joystick();
      arm->push_joystick_button(2);
    }
  }
  arm->release_joystick();

  return 1;
}



int main()
{
  printf("KinDrv example for controlling the arm \n");

  printf("Create a JacoArm \n");
  JacoArm *arm;
  try {
    arm = new JacoArm();
    printf("Successfully connected to arm! \n");
  } catch( KinDrvException &e ) {
    printf("error %i: %s \n", e.error(), e.what());
    return 0;
  }

  printf("Gaining API control over the arm \n");
  arm->start_api_ctrl();



  //check if we need to initialize arm
  jaco_retract_mode_t mode = arm->get_status();
  printf("Arm is currently in state: %i \n", mode);
  if( mode == MODE_NOINIT ) {
    //push the "HOME/RETRACT" button until arm is initialized
    jaco_joystick_button_t buttons = {0};
    buttons[2] = 1;
    arm->push_joystick_button(buttons);

    while( mode == MODE_NOINIT ) {
      usleep(1000*10); // 10 ms
      mode = arm->get_status();
    }

    arm->release_joystick();
  }
  printf("Arm is initialized now, state: %i \n", mode);

  // we want to start from home_position
  goto_home(arm);

  // need cartesian-control for joystick simulation.
  // Angular-control is also possible, then you would control each joint!
  arm->set_control_cart();
  usleep(1e6);

  printf("Sending joystick movements. We want the arm to: \n");
  // Check the documentation (or types.h) to see how to interprete the joystick-values.
  // Also make sure, that all the fields of a joystick-structs that should not have an effect are set to 0! So initialize all jaco_joystick_ structs with 0!
  jaco_joystick_axis_t axes = {0};

  printf("* translate forth \n");
  axes.trans_fb = -2.5;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  printf("* translate back \n");
  axes.trans_fb = 2.5;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  axes.trans_fb = 0.f;

  printf("* translate left \n");
  axes.trans_lr = 2.5f;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  printf("* translate right \n");
  axes.trans_lr = -2.5f;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  axes.trans_lr = 0.f;

  printf("* translate up \n");
  axes.trans_rot = 2.5;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  printf("* translate down \n");
  axes.trans_rot = -2.5;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  axes.trans_rot = 0.f;

  printf("* wrist up \n");
  axes.wrist_fb = 2.5f;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  printf("* wrist down \n");
  axes.wrist_fb = -2.5f;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  axes.wrist_fb = 0.f;

  printf("* wrist forth \n");
  axes.wrist_lr = 2.5;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  printf("* wrist back \n");
  axes.wrist_lr = -2.5;
  arm->move_joystick_axis(axes);
  usleep(2e6);
  arm->release_joystick();
  axes.wrist_lr = 0.f;

  printf("* rotate counter-clockwise \n");
  axes.wrist_rot = 2.5;
  arm->move_joystick_axis(axes);
  usleep(3e6);
  arm->release_joystick();
  axes.wrist_rot = 0.f;


  printf("Move arm back to RETRACT position \n");
  if( !goto_retract(arm) ) {
    printf("Failed to go to RETRACT. Go to HOME first \n");
    if( goto_home(arm) ) {
      printf("Try RETRACT again\n");
      goto_retract(arm);
    } else {
      printf("Also failed going to HOME. Might be some serious problem...hmm \n");
    }
  }

  return 0;
}
