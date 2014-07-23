
/***************************************************************************
 *  kindrv.h - KinDrv main header file for arm control
 *
 *  Created: Fri Oct 11 00:31:00 2013
 *  Copyright  2013  Bahram Maleki-Fard
 *  Copyright  2014  Tekin Mericli
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

#ifndef _KINDRV_H
#define _KINDRV_H

#include "types.h"
#include "exception.h"

#include <boost/thread/mutex.hpp>

struct libusb_device;
struct libusb_device_handle;

namespace KinDrv {

// just to allow using them explicitly. The API will use them implicitly if necessary
error_t init_usb();
void close_usb();

void list_devices();

/// \brief Main JacoArm class. All communication is done via an instance of this
class JacoArm
{
 public:
  JacoArm();
  virtual ~JacoArm();

  // controlling the arm via API (instead of connected joystick)
  void start_api_ctrl();
  void stop_api_ctrl();

  // start/stop force control / compliance mode
  void start_force_ctrl();
  void stop_force_ctrl();

  // getter; receiving commands
  jaco_position_t get_cart_pos();
  jaco_position_t get_ang_pos();

  jaco_position_t get_ang_command();
  jaco_position_t get_cart_command();
  jaco_position_t get_ang_vel();
  jaco_position_t get_cart_force();
  jaco_position_t get_ang_force();
  jaco_position_t get_ang_current();
  jaco_position_t get_ang_current_motor();
  jaco_sensor_info_t get_sensor_info();

  jaco_retract_mode_t get_status();

  // setter; sending basic commands
  void set_control_ang();
  void set_control_cart();
  void erase_trajectories();

  // joystick functionaliy
  void push_joystick_button(unsigned short id);
  void push_joystick_button(jaco_joystick_button_t &buttons);
  void move_joystick_axis(jaco_joystick_axis_t &axes);
  void move_joystick(jaco_joystick_t &state);
  void release_joystick();

  // arm movement
  void set_target(jaco_basic_traj_point_t &traj);
  void set_target_cart(float x, float y, float z, float euler_1, float euler_2, float euler_3, float finger_1, float finger_2, float finger_3);
  void set_target_cart(float coord[], float fingers[]);
  void set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, float finger_1, float finger_2, float finger_3);
  void set_target_ang(float joints[], float fingers[]);

 private:
  libusb_device_handle *__devh;
  boost::mutex          __lock;
  bool                  __auto_ctx;

  inline void _usb_header(usb_packet_t &p, unsigned short pid, unsigned short pquant, unsigned short cmdid, unsigned short cmdsize);
  inline int _usb_in(usb_packet_t &p, int &transferred);
  inline int _usb_out(usb_packet_t &p, int &transferred);

  // generic USB data transfer methods
  error_t _cmd_out_in(usb_packet_t &p);
  error_t _cmd_out(short cmdid);

  // Jaco specific commands
  error_t _get_cart_pos(jaco_position_t &pos);
  error_t _get_ang_pos(jaco_position_t &pos);

  error_t _get_ang_command(jaco_position_t &pos);
  error_t _get_cart_command(jaco_position_t &pos);
  error_t _get_ang_vel(jaco_position_t &pos);
  error_t _get_cart_force(jaco_position_t &pos);
  error_t _get_ang_force(jaco_position_t &pos);
  error_t _get_ang_current(jaco_position_t &pos);
  error_t _get_ang_current_motor(jaco_position_t &pos);
  error_t _get_sensor_info(jaco_sensor_info_t &info);

  error_t _send_basic_traj(jaco_basic_traj_point_t &traj);


};

} // end namespace KinDrv
#endif
