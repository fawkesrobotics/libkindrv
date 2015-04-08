
/***************************************************************************
 *  types.h - KinDrv common typedef structs and enums
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
 *  libkindrv is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with libkindrv.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __KINDRV_TYPES_H_
#define __KINDRV_TYPES_H_

namespace KinDrv {

typedef enum {
  ERROR_NONE            = 0,
  ERROR_UNKNOWN         = 1,

  ERROR_USB_INIT        = 101,
  ERROR_USB_DEVICE      = 102,
  ERROR_USB_INTERFACE   = 103,

  ERROR_USB_INTERRUPT   = 201,
  ERROR_USB_SHORT_WRITE = 202,
  ERROR_USB_SHORT_READ  = 203,

  ERROR_CMD_ID_MISMATCH = 301,
} error_t;

typedef enum {
  POSITION_NO_MOVEMENT  = 0, /**< No Movements. */
  POSITION_CARTESIAN    = 1, /**< Cartesian Position Control. */
  POSITION_ANGULAR      = 2, /**< Angular Position Control. */
  SPEED_CARTESIAN       = 7, /**< Cartesian speed control. */
  SPEED_ANGULAR         = 8, /**< Angular speed control. */
  TIME_DELAY            = 12 /**< Insert Delay. */
} jaco_position_type_t;

typedef enum {
  NO_MOVEMENT   = 0, /**< Finger movements disabled. */
  MODE_POSITION = 1, /**< Finger position control. */
  MODE_SPEED    = 2  /**< Finger Speed Control. */
} jaco_hand_mode_t;

typedef enum {
  MODE_NORMAL_TO_READY  = 0, /**< Transition mode indicating that Jaco is in NORMAL state and waiting to go in READY state. */
  MODE_READY_STANDBY    = 1, /**< Transition mode indicating that Jaco is in READY state and waiting to go in STANDBY state. */
  MODE_READY_TO_RETRACT = 2, /**< Transition mode indicating that Jaco is in READY state and waiting to go in RETRACTED state. */
  MODE_RETRACT_STANDBY  = 3, /**< Transition mode indicating that Jaco is in RETRACT state and waiting to go in STANDBY state. */
  MODE_RETRACT_TO_READY = 4, /**< Transition mode indicating that Jaco is in RETRACT state and waiting to go in READY state. */
  MODE_NORMAL           = 5, /**< Transition mode indicating that Jaco is in NORMAL state. */
  MODE_NOINIT           = 6, /**< Transition mode indicating that Jaco is in NO INIT state and waiting to go in READY state. */
  MODE_ERROR            = 25000 /**< This value indicate an error. Most of the time, it is because you received a value that is not part of the enum. */
} jaco_retract_mode_t;

typedef enum {
  LATERALITY_LEFT, /**< Left hand. */
  LATERALITY_RIGHT /**< Right hand. */
} jaco_laterality_t;


/// \brief Jaco universal position struct.
/** Contains 9 floats. The last 3 are finger positions. The first 6 can be either
 * angular joint values, or cartesian position and rotation values. */
typedef struct {
  union {
    float joints[6];        /**< Reading the position as an array of joint values. */
    struct {
      float position[3];    /**< Reading the position as arrays of position(=this) and rotation values. */
      float rotation[3];    /**< Reading the position as arrays of position and rotation(=this) values. */
    };
  };
  float finger_position[3]; /**< Array containing the three finger positions. */
} jaco_position_t; // 36 bytes

/// \brief Basic trajectory struct
typedef struct {
  jaco_position_t target;        /**< The position of this trajectory point. */
  float time_delay;              /**< Time delay. */
  jaco_hand_mode_t hand_mode;    /**< The hand mode, defines how target.finger_position field affects the fingers. */
  jaco_position_type_t pos_type; /**< The position type, defines what kind of position this trajectory point is. */
} jaco_basic_traj_point_t;

/// \brief Simulation of joystick buttons (1 for pressed, 0 for released). Make sure to initialze with 0s!
typedef unsigned short jaco_joystick_button_t[16];

/// \brief Struct for joystick axis movement. Make sure to initialze with 0s!
typedef struct {
  union {
    float axis[6];      /**< Representing all movement values in one array. */
    struct {
      float trans_lr;    /**< (Translation Mode) Move stick +left -right. Left/Right translation of arm. */
      float trans_fb;    /**< (Translation Mode) Move stick +back -forth. Back/Forth translation to arm. */
      float trans_rot;   /**< (Translation Mode) Rotate stick +cw -ccw. Up/Down translation of arm. */
      float wrist_fb;    /**< (Wrist Mode) Move stick +forth -back. Up/Down inclination of wrist. */
      float wrist_lr;    /**< (Wrist Mode) Move stick +right -left. Forth/Back inclination of wrist. */
      float wrist_rot;   /**< (Wrist Mode) Rotate stick +cw -ccw. Ccw/cw rotation around wrist. */
    };
  };
} jaco_joystick_axis_t;

/// \brief Each joystick action (button,axis) as a whole is represented by one of this struct. Make sure to initialze with 0s!
typedef struct {
  jaco_joystick_button_t button; /**< Simulated buttons. */
  jaco_joystick_axis_t   axis;   /**< Simulated axes. */
} jaco_joystick_t;

/// \brief Struct that holds values of various sensors of the robot
typedef struct {
  float voltage;                /**< The main power supply voltage. (24 V) Unit is V. */
  float current;                /**< Te main power supply's current. Unit is A. */
  float acceleration[3];        /**< The value read by the acceleration sensors on the X, Y, Z axes, respectively. Unit is G.   */
  float joint_temperature[6];   /**< The value read by the temperature sensor on each actuator. Unit is C°.*/
  float finger_temperature[3];  /**< The value read by the temperature sensor on each finger. Unit is C°.*/
} jaco_sensor_info_t;

/// \brief User position struct.
typedef struct {
  jaco_position_type_t pos_type;  /**< The position type. */
  float delay;                    /**< Delay for the trajectory. */
  jaco_position_t cartesian_pos;  /**< The position in cartesian coordinates (position + rotation). */
  jaco_position_t angular_pos;    /**< The position in angular joint values. */

  jaco_hand_mode_t hand_mode;     /**< The hand mode. */
  float finger_position[3];       /**< Array containing the three finger positions. */
} jaco_user_position_t; // 96 bytes

/// \brief Struct containing the client information and configuration
typedef struct {
  union {
    unsigned char data[112]; // raw data
    struct {
      // client information; 100 bytes
      char id[20];
      char name[20];
      char organization[20];
      char sn[20];
      char model_no[20];

      char temp_buffer[12]; // just to make the total size a multiple of 56
    };
  };
/*

  jaco_laterality_t laterality;

  // thresholds; 20 bytes
  float max_linear_speed;
  float max_angular_speed;
  float max_linear_acc;
  float max_angular_acc;
  float max_force;

  float sensibility;
  float drinking_height;

  // retract settings; 1932 bytes
  int complex_retract_active;
  float retracted_position_angle;
  int retracted_position_count;
  jaco_user_position_t retract_positions[20]; // 1920 bytes

  float drinking_distance;
  int fingers_2and3_inverted;
  float drinking_lenght;

  int reset_at_retract;

  int enable_flash_error_log;
  int enable_flash_position_log;
*/
} jaco_client_config_t; //2088 bytes


/// \brief USB packet header struct. All USB packets must have this header structure.
typedef struct {
  unsigned short id_packet;       /**< The packet id. */
  unsigned short packet_quantity; /**< Total number of packets, that are transmitted for this command. */
  unsigned short command_id;      /**< The command id. */
  unsigned short command_size;    /**< The total size of the command data (in bytes) that is transmitted. */
} usb_packet_header_t;

/// \brief USB packet struct. All USB I/O is one of this.
typedef struct {
  union {
    unsigned char data[64];       /**< Access the whole 64 bytes of raw data. */
    struct {
      usb_packet_header_t header; /**<  8 byte header information. */
      float body[14];             /**< 56 byte body, containing the actual data. */
    };
  };
} usb_packet_t;


} // end namespace KinDrv
#endif
