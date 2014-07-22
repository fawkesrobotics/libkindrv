
/***************************************************************************
 *  kindrv.cpp - KinDrv main cpp file for arm control
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

#include "kindrv.h"

#include <libusb.h>
#include <stdio.h>

#include <boost/thread/locks.hpp>

#define VENDOR_ID     0x22CD
#define PRODUCT_ID    0x0000

#define EP_IN         (2 | LIBUSB_ENDPOINT_IN)
#define EP_OUT        (2 | LIBUSB_ENDPOINT_OUT)
#define INTR_LENGTH   64
#define INTR_TIMEOUT  1000

#define CMD_CTRL_ANG                     47
#define CMD_CTRL_CART                    49
#define CMD_START_FORCE_CTRL             57
#define CMD_STOP_FORCE_CTRL              58
#define CMD_START_API_CTRL              302
#define CMD_STOP_API_CTRL               303

#define CMD_GET_CLIENT_INFO               1
#define CMD_GET_CART_POS                 44
#define CMD_GET_ANG_POS                  15
#define CMD_GET_CART_INFO               104
#define CMD_GET_ANG_INFO                105
#define CMD_GET_CART_COMMAND            106
#define CMD_GET_ANG_COMMAND             107
#define CMD_GET_CART_FORCE              108
#define CMD_GET_ANG_FORCE               109
#define CMD_GET_ANG_VEL                 114
#define CMD_GET_ANG_CURRENT             110
#define CMD_GET_ANG_CURRENT_MOTOR       113

#define CMD_GET_SENSOR_INFO             111
#define CMD_GET_ARM_INFO                200

#define CMD_ERASE_TRAJECTORIES          301
#define CMD_SEND_BASIC_TRAJ             308

#define CMD_JOYSTICK                    305

namespace KinDrv {

/** The libusb context. Set this so that it doesn't interfer with other contexts,
 * and such that including this lib would use the same context. */
static libusb_context *__ctx = NULL;
static libusb_device** __devices; // testing

/*
#define USB_CMD(ep,msg)         \
  (libusb_interrupt_transfer(__lusb_devh, ep, msg.data, INTR_LENGTH, &transferred, 1000))

#define USB_CMD_IN(msg)         (USB_CMD(EP_IN,msg))
#define USB_CMD_OUT(msg)        (USB_CMD(EP_OUT,msg))
#define USB_MSG(msg,pid,pquant,cmdid,cmdsize) { \
  memset(msg.data, 0, sizeof(msg));             \
  msg.header.IdPacket = pid;                    \
  msg.header.PacketQuantity = pquant;           \
  msg.header.CommandId = cmdid;                 \
  msg.header.CommandSize = cmdsize;           }

*/



/* /================================================\
 *   Public libusb-control methods
 * \================================================/ */
/** Initialize libusb.
 * This creates a libusb context which is used for the whole KinDrv library.
 * Although a context is always created implicitly when needed (e.g. when
 * creating a JacoArm instance), it is possible for the user to explicitly init
 * libusb.
 * Implicit creation also deletes the context when the "creator" is destroyed
 * (e.g. deleting the JacoArm instance), so this method can be used to sustain
 * the context until the end. That also means, that after an explicit 'init',
 * there should follow an explicit 'close' some time later.
 * @return Potential error (see enum error_t)
 */
error_t
init_usb()
{
  int r = libusb_init(&__ctx);
  if( r<0 ) {
    fprintf(stderr, "Error initializing libusb. libusb-error: %i", r);
    return ERROR_USB_INIT;
  }

  return ERROR_NONE;
}

/** Exit libusb.
 * This closes the explicitly created libusb context. It is not needed to call
 * this when 'init_usb()' has not been called before. However, calling this does
 * no harm (no exception throwing etc.).
 */
void
close_usb()
{
  libusb_exit(__ctx);
  __ctx = NULL;
}


void
list_devices(libusb_device **devices)
{
  libusb_device *dev;
  int i = 0;

  while ((dev = devices[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0) {
      fprintf(stderr, "failed to get device descriptor");
      return;
    }

    printf("idVendor:%04x  idProduct:%04x  SN:%02x (bus %d, device %d)\n",
    desc.idVendor, desc.idProduct, desc.iSerialNumber,
    libusb_get_bus_number(dev), libusb_get_device_address(dev));
  }
}

/** List available usb devices. */
void
list_devices()
{
  bool tmp_ctx = false;
  if( __ctx == NULL ) {
    // try creating a temporary context
    error_t err = init_usb();
    if( err != ERROR_NONE )
       throw KinDrvException(err, "Failed to initialize temporary libusb context");
    else
      tmp_ctx = true;
  }

  // Get devices
  ssize_t cnt;
  cnt = libusb_get_device_list(__ctx, &__devices);
  if( cnt<0 ) {
    fprintf( stderr, "Get_device_list error: %i \n", cnt);
  } else {
    printf("%i USB devices detected \n", cnt);

    list_devices(__devices);

    // Clear devices list
    libusb_free_device_list(__devices, /*auto-unref*/ true);
  }

  if( tmp_ctx )
    close_usb();
}


void
print_message(usb_packet_t &msg)
{
  usb_packet_header_t h = msg.header;
  float *b = msg.body;
  printf("h: %i  %i  %i  %i \n", h.id_packet, h.packet_quantity, h.command_id, h.command_size);
  printf("b: ");
  for(unsigned int i=0; i<2; ++i) {
    for(unsigned int j=0; j<7; ++j) {
      printf("%f   ", *b);
      ++b;
    }
    printf("\n   ");
  }
}


/* /================================================\
 *   Generic USB data transfer commands (private)
 * \================================================/ */

/** Incoming USB interrupt transfer. */
inline int
JacoArm::_usb_in(usb_packet_t &p, int &transferred)
{
  return libusb_interrupt_transfer(__devh, EP_IN, p.data, INTR_LENGTH, &transferred, INTR_TIMEOUT);
}

/** Outgoing USB interrupt transfer. */
inline int
JacoArm::_usb_out(usb_packet_t &p, int &transferred)
{
  return libusb_interrupt_transfer(__devh, EP_OUT, p.data, INTR_LENGTH, &transferred, INTR_TIMEOUT);
}

/** Set the usb_packet header. */
inline void
JacoArm::_usb_header(usb_packet_t &p, unsigned short pid, unsigned short pquant, unsigned short cmdid, unsigned short cmdsize)
{
  memset(p.data, 0, sizeof(p));
  p.header.id_packet = pid;
  p.header.packet_quantity = pquant;
  p.header.command_id = cmdid;
  p.header.command_size = cmdsize;
}

/** Perform an outgoing and then ingoing command.*/
error_t
JacoArm::_cmd_out_in(usb_packet_t &p)
{
  int r, transferred;
  unsigned short exp = p.header.command_id;

  boost::lock_guard<boost::mutex> lock(__lock);

  r = _usb_out(p, transferred);
  if( r < 0 )
    return ERROR_USB_INTERRUPT;
  else if( transferred < INTR_LENGTH )
    return ERROR_USB_SHORT_WRITE;

  r = _usb_in(p, transferred);
  if( r < 0 )
    return ERROR_USB_INTERRUPT;
  else if( transferred < INTR_LENGTH )
    return ERROR_USB_SHORT_READ;

  if( exp != p.header.command_id)
    return ERROR_CMD_ID_MISMATCH;

  return ERROR_NONE;
}

/** Just send a simple outgoing command.
 * This is for convenience, as some Kinova methods are just setters (like SetControlApi),
 * so one can simply pass the command-id to this method and it's done.
 */
error_t
JacoArm::_cmd_out(short cmd)
{
  int r, transferred;
  usb_packet_t p;
  _usb_header(p, 1, 1, cmd, 1);

  boost::lock_guard<boost::mutex> lock(__lock);

  r = _usb_out(p, transferred);
  if( r < 0 )
    return ERROR_USB_INTERRUPT;
  else if( transferred < INTR_LENGTH )
    return ERROR_USB_SHORT_WRITE;

  return ERROR_NONE;
}




/* /================================================\
 *   JacoArm
 * \================================================/ */
/** Constructor. */
JacoArm::JacoArm() :
  __devh( 0 ),
  __auto_ctx( 0 )
{
  if( __ctx == NULL ) {
    // initialize libusb
    error_t e = init_usb();
    if( e != ERROR_NONE )
      throw KinDrvException(e, "Abort creating JacoArm, libusb could not be initialized");
    __auto_ctx = true;
  }

  // Get device handle by vendorId and productId
  __devh = libusb_open_device_with_vid_pid(__ctx, VENDOR_ID, PRODUCT_ID);
  if( !__devh )
    throw KinDrvException("Failed getting usb-device-handle for JacoArm!" );

  // Claim usb interface
  if( libusb_claim_interface(__devh, 0) < 0 )
    throw KinDrvException("Could not claim usb interface 0!");
}

/** Destructor. */
JacoArm::~JacoArm()
{
  if( __devh != NULL ) {
    // need to relase interface and device-handler
    libusb_release_interface(__devh, 0);
    libusb_close(__devh);
  }
  if( __auto_ctx ) {
    // libusb context was created implicitly. so delete it now
    close_usb();
  }
}




/* /================================================\
 *   Jaco specific commands (private)
 * \================================================/ */
error_t
JacoArm::_get_cart_pos(jaco_position_t &pos)
{
  /*
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_CART_POS, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.position, p.body + 2, sizeof(pos.position));
    memcpy(pos.rotation, p.body + 7, sizeof(pos.rotation));
    memcpy(pos.finger_position, p.body + 10, sizeof(pos.finger_position));
  }
  */
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_CART_INFO, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.position, p.body, sizeof(pos.position));
    memcpy(pos.rotation, p.body + 3, sizeof(pos.rotation));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get cart position message: " << std::endl;
  //print_message(p);
  /*
  if( e != ERROR_NONE )
    return e;

  memcpy(pos.position, p.body + 2, sizeof(pos.position));
  memcpy(pos.rotation, p.body + 8, sizeof(pos.rotation));

  _usb_header(p, 1, 1, CMD_GET_CART_INFO, 1);
  e = _cmd_out_in(p);
  if( e == ERROR_NONE )
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  */

  return e;
}

error_t
JacoArm::_get_ang_pos(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_ANG_POS, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.joints, p.body, sizeof(pos.joints));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get ang position message: " << std::endl;
  //print_message(p);

  return e;
}


error_t
JacoArm::_get_ang_command(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_ANG_COMMAND, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.joints, p.body, sizeof(pos.joints));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get ang command message: " << std::endl;
  //print_message(p);

  return e;
}

error_t
JacoArm::_get_cart_command(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_CART_COMMAND, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.position, p.body, sizeof(pos.position));
    memcpy(pos.rotation, p.body + 3, sizeof(pos.rotation));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get cart command message: " << std::endl;
  //print_message(p);

  return e;
}

error_t
JacoArm::_get_ang_vel(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_ANG_VEL, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.joints, p.body, sizeof(pos.joints));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get ang velocity message: " << std::endl;
  //print_message(p);

  return e;
}

error_t
JacoArm::_get_cart_force(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_CART_FORCE, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.position, p.body, sizeof(pos.position));
    memcpy(pos.rotation, p.body + 3, sizeof(pos.rotation));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get cart force message: " << std::endl;
  //print_message(p);

  return e;
}

error_t
JacoArm::_get_ang_force(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_ANG_FORCE, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.joints, p.body, sizeof(pos.joints));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get ang force message: " << std::endl;
  //print_message(p);

  return e;
}

error_t
JacoArm::_get_ang_current(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_ANG_CURRENT, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.joints, p.body, sizeof(pos.joints));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get ang current message: " << std::endl;
  //print_message(p);

  return e;
}

error_t
JacoArm::_get_ang_current_motor(jaco_position_t &pos)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_ANG_CURRENT_MOTOR, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(pos.joints, p.body, sizeof(pos.joints));
    memcpy(pos.finger_position, p.body + 6, sizeof(pos.finger_position));
  }

  //std::cout << "get motor current message: " << std::endl;
  //print_message(p);

  return e;
}

error_t
JacoArm::_get_sensor_info(jaco_sensor_info_t &info)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_GET_SENSOR_INFO, 1);

  error_t e = _cmd_out_in(p);
  if( e == ERROR_NONE )
  {
    memcpy(&(info.voltage), p.body, sizeof(info.voltage));
    short offset = sizeof(info.voltage) / sizeof(float);
    memcpy(&(info.current), p.body + offset, sizeof(info.current));
    offset += sizeof(info.current) / sizeof(float);
    memcpy(info.acceleration, p.body + offset, sizeof(info.acceleration));
    offset += sizeof(info.acceleration) / sizeof(float);
    memcpy(info.joint_temperature, p.body + offset, sizeof(info.joint_temperature));
    offset += sizeof(info.joint_temperature) / sizeof(float);
    memcpy(info.finger_temperature, p.body + offset, sizeof(info.finger_temperature));
  }

  return e;
}

 error_t
JacoArm::_update_client_config()
{
  usb_packet_t p;
  error_t e;
  for( unsigned int i=1; i<=55; ++i ) {
    _usb_header(p, i, 1, CMD_GET_CLIENT_INFO, 1);
    e = _cmd_out_in(p);
    if( e != ERROR_NONE )
      return e;

    if( i<=2 )
      memcpy(__client_config.data+(i-1)*56, p.body, 56);
  }
}

error_t
JacoArm::_send_basic_traj(jaco_basic_traj_point_t &traj)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_SEND_BASIC_TRAJ, 48);
  memcpy(&(p.body), &traj, 48);

  return _cmd_out_in(p);
}


/* /================================================\
 *   Public JacoArm methods
 * \================================================/ */
/** Start/enable controlling the arm via API/USB.
 * The default is that a connected joystick takes over control as soon as activated.
 * It is recommended to use this command before sending other commands to the arm!
 */
void
JacoArm::start_api_ctrl()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_START_API_CTRL, 0);
  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not start API control! libusb error.");
}

/** Stop/disable controlling the arm via API/USB.
 * This method should only be used in very rare occasions, it 'might' break
 * some behaviour if the control is suddenly take away.
 * If you want the arm to stop moving, use the release_joystick() method. Remember
 * to start_api_ctrl() afterwards if you want to use the API again.
 */
void
JacoArm::stop_api_ctrl()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_STOP_API_CTRL, 0);
  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not stop API control! libusb error.");
}

/** Start/enable force control / compliant mode.
 */
void
JacoArm::start_force_ctrl()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_START_FORCE_CTRL, 0);
  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not start force/compliant control! libusb error.");
}

/** Stop/disable force control / compliant mode.
 */
void
JacoArm::stop_force_ctrl()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_STOP_FORCE_CTRL, 0);
  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not stop force/compliant control! libusb error.");
}

/** Set arm control type to angular control. */
void
JacoArm::set_control_ang()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_CTRL_ANG, 0);
  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not set angular control! libusb error.");
}

/** Set arm control type to cartesian control. */
void
JacoArm::set_control_cart()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_CTRL_CART, 1);
  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not set cartesian control! libusb error.");
}

/** Erase all queued trajectores. */
void
JacoArm::erase_trajectories()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_ERASE_TRAJECTORIES, 0);
  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not erase trajectories! libusb error.");
}

/** Get current cartesian position of Jaco arm.
 * Additionally, this is the method that can fetch joint values of the fingers.
 * Rememer, that at the moment the Jaco firmware can only provide updated cartesian
 * position if the arm is set to CartesianControl mode!
 * @return position struct that contains the current joint values
 */
jaco_position_t
JacoArm::get_cart_pos()
{
  jaco_position_t pos;
  error_t e = _get_cart_pos(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get cartesian position, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get cartesian position! libusb error.");

  return pos;
}

/** Get current angular position of Jaco arm.
 * @return position struct that contains the current joint values
 */
jaco_position_t
JacoArm::get_ang_pos()
{
  jaco_position_t pos;
  error_t e = _get_ang_pos(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get angular position, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get angular position! libusb error.");

  return pos;
}

/** Get current angular velocities of the arm.
 * @return universal position struct that contains the current joint velocities
 */
jaco_position_t
JacoArm::get_ang_vel()
{
  jaco_position_t pos;
  error_t e = _get_ang_vel(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get angular velocities, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get angular velocities! libusb error.");

  return pos;
}

/** Get current angular command of the arm.
 * @return universal position struct that contains the current joint command
 */
jaco_position_t
JacoArm::get_ang_command()
{
  jaco_position_t pos;
  error_t e = _get_ang_command(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get angular command, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get angular command! libusb error.");

  return pos;
}

/** Get current angular velocities of the arm.
 * @return universal position struct that contains the current joint velocities
 */
jaco_position_t
JacoArm::get_cart_command()
{
  jaco_position_t pos;
  error_t e = _get_cart_command(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get cartesian command, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get cartesian command! libusb error.");

  return pos;
}

/** Get current cartesian forces of the arm.
 * @return universal position struct that contains the current cartesian forces
 */
jaco_position_t
JacoArm::get_cart_force()
{
  jaco_position_t pos;
  error_t e = _get_cart_force(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get cartesian forces, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get cartesian forces! libusb error.");

  return pos;
}

/** Get current angular forces of the arm.
 * @return universal position struct that contains the current angular forces
 */
jaco_position_t
JacoArm::get_ang_force()
{
  jaco_position_t pos;
  error_t e = _get_ang_force(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get angular forces, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get angular forces! libusb error.");

  return pos;
}

/** Get currents that each actuator of the arm consumes on the main supply.
 * @return universal position struct that contains the joint currents
 */
jaco_position_t
JacoArm::get_ang_current()
{
  jaco_position_t pos;
  error_t e = _get_ang_current(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get joint currents, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get joint currents! libusb error.");

  return pos;
}

/** Get current readings on each actuator of the arm.
 * @return universal position struct that contains the joint currents
 */
jaco_position_t
JacoArm::get_ang_current_motor()
{
  jaco_position_t pos;
  error_t e = _get_ang_current(pos);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get joint motor currents, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get joint motor currents! libusb error.");

  return pos;
}

/** Get the voltage, current, acceleration, temperature sensor readings of the arm.
 * @return sensor info struct that contains the sensor values
 */
jaco_sensor_info_t
JacoArm::get_sensor_info()
{
  jaco_sensor_info_t info;
  error_t e = _get_sensor_info(info);
  if( e == ERROR_CMD_ID_MISMATCH )
    throw KinDrvException(e, "Could not get sensor readings, received packet with wrong CMD_ID.");
  else if( e != ERROR_NONE )
    throw KinDrvException(e, "Could not get sensor readings! libusb error.");

  return info;
}

/** Get the current client information of Jaco arm.
 * This method gets the client information (ID, SerialNumber,..) of the arm.
 * @param refresh False, if cached values should be used (they usually need to be read just once).
 *   True, if the values should be read from the arm (default).
 * @return The current client information
 */
jaco_client_config_t
JacoArm::get_client_config(bool refresh)
{
  if(refresh) {
    error_t e = _update_client_config();
    if( e!= ERROR_NONE )
      throw KinDrvException(e, "Could not get client config! libusb error.");
  }
  return __client_config;
}

/** Get the current retract mode of Jaco arm.
 * This method can be for instance be useful to decide when, which and how many joystick
 * buttons to simulate to move into/from HOME/RETRACT position.
 * @return the retract mode (check enum jaco_retract_mode)
 */
jaco_retract_mode_t
JacoArm::get_status()
{
  jaco_retract_mode_t mode = MODE_ERROR;

  usb_packet_t p;
  error_t e;
  for( unsigned int i=1; i<=19; ++i ) {
    _usb_header(p, i, 1, CMD_GET_ARM_INFO, 1);
    e = _cmd_out_in(p);
    if( e != ERROR_NONE )
      throw KinDrvException(e, "Could not get arm status! libusb error.");

    if( i==2 )
      memcpy(&mode, p.data+8+52, sizeof(mode));
  }

  return mode;
}

/** Simulate a push of a joystick button.
 * The button is held until the user calls a relase_joystick()!!
 * @param id The id of the joystick (from 0 to 15)
 */
void
JacoArm::push_joystick_button(unsigned short id)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_JOYSTICK, 56);

  unsigned short *buttons = (unsigned short*)p.body;
  buttons[id] = 1;

  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not push joystick button! libusb error.");
}

/** Simulate a push of multiple joystick buttons.
 * These buttons are "pushed" until the user calls a relase_joystick().
 * @param buttons The struct containing the values for each button
 */
void
JacoArm::push_joystick_button(jaco_joystick_button_t &buttons)
{
  jaco_joystick_t state;
  memset(&state, 0, sizeof(state));
  memcpy(state.button, buttons, 32);
  move_joystick(state);
}

/** Simulate joystick movement along the joystick axes.
 * This sets momentary values for the axes, so in order to stop the movement
 * you need to reset the values, or call release_joystick().
 * @param axes The struct containing the values for axes movement
 */
void
JacoArm::move_joystick_axis(jaco_joystick_axis_t &axes)
{
  jaco_joystick_t state;
  memset(&state, 0, sizeof(state));
  memcpy(&(state.axis), &axes, 24);
  move_joystick(state);
}

/** Simulate the global joystick state.
 * The joystick state is a combination of button values and axes state.
 * @param state The joystick state contatining button and axes values
 */
void
JacoArm::move_joystick(jaco_joystick_t &state)
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_JOYSTICK, 56);
  memcpy(&(p.body), &state, 56);

  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not send joystick command! libusb error.");
}

/** Simulate a release of a joystick button.
 * This automatically stops any arm movement, so this method is very convenient
 * as a kind of "stop"-command to the arm!
 */
void
JacoArm::release_joystick()
{
  usb_packet_t p;
  _usb_header(p, 1, 1, CMD_JOYSTICK, 56);

  error_t e = _cmd_out_in(p);
  if( e != ERROR_NONE )
    throw KinDrvException("Could not release joystick! libusb error.");
}

/** Move the arm to a given position (trajectory point).
 * @param traj The trajectory point that the arm should move to (check jaco_basic_traj_point_t)
 */
void
JacoArm::set_target(jaco_basic_traj_point_t &traj)
{
  error_t e = _send_basic_traj(traj);
  if( e != ERROR_NONE  )
    throw KinDrvException("Could not send basic trajectory! libusb error.");
}

/** Move the arm to a given position (trajectory point).
 * @param coord Array of 6 floats containing translation and rotation
 * @param fingers Array of 3 floats containing finger values
 */
void
JacoArm::set_target_cart(float coord[], float fingers[])
{
  jaco_basic_traj_point_t traj;
  memcpy(&traj.target, coord, 24);
  memcpy(traj.target.finger_position, fingers, 12);
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_CARTESIAN;

  set_target(traj);
}

/** Move the arm to a given position (trajectory point).
 * @param joints Array of 6 floats containing angular values for each joint
 * @param fingers Array of 3 floats containing finger values
 */
void
JacoArm::set_target_ang(float joints[], float fingers[])
{
  jaco_basic_traj_point_t traj;
  memcpy(&traj.target, joints, 24);
  memcpy(traj.target.finger_position, fingers, 12);
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_ANGULAR;

  set_target(traj);
}

/** Move the arm to a given position (trajectory point).
 * @param x Translation on x-axis
 * @param y Translation on y-axis
 * @param z Translation on z-axis
 * @param euler_1 1st Euler rotation
 * @param euler_2 2nd Euler rotation
 * @param euler_3 3rd Euler rotation
 * @param finger_1 Value of 1st finger
 * @param finger_2 Value of 2nd finger
 * @param finger_3 Value of 3rd finger
 */
void
JacoArm::set_target_cart(float x, float y, float z, float euler_1, float euler_2, float euler_3, float finger_1, float finger_2, float finger_3)
{
  jaco_basic_traj_point_t traj;
  traj.target.position[0] = x;
  traj.target.position[1] = y;
  traj.target.position[2] = z;
  traj.target.rotation[0] = euler_1;
  traj.target.rotation[1] = euler_2;
  traj.target.rotation[2] = euler_3;
  traj.target.finger_position[0] = finger_1;
  traj.target.finger_position[1] = finger_2;
  traj.target.finger_position[2] = finger_3;
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_CARTESIAN;

  set_target(traj);
}

/** Move the arm to a given position (trajectory point).
 * @param j1 Angular value of joint 1
 * @param j2 Angular value of joint 2
 * @param j3 Angular value of joint 3
 * @param j4 Angular value of joint 4
 * @param j5 Angular value of joint 5
 * @param j6 Angular value of joint 6
 * @param finger_1 Value of 1st finger
 * @param finger_2 Value of 2nd finger
 * @param finger_3 Value of 3rd finger
 */
void
JacoArm::set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, float finger_1, float finger_2, float finger_3)
{
  jaco_basic_traj_point_t traj;
  traj.target.joints[0] = j1;
  traj.target.joints[1] = j2;
  traj.target.joints[2] = j3;
  traj.target.joints[3] = j4;
  traj.target.joints[4] = j5;
  traj.target.joints[5] = j6;
  traj.target.finger_position[0] = finger_1;
  traj.target.finger_position[1] = finger_2;
  traj.target.finger_position[2] = finger_3;
  traj.time_delay = 0;
  traj.hand_mode = MODE_POSITION;
  traj.pos_type = POSITION_ANGULAR;

  set_target(traj);
}
} // end namespace KinDrv
