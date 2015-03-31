
/***************************************************************************
*  jaco_sensors.cpp - KinDrv main cpp for Jaco tactile sensors
 *
 *  Created: Tue Mar 10 13:38:00 2015
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

#include "jaco_sensors.h"
#include <boost/bind.hpp>

#define DEV_PATH "/dev/jacoSensors"

#define SENSORS          7
#define PRESSURE_AREAS  12

#define MSG_ID   0xFA

#define PRESSURE_IN_SIZE  (2 + (2*PRESSURE_AREAS))
#define VIBRATION_IN_SIZE (2 +  2)

#define INIT_READS                5 // number of readings until considered "initialized"
#define INIT_TRIES                5 // number of tries to initialize a sensor
#define NORMALIZE_IGNORE_MAX     10 // max value to be ignored when normalizing
#define TIMEOUT_READ            500 // asio read timeout in ms

using namespace boost::asio;

namespace KinDrv {

// static CRC16 table for the sensor-id's 1-7
const unsigned short CRC[8] = {0x1043, 0xD082, 0xD1C2, 0x1103, 0xD342, 0x1383, 0x12C3, 0xD202};

///\brief structure for incoming "packet" with pressure data
typedef struct {
  union {
    char data[PRESSURE_IN_SIZE];
    struct {
      char msg_id;
      char sensor_id;
      unsigned short area[PRESSURE_AREAS];
    };
  };
} pressure_in_t;

///\brief structure for incoming "packet" with vibration data
typedef struct {
  union {
    char data[VIBRATION_IN_SIZE];
    struct {
      char msg_id;
      char sensor_id;
      unsigned short freq;
    };
  };
} vibration_in_t;


/* /================================================\
 *   Methods for more convenient usage of boost::asio
 *   asynchronous reads with timeouts
 * \================================================/ */
/** Callback method for when serial read is finished.
 * The error code tells if it was successful or not.
 */
void
JacoSensors::_read_callback(boost::system::error_code error, size_t bytes_transferred)
{
  __error = error;
  __bytes_transferred = bytes_transferred;

  // Cancel the timer
  __timer.cancel();
}

/** Callback method for the timeout.
 * The error code tells if it timed out or was aborted.
 */
void
JacoSensors::_timer_callback(boost::system::error_code error)
{
  if( error ) {
    // timer has been manually cancelled. All good
    return;
  } else {
    // timed out! Cancel the read operation with an error
    __port.cancel();
  }
}

/** Convenience wrapper method to perform a read_until(),
 * which reads the port until the given character is read,
 * or a timeout fires. */
bool
JacoSensors::_read_until(char c)
{
  // reset io service
  __io.reset();

  // read until we find desired char
  //printf("_read_until %.2hhX \n", c);
  async_read_until(__port, __buf, c, boost::bind(
                                       &JacoSensors::_read_callback,
                                       this,
                                       placeholders::error,
                                       placeholders::bytes_transferred
                                     ));

  // Setup a deadline time to implement our timeout.
  __timer.expires_from_now(boost::posix_time::milliseconds(TIMEOUT_READ));
  __timer.async_wait(boost::bind(
                       &JacoSensors::_timer_callback,
                       this,
                       placeholders::error
                     ));

  // run the io service
  __io.run();

  // at this point, one of the callbacks has fired and the port is not reading anymore!
  if( __error ) {
    //printf("read_until: have some read error: %s \n", __error.message().c_str());
    return false;
  }

  // all good
  return true;
}

/** Convenience wrapper method to perform a read(),
 * which reads until the given buffer has been filled,
 * or a timeout fires. */
bool
JacoSensors::_read(char* buf, size_t size)
{
  // reset io service
  __io.reset();

  if( size==0 )
    return true;

  // read until buffer is filled
  //printf("read %lu bytes \n", size);
  async_read(__port, buffer(buf, size), boost::bind(
                                          &JacoSensors::_read_callback,
                                          this,
                                          placeholders::error,
                                          placeholders::bytes_transferred
                                        ));

  // Setup a deadline time to implement our timeout.
  __timer.expires_from_now(boost::posix_time::milliseconds(TIMEOUT_READ));
  __timer.async_wait(boost::bind(
                       &JacoSensors::_timer_callback,
                       this,
                       placeholders::error
                     ));

  // run the io service
  __io.run();

  // at this point, one of the callbacks has fired and the port is not reading anymore!
  if( __error ) {
    //printf("read: have some read error: %s \n", __error.message().c_str());
    return false;
  }

  // all good
  return true;
}


/* /================================================\
 *   JacoSensors private methods
 * \================================================/ */
/** Request and read on the serial port.
 *
 * @param req Pointer to beginning of buffer which contains the request
 * @param req_size Data size of the request buffer
 * @param buf Pointer to beginning of buffer into which the data is read
 *            from the serial port
 * @param read_size Numer of bytes that we want to read from the serial port
 */
void
JacoSensors::_read(char* req, size_t req_size, char* buf, size_t read_size)
{
  // send request to port
  write(__port, buffer(req, req_size));

  // read incoming data until we find start of incoming message
  if( !_read_until(MSG_ID) )
    throw KinDrvException(ERROR_SERIAL_READ, "Error detecting start of incoming message.");

  // extract all bytes before MSG_ID
  std::istream is(&__buf);
  if( __bytes_transferred > 1 )
    is.ignore(__bytes_transferred - 1);

  // copy valid read data
  int valid_buf_size = __buf.size() - __bytes_transferred + 1;
  if( valid_buf_size > (int)read_size )
    throw KinDrvException(ERROR_SERIAL_READ, "More incoming data than expected.");
  is.read(buf, valid_buf_size);

  // read rest of the incoming message
  if( !_read(buf + valid_buf_size, read_size - valid_buf_size) )
    throw KinDrvException(ERROR_SERIAL_READ, "Error receiving remaining data from device.");

  // byte-swapping of readings
  // skip first two bytes, which are msg_id and sensor_id
  for(unsigned short i=2; i<read_size; i+=2) {
    //area.raw[i] = ((in.area[i] & 0xFF00) >> 8) | ((in.area[i] & 0x00FF) << 8);
    char tmp = buf[i];
    buf[i]   =  buf[i+1];
    buf[i+1] = tmp;
  }
}

/** Initialize sensors.
 * This reads the current sensor pressure values and uses them as initial values.
 * The normalized values from jaco_sensor_pressure_t uses these intial values
 * for comparison.
 */
void
JacoSensors::_init()
{
  __init_pressure.clear();
  __init_pressure.resize(SENSORS);

  for(unsigned short i=0; i<INIT_READS; ++i) {
    for(unsigned short id=0; id<SENSORS; ++id) {
      //printf("_init: round %i , sensor %i\n", i+1, id+1);
      for(unsigned short tries=0; tries<INIT_TRIES; ++tries) {
        try{
          __init_pressure.at(id)=get_pressure(id);
          break;
        } catch(KinDrvException &e) {
          //printf("_init: error: %s. Retrying \n", e.what());
        }
      }

      usleep(100e3);
    }
  }
}

/* /================================================\
 *   JacoSensors public methods
 * \================================================/ */
/** Constructor.
 * Connects to tactile sensors on the given path.
 * @param dev_path The path to the device (e.g. "/dev/ttyUSB0").
 *  If the udev rules have been installed with libkindrv, this
 *  can be ignored, libkindrv will use the installed symlink
 *  to "/dev/jacoSensors".
 */
JacoSensors::JacoSensors(const char* dev_path)
 : __port( __io ),
   __timer( __io )
{
  try {
    if( !dev_path )
      __port.open(DEV_PATH);
    else
      __port.open(dev_path);
  } catch(boost::system::system_error& e) {
    //printf("Error: %s \n", e.what());
    throw KinDrvException(ERROR_SERIAL_ACCESS, "Could not connect to sensors. Please check the device path and privileges.");
  }

  // use recommended serial port settings
  __port.set_option(serial_port_base::baud_rate(460800));
  __port.set_option(serial_port_base::parity(serial_port_base::parity::none));
  __port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  __port.set_option(serial_port_base::character_size(8));
  __port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

  // initialize sensors
  _init();
}

/** Destructor. */
JacoSensors::~JacoSensors()
{
  __port.close();
}

/** Get sensed pressure on a sensor..
 * Check the documentation on jaco_sensor_pressure_t struct for information
 * on the data it holds.
 * @param id The id of the sensor (0-7)
 * @return The struct that contains the raw and normalized pressure data.
 */
jaco_sensor_pressure_t
JacoSensors::get_pressure(unsigned short id)
{
  // send request to port
  char req[4];
  req[0]=MSG_ID;
  req[1]=(id+1) & 0xFF;                     //id's start with 1 in the sensors
  req[2]=(char)( CRC[id+1] & 0x00FF);       //CRC16 hight byte
  req[3]=(char)((CRC[id+1] & 0xFF00) >> 8); //CRC16 low byte

  pressure_in_t in;
  _read(req, sizeof(req), in.data, sizeof(in.data));

  // prepare data for returning
  jaco_sensor_pressure_t press;
  memcpy(press.raw, in.area, sizeof(in.area));

  // normalize values
  for(unsigned short i=0; i<PRESSURE_AREAS; ++i) {
    press.normalized[i] = press.raw[i] > __init_pressure[id].raw[i]
                          ? press.raw[i] - __init_pressure[id].raw[i]
                          : 0;
    if (press.normalized[i] <= NORMALIZE_IGNORE_MAX) {
      press.normalized[i] = 0;
    }
  }

  return press;
}

/** Get sensed vibration on the sensor surface.
 * @param id The id of the sensor (0-7)
 * @return The vibration in Hz
 */
unsigned short
JacoSensors::get_vibration(unsigned short id)
{
  // send request to port
  char req[2];
  req[0]=MSG_ID;
  req[1]=(id+1 + 128) & 0xFF; //id's start with 1 in the sensors

  vibration_in_t in;
  _read(req, sizeof(req), in.data, sizeof(in.data));

  return in.freq;
}

} // end namespace KinDrv
