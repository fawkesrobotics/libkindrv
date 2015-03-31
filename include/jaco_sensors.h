
/***************************************************************************
 *  jaco_sensors.h - KinDrv header file for Jaco tactile sensors
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

#ifndef _JACO_SENSORS_H
#define _JACO_SENSORS_H

#include "types.h"
#include "exception.h"

#include <boost/asio.hpp>
#include <vector>

namespace KinDrv {

/// \brief Main JacoSensors class
class JacoSensors
{
 public:
  JacoSensors(const char* dev_path = NULL);
  virtual ~JacoSensors();

  jaco_sensor_pressure_t get_pressure(unsigned short id);
  unsigned short get_vibration(unsigned short id);

  // get_pressures();
  // get_vibrations();

 private:
  bool _read_until(char c);
  bool _read(char* buf, size_t size);

  void _read(char* req, size_t req_size, char* buf, size_t read_size);
  void _init();

  void _read_callback(boost::system::error_code error, size_t bytes_transferred);
  void _timer_callback(boost::system::error_code error);

  boost::asio::io_service       __io;
  boost::asio::serial_port      __port;
  boost::asio::deadline_timer   __timer;
  boost::asio::streambuf        __buf;

  boost::system::error_code     __error;
  size_t                        __bytes_transferred;

  std::vector<jaco_sensor_pressure_t> __init_pressure;
};

} // end namespace KinDrv
#endif
