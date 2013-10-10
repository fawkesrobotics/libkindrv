
/***************************************************************************
 *  exception.h - KinDrv exceptions
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

#ifndef _EXCEPTION_H
#define _EXCEPTION_H

#include "types.h"

#include <exception>

namespace KinDrv {

/// \brief Exception that is thrown by this Api
class KinDrvException : public std::exception
{
 public:
  KinDrvException() throw();
  KinDrvException(const char *msg) throw();
  KinDrvException(error_t err, const char *msg) throw();
  virtual ~KinDrvException() throw();

  const char* what() const throw();
  const error_t error() const throw();

 private:
  char *__msg;
  error_t __err;
};

} // end namespace KinDrv
#endif