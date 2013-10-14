
/***************************************************************************
 *  exception.cpp - KinDrv exceptions
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

#include "exception.h"

#include <string.h>

namespace KinDrv {

/** Constructor.
 * Constructs a new unknown exception with default parameters
 * (type is ERROR_UNKNOWN, with "Unknown exception" as the message text).
 */
KinDrvException::KinDrvException() throw()
{
 __err = ERROR_UNKNOWN;
 __msg = strdup("Unknown exception");
}

/** Constructor.
 * Constructs a new unknown exception (type is ERROR_UNKNOWN).
 * The error message can be set manually.
 * @param msg The message text of the exception.
 */
KinDrvException::KinDrvException(const char *msg) throw()
{
 __err = ERROR_UNKNOWN;
 __msg = strdup(msg);
}

/** Constructor.
 * Constructs a new exception, type and message text can be set manually.
 * @param err The error type of this exception.
 * @param msg The message text of the exception.
 */
KinDrvException::KinDrvException(error_t err, const char *msg) throw()
{
 __err = err;
 __msg = strdup(msg);
}

/** Destructor. */
KinDrvException::~KinDrvException() throw()
{}

/** Get the error message.
 * @return The error message of the exception.
 */
const char*
KinDrvException::what() const throw()
{
  return __msg;
}

/** Get the error type.
 * @return The error type of the exception.
 */
const error_t
KinDrvException::error() const throw()
{
 return __err;
}

} // end namespace KinDrv