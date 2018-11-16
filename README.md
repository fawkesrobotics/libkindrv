libkindrv
====================

Open source library with libusb-based drivers to control robotic arms by Kinova.


Installing:
====================

## Precompiled packages

If you are on Fedora, you can directly install libkindrv:

    # dnf install libkindrv


If you want to develop your application using libkindrv, you should also install
`libkindrv-devel`.

## Building from source
The installation procedure is based on cmake and contains nothing
more than the usual procedure:

  # 1. Make a "build" directory inside libkindrv directory

    mkdir build

  # 2. Run cmake from inside the "build" directory

    cd build
    cmake ..

  # 3. Build the kindrv library

    make

  # 4. Install the kindrv library (may need root privileges)

    make install

  # 5. [Optional] Remove "build" directory. Skip this step if you want to allow fast uninstalling.

    cd ..
    rm -rf build


Uninstalling:
====================
In case you removed the "build" directory, repeat installation steps #1 and #2.
Simply execute the following from inside the "build" directory (may need root privileges) :

    make uninstall



Documentation:
====================
By default, documentation is built when running "make", if Doxygen is available.
You can find it in the "doc" directory inside the "build" directory.
In order to disable auto-build of documentation, pass "-DBUILD_DOC=OFF" to cmake.
Either way, you can always build the documentation by running

    make apidoc

from inside the "build" directory.



Other notes:
====================
Of course the default cmake options are availabe, as well as
using the GUI version of cmake in installation step #2 with

    cd build
    ccmake ..

