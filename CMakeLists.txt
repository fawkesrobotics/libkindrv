cmake_minimum_required (VERSION 2.6)
project (libkindrv)

# The version number.
set (libkindrv_VERSION_MAJOR 0)
set (libkindrv_VERSION_MINOR 1)
set (libkindrv_VERSION_PATCH 0)
set (libkindrv_VERSION ${libkindrv_VERSION_MAJOR}.${libkindrv_VERSION_MINOR}.${libkindrv_VERSION_PATCH})

# Compile examples?
#option (BUILD_EXAMPLES "Build examples?" OFF)

# find libusb library
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
find_package (libusb-1.0 REQUIRED)
include_directories (${LIBUSB_1_INCLUDE_DIRS})

# set as shared library
add_library (kindrv SHARED kindrv.cpp exception.cpp)

# link external libraries
target_link_libraries (kindrv ${LIBUSB_1_LIBRARIES})

# set the install targets
install (TARGETS kindrv DESTINATION lib)
install (FILES kindrv.h types.h exception.h DESTINATION include/libkindrv)

# configure and install pkg-config file
configure_file ("${CMAKE_SOURCE_DIR}/kindrv.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/libkindrv.pc" @ONLY)
install (FILES "${CMAKE_CURRENT_BINARY_DIR}/libkindrv.pc" DESTINATION lib/pkgconfig)