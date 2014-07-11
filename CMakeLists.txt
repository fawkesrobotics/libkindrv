cmake_minimum_required (VERSION 2.6)
project (libkindrv)

# The version number.
set (libkindrv_VERSION_MAJOR 0)
set (libkindrv_VERSION_MINOR 1)
set (libkindrv_VERSION_PATCH 0)
set (libkindrv_VERSION ${libkindrv_VERSION_MAJOR}.${libkindrv_VERSION_MINOR}.${libkindrv_VERSION_PATCH})

# add custom CMAKE_MODULE_PATH for our provided modules
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# detect correct lib-suffix
include (libkindrv_detect_lib_suffix)

# Compile examples?
#option (BUILD_EXAMPLES "Build examples?" OFF)

# find libusb library
find_package (libusb-1.0 REQUIRED)
include_directories (${LIBUSB_1_INCLUDE_DIRS})

# find boost-system
find_package (Boost REQUIRED COMPONENTS system)
include_directories (${Boost_INCLUDE_DIRS})

# add subdirectory containing source files
add_subdirectory (src)

# configure and install pkg-config file
configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/kindrv.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/libkindrv.pc" @ONLY)
install (FILES "${CMAKE_CURRENT_BINARY_DIR}/libkindrv.pc" DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

# install udev-rule
install (FILES "10-libkindrv.rules" DESTINATION "/etc/udev/rules.d")

# uninstall target
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
