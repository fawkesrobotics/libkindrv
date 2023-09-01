cmake_minimum_required (VERSION 3.6)
project (libkindrv)

# The version number.
set (libkindrv_VERSION_MAJOR 1)
set (libkindrv_VERSION_MINOR 0)
set (libkindrv_VERSION_PATCH 0)
set (libkindrv_VERSION ${libkindrv_VERSION_MAJOR}.${libkindrv_VERSION_MINOR}.${libkindrv_VERSION_PATCH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

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
option (INSTALL_UDEV "Install udev-rule? Needs root privileges, as it writes to '/etc' directory." ON)
set (UDEV_INSTALL_DIR "/etc/udev/rules.d" CACHE STRING "Destination for udev rules")
if (INSTALL_UDEV)
  install (FILES "10-libkindrv.rules" DESTINATION ${UDEV_INSTALL_DIR} OPTIONAL)
endif()

# apidoc target
find_package (Doxygen)
if (DOXYGEN_FOUND)
  option (BUILD_DOC "Build documentation files? If 'OFF', you have to call 'make apidoc' separately" ON)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in" "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile" @ONLY)
  if (BUILD_DOC)
    set (API_ALL "ALL")
    install (DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/doc/html" DESTINATION "share/doc/libkindrv")
  endif()
  add_custom_target(apidoc ${API_ALL}
    COMMAND "${DOXYGEN_EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
    COMMENT "Building API Documentation..."
  )
else ()
  message (WARNING "Doxygen not found! Cannot build documentation ")
endif()

# uninstall target
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
