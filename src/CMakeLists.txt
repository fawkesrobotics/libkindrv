cmake_minimum_required (VERSION 2.6)

# include directory containing header files
include_directories (${libkindrv_SOURCE_DIR}/include)

# set as shared library
add_library (kindrv SHARED kindrv.cpp exception.cpp)

set_target_properties(
  kindrv
  PROPERTIES
    SOVERSION ${libkindrv_VERSION_MAJOR}
    VERSION ${libkindrv_VERSION}
)

# link external libraries
target_link_libraries (kindrv ${LIBUSB_1_LIBRARIES})
target_link_libraries (kindrv ${Boost_LIBRARIES})

# set the install targets
install (TARGETS kindrv LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")

file (GLOB kindrv_header_files ${libkindrv_SOURCE_DIR}/include/*.h)
install (FILES ${kindrv_header_files} DESTINATION include/libkindrv)
