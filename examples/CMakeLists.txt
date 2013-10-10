#examples
cmake_minimum_required (VERSION 2.6)
project (libkindrv_exampels)

# LIBKINDRV entries should only be shown in advanced view
mark_as_advanced(LIBKINDRV_LIB LIBKINDRV_INCLUDE_DIR)

# find installed kindrv library and include dir
set (LIBKINDRV_FOUND FALSE CACHE STRING "FALSE" FORCE)
find_library (LIBKINDRV_LIB kindrv)
find_path (LIBKINDRV_INCLUDE_DIR kindrv.h
  PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
  PATH_SUFFIXES
    libkindrv
)
if (LIBKINDRV_LIB AND LIBKINDRV_INCLUDE_DIR)
  set (LIBKINDRV_FOUND TRUE CACHE STRING "TRUE" FORCE)
endif (LIBKINDRV_LIB AND LIBKINDRV_INCLUDE_DIR)

if (LIBKINDRV_FOUND)
  include_directories (${LIBKINDRV_INCLUDE_DIR})
else (LIBKINDRV_FOUND)
  message(FATAL_ERROR "Could not find libkindrv!")
endif (LIBKINDRV_FOUND)

add_executable (libkindrv_ex_simple ex_simple.cpp)
add_executable (libkindrv_ex_ctrl ex_ctrl.cpp)

target_link_libraries (libkindrv_ex_simple ${LIBKINDRV_LIB})
target_link_libraries (libkindrv_ex_ctrl ${LIBKINDRV_LIB})
