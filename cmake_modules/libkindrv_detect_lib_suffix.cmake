# Use GNUInstallDirs for paths, e.g. proper lib-suffix
include (GNUInstallDirs OPTIONAL RESULT_VARIABLE GNU_DIRS_SET)
if (GNU_DIRS_SET)
  if (DEFINED LIB_SUFFIX)
    set (CMAKE_INSTALL_LIBDIR "lib${LIB_SUFFIX}")
    message (STATUS "LIB_SUFFIX is manually set, install to ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}, overriding auto-detection")
  else ()
    message (STATUS "LIB_SUFFIX auto-detected, install to ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
  endif ()

else (GNU_DIRS_SET)
  set (LIB_SUFFIX "" CACHE STRING "Set lib-suffix. Should either be empty (for 'lib') or 64 (for 'lib64')")
  set (CMAKE_INSTALL_LIBDIR "lib${LIB_SUFFIX}")
  message (WARNING "Can't automatically detect lib-suffix, 'GNUInstallDirs' Not found."
                   " You need to adjust LIB_SUFFIX variable (set to '' or '64')"
                   " if you don't want to install libs to ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif (GNU_DIRS_SET)
