# Install script for directory: /home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libzed_open_capture.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libzed_open_capture.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libzed_open_capture.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libzed_open_capture.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/build/libzed_open_capture.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libzed_open_capture.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libzed_open_capture.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libzed_open_capture.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/zed-open-capture/sensorcapture.hpp;/usr/local/include/zed-open-capture/defines.hpp;/usr/local/include/zed-open-capture/sensorcapture_def.hpp;/usr/local/include/zed-open-capture/videocapture.hpp;/usr/local/include/zed-open-capture/defines.hpp;/usr/local/include/zed-open-capture/videocapture_def.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/zed-open-capture" TYPE FILE FILES
    "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/include/sensorcapture.hpp"
    "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/include/defines.hpp"
    "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/include/sensorcapture_def.hpp"
    "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/include/videocapture.hpp"
    "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/include/defines.hpp"
    "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/include/videocapture_def.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/zed_open_capture_depth_tune_stereo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/zed_open_capture_depth_tune_stereo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/zed_open_capture_depth_tune_stereo"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/zed_open_capture_depth_tune_stereo")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/build/zed_open_capture_depth_tune_stereo")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/zed_open_capture_depth_tune_stereo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/zed_open_capture_depth_tune_stereo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/zed_open_capture_depth_tune_stereo"
         OLD_RPATH "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/build:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/zed_open_capture_depth_tune_stereo")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nadine/latest-mate-repo/MATE-ROV-2025/software/length-measurement/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
