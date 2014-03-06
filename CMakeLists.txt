# The ODE Library

cmake_minimum_required(VERSION 2.6)
#include_directories(../opende/sys/include)

project(LIBSTKCOMMS)

find_package(LIBBAROBO)
include_directories(${LIBBAROBO_INCLUDE_DIRS})

if(CMAKE_HOST_WIN32)
  # FIXME hack--put these cmake scripts in a proper place (we need a
  # barobo_common repo) 
  if(NOT WINSDK_INCLUDE_PATH)
    include("../findwindowssdk.cmake")
  endif(NOT WINSDK_INCLUDE_PATH)
  if(NOT MSVC_INCLUDE_PATH)
    include("../findmsvc.cmake")
  endif(NOT MSVC_INCLUDE_PATH)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -idirafter \"${WINSDK_INCLUDE_PATH}\" -idirafter \"${MSVC_INCLUDE_PATH}\"")

  add_library(stkcomms STATIC
    libstkcomms.cpp
    libstkcomms++.cpp
    )
  add_definitions(-D_WIN32_WINNT=0x0502 -DWIN32_LEAN_AND_MEAN -DBUILD_CSTKCOMMS)
  target_link_libraries(stkcomms Ws2_32)
  #set_target_properties(stkcomms PROPERTIES COMPILE_FLAGS "/GS-")
elseif(CMAKE_HOST_APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch i386")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch i386")
  set(CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
	add_definitions(-DBUILD_CSTKCOMMS)
  add_library(stkcomms STATIC
	  libstkcomms.cpp
	  libstkcomms++.cpp
	  )
	target_link_libraries(stkcomms pthread)
  set_target_properties(stkcomms PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  install(TARGETS stkcomms 
          ARCHIVE DESTINATION lib)
  install(FILES libstkcomms.hpp DESTINATION include)
else(CMAKE_HOST_WIN32)
	add_definitions(-DBUILD_CSTKCOMMS)
  add_library(stkcomms STATIC
	  libstkcomms.cpp
	  libstkcomms++.cpp
	  )
  #target_link_libraries(stkcomms bluetooth pthread rt)
  install(TARGETS stkcomms 
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(FILES libstkcomms.h DESTINATION include)
endif(CMAKE_HOST_WIN32)

