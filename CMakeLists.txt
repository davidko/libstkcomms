# STK Communications library
# Barobo, Inc.

cmake_minimum_required(VERSION 2.8.9)
project(LIBSTKCOMMS C CXX)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

##############################################################################
# Headers

set(PUBLIC_INCLUDE_DIR include)
include_directories(${PUBLIC_INCLUDE_DIR})

##############################################################################
# Setup

add_definitions(-DBUILD_CSTKCOMMS)

##############################################################################
# Targets

set(TARGET stkcomms)
set(SOURCES src/libstkcomms.cpp src/libstkcomms++.cpp)

add_library(${TARGET} ${SOURCES})

##############################################################################
# Platform Specific Setup

if(WIN32)
  # FIXME find a place for findwindowssdk.cmake to live
  include("../libbarobo/findwindowssdk.cmake")
  if(NOT WINSDK_VERSION)
    message(FATAL_ERROR "libstkcomms requires the Windows SDK")
  endif()

  # Prior to Windows SDK 8.0, there was only a single flat Include directory.
  # Thereafter, the headers were broken out into three separate directories:
  # winrt (Windows Store API), shared, and um (user mode). User mode is the
  # one which contains the classic headers we care about, like windows.h and
  # ws2bth.h. This command will figure out which one we're dealing with.
  find_path(WINSDK_UM_INCLUDE_DIR windows.h
    PATHS "${WINSDK_INSTALL_DIR}/Include"
          "${WINSDK_INSTALL_DIR}/Include/um"
    NO_DEFAULT_PATH)

  if(NOT WINSDK_UM_INCLUDE_DIR)
    message(FATAL_ERROR "Unable to find Windows SDK user mode headers under ${WINSDK_INSTALL_DIR}")
  endif()

  add_definitions(-D_WIN32_WINNT=0x0502 -DWIN32_LEAN_AND_MEAN)

  set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS
    "-idirafter \"${WINSDK_UM_INCLUDE_DIR}\"")

  target_link_libraries(${TARGET} ws2_32)
  #set_target_properties(${TARGET} PROPERTIES COMPILE_FLAGS "/GS-")
elseif(APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch i386")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch i386")
  set(CMAKE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  target_link_libraries(${TARGET} pthread)
  set_target_properties(${TARGET} PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  install(TARGETS ${TARGET} 
          ARCHIVE DESTINATION lib)
  install(FILES libstkcomms.hpp DESTINATION include)
else()
  #target_link_libraries(${TARGET} bluetooth pthread rt)
  install(TARGETS ${TARGET} 
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(FILES libstkcomms.h DESTINATION include)
endif()

##############################################################################
# Configuration for dependent projects

get_filename_component(${PROJECT_NAME}_INCLUDE_DIRS ${PUBLIC_INCLUDE_DIR} ABSOLUTE CACHE)
set(${PROJECT_NAME}_LIBRARIES ${TARGET} CACHE INTERNAL "")
set(${PROJECT_NAME}_VERSION ${VERSION} CACHE INTERNAL "")
