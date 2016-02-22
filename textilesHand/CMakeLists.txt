# Copyright: (C) 2012 Universidad Carlos III de Madrid
# Authors: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)
find_package(YARP REQUIRED)
#include_directories(${YARP_INCLUDE_DIRS})
#include(${YARP_MODULE_PATH}/YarpPlugin.cmake)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpPlugin)
yarp_begin_plugin_library(TextilesPlugins)
add_subdirectory(TextilesHand)
yarp_end_plugin_library(TextilesPlugins)

#yarp_add_plugin_yarpdev_executable(launchTextilesPlugins TextilesPlugins)
#INSTALL(TARGETS launchTextilesPlugins DESTINATION bin)

