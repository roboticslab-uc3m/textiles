# Copyright: (C) 2012 Universidad Carlos III de Madrid
# Author: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

YARP_PREPARE_DEVICE(TextilesHand TYPE TextilesHand INCLUDE TextilesHand.hpp WRAPPER controlboard)

#IF (NOT SKIP_TextilesHand)

find_package(YARP REQUIRED)

# Include any directories needed for YARP
include_directories(${YARP_INCLUDE_DIRS})

YARP_ADD_PLUGIN(TextilesHand TextilesHand.hpp TextilesHand.cpp HelperFuncs.cpp)
TARGET_LINK_LIBRARIES(TextilesHand ${YARP_LIBRARIES})

#ENDIF (NOT SKIP_TextilesHand)

