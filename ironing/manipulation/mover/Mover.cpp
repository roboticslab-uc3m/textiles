// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Mover.hpp"

namespace teo
{

/************************************************************************/

bool Mover::configure(yarp::os::ResourceFinder &rf) {

    std::string cartesianControl = rf.check("cartesianControl",yarp::os::Value(DEFAULT_CARTESIAN_CONTROL),"full name of arm to be used").asString();
    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"full name of arm to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Mover options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--cartesianControl: %s [%s]\n",cartesianControl.c_str(),DEFAULT_CARTESIAN_CONTROL);
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        ::exit(0);
    }

    //-- Connect to robot arm (w/hand) device to send joint space commands.
    yarp::os::Property robotOptions;
    robotOptions.fromString( rf.toString() );
    robotOptions.put("device",robot);
    robotDevice.open(robotOptions);
    if( ! robotDevice.isValid() ) {
        CD_ERROR("robot device not valid: %s.\n",robotOptions.find("device").asString().c_str());
        return false;
    }
    if ( ! robotDevice.view(iPositionControl) ) {
        CD_ERROR("Could not view iPositionControl in: %s.\n",robotOptions.find("device").asString().c_str());
        return false;
    }

    yarp::os::Property cartesianControlOptions;
    cartesianControlOptions.fromString( rf.toString() );
    cartesianControlOptions.put("device",cartesianControl);

    cartesianControlDevice.open(cartesianControlOptions);
    if( ! cartesianControlDevice.isValid() ) {
        CD_ERROR("CartesianControl device not valid: %s.\n",cartesianControlOptions.find("device").asString().c_str());
        return false;
    }
    if( ! cartesianControlDevice.view(iCartesianControl) ) {
        CD_ERROR("Could not view iCartesianControl in: %s.\n",cartesianControlOptions.find("device").asString().c_str());
        return false;
    }
    yarp::os::Time::delay(1);

    return true;
}

/************************************************************************/
double Mover::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Mover::updateModule() {
    //printf("StateMachine in state [%d]. Mover alive...\n", stateMachine.getMachineState());
    CD_INFO("Mover alive...\n");
    return true;
}

/************************************************************************/

bool Mover::interruptModule() {
    printf("Mover closing...\n");
    cartesianControlDevice.close();
    return true;
}

/************************************************************************/

}  // namespace teo
