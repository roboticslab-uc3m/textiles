// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Mover.hpp"

namespace teo
{

/************************************************************************/

bool Mover::configure(yarp::os::ResourceFinder &rf) {

    std::string cartesianControl = rf.check("cartesianControl",yarp::os::Value(DEFAULT_CARTESIAN_CONTROL),"full name of arm to be used").asString();
    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Mover options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--cartesianControl: %s [%s]\n",cartesianControl.c_str(),DEFAULT_CARTESIAN_CONTROL);
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        ::exit(0);
    }

    std::string moverStr("/mover");
    //-- Connect to arm device to send joint space commands.
    yarp::os::Property rightArmOptions;
    rightArmOptions.fromString( rf.toString() );
    rightArmOptions.put("device","remote_controlboard");
    rightArmOptions.put("local",moverStr+robot+"/rightArm");
    rightArmOptions.put("remote",robot+"/rightArm");
    rightArmDevice.open(rightArmOptions);
    if( ! rightArmDevice.isValid() ) {
        CD_ERROR("rightArm device not valid: %s.\n",rightArmOptions.find("device").asString().c_str());
        return false;
    }
    if ( ! rightArmDevice.view(rightArmIPositionControl) ) {
        CD_ERROR("Could not view rightArmIPositionControl in: %s.\n",rightArmOptions.find("device").asString().c_str());
        return false;
    }

    //-- Connect to trunk device to send joint space commands.
    yarp::os::Property trunkOptions;
    trunkOptions.fromString( rf.toString() );
    trunkOptions.put("device","remote_controlboard");
    trunkOptions.put("local",moverStr+robot+"/trunk");
    trunkOptions.put("remote",robot+"/trunk");
    trunkDevice.open(trunkOptions);
    if( ! trunkDevice.isValid() ) {
        CD_ERROR("trunk device not valid: %s.\n",trunkOptions.find("device").asString().c_str());
        return false;
    }
    if ( ! trunkDevice.view(trunkIPositionControl) ) {
        CD_ERROR("Could not view trunkIPositionControl in: %s.\n",trunkOptions.find("device").asString().c_str());
        return false;
    }

    //-- Connect to send Cartesian space commands.
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
    rightArmDevice.close();
    trunkDevice.close();
    return true;
}

/************************************************************************/

}  // namespace teo
