// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Mover.hpp"

namespace teo
{

/************************************************************************/

bool Mover::configure(yarp::os::ResourceFinder &rf) {

    //std::string arm = rf.check("arm",yarp::os::Value(DEFAULT_ARM),"full name of arm to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Mover options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    //    printf("\t--arm: %s [%s]\n",arm.c_str(),DEFAULT_ARM);
    }

    //yarp::os::Property cartesianControlOptions("(device BasicCartesianControl) (robot FakeControlboard) (axes 1) (solver KdlSolver) (angleRepr axisAngle) (gravity 0 -10 0) (numLinks 1) (link_0 (A 1) (mass 1) (cog -0.5 0 0) (inertia 1 1 1))");
    yarp::os::Property cartesianControlOptions;
    cartesianControlOptions.fromString( rf.toString() );

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
