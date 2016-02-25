// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesMover1.hpp"

namespace teo
{

/************************************************************************/

bool TextilesMover1::configure(ResourceFinder &rf) {

    std::string arm = rf.check("arm",Value(DEFAULT_ARM),"full name of arm to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("TextilesMover1 options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--arm: %s [%s]\n",arm.c_str(),DEFAULT_ARM);
    }

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //
    Property armOptions;
    armOptions.put("device","remote_controlboard");
    armOptions.put("remote",arm);
    armOptions.put("local","/local");
    armDevice.open(armOptions);
    if(!armDevice.isValid()) {
      printf("armDevice device not available.\n");
      return 1;
    }
    if (! armDevice.view(iPositionControl) ) {
        printf("[error] Problems acquiring armDevice position interface\n");
        return 1;
    }

    //
    cartesianPort.open("/textilesMover1/cartesian/rpc:o");
    std::string teoCartesianServerName("/teoCartesianServer");
    teoCartesianServerName += arm;
    teoCartesianServerName += "/rpc:i";
    yarp::os::Network::connect("/textilesMover1/rpc:o",teoCartesianServerName);

    //-----------------OPEN LOCAL PORTS------------//
    inCommandPort.setInCvPortPtr(&inCvPort);
    inCommandPort.setIPositionControl(iPositionControl);
    inCommandPort.setCartesianPortPtr(&cartesianPort);
    inCommandPort.useCallback();
    inCommandPort.open("/textilesMover1/command:i");
    inCvPort.open("/textilesMover1/cv/state:i");

    return true;
}

/************************************************************************/
double TextilesMover1::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool TextilesMover1::updateModule() {
    //printf("StateMachine in state [%d]. TextilesMover1 alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool TextilesMover1::interruptModule() {
    printf("TextilesMover1 closing...\n");
    inCommandPort.disableCallback();
    inCvPort.interrupt();
    inCommandPort.interrupt();
    cartesianPort.interrupt();
    armDevice.close();
    inCvPort.close();
    inCommandPort.close();
    cartesianPort.close();
    return true;
}

/************************************************************************/

}  // namespace teo
