// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Mover.hpp"

namespace teo
{

/************************************************************************/

bool Mover::configure(ResourceFinder &rf) {

    std::string arm = rf.check("arm",Value(DEFAULT_ARM),"full name of arm to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Mover options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--arm: %s [%s]\n",arm.c_str(),DEFAULT_ARM);
    }

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    //-- Connect to robot arm (w/hand) device to send joint space commands.
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

    //-- Connect to right arm cartesian program to send Cartesian space commands.
    cartesianPort.open("/mover/cartesian/rpc:o");
    std::string teoCartesianServerName("/teoCartesianServer");
    teoCartesianServerName += arm;
    teoCartesianServerName += "/rpc:i";
    printf("Wait to connect to cartesian solver...\n");
    while(cartesianPort.getOutputCount() < 1)
    {
        Time::delay(0.2);
        cartesianPort.addOutput(teoCartesianServerName);
    }

    //-----------------OPEN LOCAL PORTS------------//
    inCommandPort.setInCvPortPtr(&inCvPort);
    inCommandPort.setIPositionControl(iPositionControl);
    inCommandPort.setCartesianPortPtr(&cartesianPort);
    inCommandPort.useCallback();
    inCommandPort.open("/mover/command:i");
    inCvPort.open("/mover/cv/state:i");

    return true;
}

/************************************************************************/
double Mover::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/
bool Mover::updateModule() {
    //printf("StateMachine in state [%d]. Mover alive...\n", stateMachine.getMachineState());
    return true;
}

/************************************************************************/

bool Mover::interruptModule() {
    printf("Mover closing...\n");
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
