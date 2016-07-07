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
    /*yarp::os::Property trunkOptions;
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

    //-- Connect to head device to send joint space commands.
    yarp::os::Property headOptions;
    headOptions.fromString( rf.toString() );
    headOptions.put("device","remote_controlboard");
    headOptions.put("local",moverStr+robot+"/head");
    headOptions.put("remote",robot+"/head");
    headDevice.open(headOptions);
    if( ! headDevice.isValid() ) {
        CD_ERROR("head device not valid: %s.\n",headOptions.find("device").asString().c_str());
        return false;
    }
    if ( ! headDevice.view(headIPositionControl) ) {
        CD_ERROR("Could not view headIPositionControl in: %s.\n",headOptions.find("device").asString().c_str());
        return false;
    }*/

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

    //-- Connect to FT sensor device to send joint space commands.
    rightArmFTSensorPort.open("/mover/force:i");
    if( ! yarp::os::Network::connect("/jr3ch3:o","/mover/force:i") )
    {
        CD_ERROR("Failed to connect to force.\n");
        return false;
    }
    CD_SUCCESS("Connected to force.\n");

    yarp::os::Time::delay(1);

    //-- Tilt trunk forward/down
    //trunkIPositionControl->positionMove(1,DEFAULT_TRUNK_TILT);

    //-- Tilt head forward/down
    //headIPositionControl->positionMove(1,DEFAULT_HEAD_TILT);

    rightArmIPositionControl->setPositionMode();
    //-- Move arm to good position out of singularity
    CD_DEBUG("Move arm to good position out of singularity\n");
    std::vector<double> q(7,0.0);
    q[0] = -10;  //-- shoulder first
    q[3] = 30;  //-- elbow
    qMoveAndWait(q);

    int state;
    std::vector<double> x;
    //-- Move arm up
    CD_DEBUG("Move arm up\n");
    iCartesianControl->stat(state,x);
    x[2] += 0.1;
    iCartesianControl->movj(x);

    //-- Move arm front
    CD_DEBUG("Move arm front\n");
    //iCartesianControl->stat(state,x);
    x[0] += 0.2;
    x[2] += 0.1;
    iCartesianControl->movj(x);

    //-- Move arm more front
    CD_DEBUG("Move arm more front\n");
    //iCartesianControl->stat(state,x);
    x[0] += 0.1;
    iCartesianControl->movj(x);

    //-- Rotate
    /*CD_DEBUG("Rotate\n");
    double q1[7] = { 32.601055, -33.989471, -42.776794, 68.014061, 33.378273, -54.288239 ,0.0};
    for(int i=0;i<7;i++) q[i] = q1[i];
    rightArmIPositionControl->positionMove( q.data() );
    CD_SUCCESS("Waiting\n");
    done = false;
    while(!done)
    {
        rightArmIPositionControl->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
        yarp::os::Time::delay(0.5);
    }*/

    //CD_DEBUG("***************PRE-LOOP*****************\n");
    //iCartesianControl->stat(state,x);
    //iCartesianControl->movj(x);
    CD_DEBUG("***************LOOP*****************\n");
    double force = 0;
    while( force > -0.2)
    {
        yarp::os::Bottle b;
        x[2] -= 0.005;
        iCartesianControl->movj(x);
        rightArmFTSensorPort.read(b);
        force = b.get(3).asDouble();
        CD_DEBUG("Moved arm down, %f\n",b.get(3).asDouble());
    }

    CD_DEBUG("***************RETURN*****************\n");

    while( 1 )
    {
        yarp::os::Bottle b;
        x[2] += 0.005;
        iCartesianControl->movj(x);
        rightArmFTSensorPort.read(b);
        force = b.get(3).asDouble();
        CD_DEBUG("Moved arm up, %f\n",b.get(3).asDouble());
    }

    CD_DEBUG("***************DONE*****************\n");
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

bool Mover::qMoveAndWait(std::vector<double>& q)
{
    rightArmIPositionControl->positionMove( q.data() );
    CD_SUCCESS("Waiting\n");
    bool done = false;
    while(!done)
    {
        rightArmIPositionControl->checkMotionDone(&done);
        printf(".");
        fflush(stdout);
        yarp::os::Time::delay(0.5);
    }
    return true;
}

}  // namespace teo
