// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Mover.hpp"

namespace teo
{

/************************************************************************/

bool Mover::configure(yarp::os::ResourceFinder &rf) {

    std::string cartesianControl = rf.check("cartesianControl",yarp::os::Value(DEFAULT_CARTESIAN_CONTROL),"full name of arm to be used").asString();
    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();
    forceThreshold = rf.check("forceThreshold",yarp::os::Value(DEFAULT_FORCE_THRESHOLD),"force threshold").asDouble();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Mover options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--cartesianControl: %s [%s]\n",cartesianControl.c_str(),DEFAULT_CARTESIAN_CONTROL);
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        printf("\t--forceThreshold: %f [%f]\n",forceThreshold,DEFAULT_FORCE_THRESHOLD);
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

    //-- Connect to FT sensor device to send joint space commands.
    rightArmFTSensorPort.open("/mover/force:i");
    if( ! yarp::os::Network::connect("/jr3ch3:o","/mover/force:i") )
    {
        CD_ERROR("Failed to connect to force.\n");
        return false;
    }
    CD_SUCCESS("Connected to force.\n");

    yarp::os::Time::delay(1);

    //-- Pan trunk
    if(robot=="/teo")
        trunkIPositionControl->setRefSpeed(0,0.1);
    else if(robot=="/teoSim")
        trunkIPositionControl->setRefSpeed(0,15);
    trunkIPositionControl->positionMove(0,DEFAULT_TRUNK_PAN);

    //-- Tilt trunk forward/down
    if(robot=="/teo")
        trunkIPositionControl->setRefSpeed(1,0.1);
    else if(robot=="/teoSim")
        trunkIPositionControl->setRefSpeed(1,15);
    trunkIPositionControl->positionMove(1,DEFAULT_TRUNK_TILT);
    CD_DEBUG("Waiting for trunk.");
    bool done = false;
    while(!done)
    {
        trunkIPositionControl->checkMotionDone(&done);
        CD_DEBUG_NO_HEADER(".");
        fflush(stdout);
        yarp::os::Time::delay(0.25);
    }
    CD_DEBUG_NO_HEADER("\n");

    //-- Pan head
    headIPositionControl->positionMove(0,DEFAULT_HEAD_PAN);
    //-- Tilt head forward/down
    headIPositionControl->positionMove(1,DEFAULT_HEAD_TILT);

    //-- Right arm
    if(robot=="/teoSim")
        for(int i=0;i<6;i++)
            rightArmIPositionControl->setRefSpeed(i,25);
    rightArmIPositionControl->setPositionMode();
    yarp::os::Time::delay(1);
    {
        std::vector<double> q(7,0.0);
        q[3] = 10;
        qMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        q[0] = -40;
        q[1] = -30;
        q[3] = 30;
        qMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        q[0] = -40;
        q[1] = -70;
        q[3] = 30;
        qMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        q[0] = 30;
        q[1] = -70;
        q[3] = 30;
        qMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        double qd[7]={6.151142, -65.448151, 9.40246, 97.978912, 72.664323, -48.400696, 0.0};
        for(int i=0;i<7;i++) q[i]=qd[i];
        qMoveAndWait(q);
    }

    //return strategyBasic();
    return strategyBasicVel();
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
    CD_DEBUG("Waiting for right arm.");
    bool done = false;
    while(!done)
    {
        rightArmIPositionControl->checkMotionDone(&done);
        CD_DEBUG_NO_HEADER(".");
        fflush(stdout);
        yarp::os::Time::delay(0.25);
    }
    CD_DEBUG_NO_HEADER("\n");
    return true;
}

/************************************************************************/

bool Mover::strategyBasic()
{
    int state;
    std::vector<double> x;
    iCartesianControl->stat(state,x);
    iCartesianControl->movj(x);
    CD_DEBUG("***************DOWN*****************\n");
    double force = 0;
    while( force > forceThreshold )
    {
        yarp::os::Bottle b;
        x[2] -= 0.005;
        bool okMove = iCartesianControl->movj(x);
        rightArmFTSensorPort.read(b);
        force = b.get(2).asDouble();
        if( okMove ) {
            CD_DEBUG("Moved arm down, %f\n",force);
        } else {
            CD_WARNING("Failed to move arm down, %f\n",force);
        }
    }

    CD_DEBUG("***************ADVANCE*****************\n");
    for(int i=0;i<24;i++)
    {
        yarp::os::Bottle b;
        x[1] += 0.005;
        bool okMove = iCartesianControl->movj(x);

        rightArmFTSensorPort.read(b);

        if( okMove ) {
            CD_DEBUG("[i:%d of 24] Moved arm advance, %f\n",i,b.get(2).asDouble());
        } else {
            CD_WARNING("[i:%d of 24] Failed to move arm advance, %f\n",i,b.get(2).asDouble());
        }
    }

    CD_DEBUG("***************UP*****************\n");

    for(int i=0;i<24;i++)
    {
        yarp::os::Bottle b;
        x[2] += 0.005;
        bool okMove = iCartesianControl->movj(x);

        rightArmFTSensorPort.read(b);

        force = b.get(2).asDouble();

        if( okMove ) {
            CD_DEBUG("[i:%d of 24] Moved arm up, %f\n",i,b.get(2).asDouble());
        } else {
            CD_WARNING("[i:%d of 24] Failed to move arm up, %f\n",i,b.get(2).asDouble());
        }
    }

    CD_DEBUG("***************DONE*****************\n");

    return true;
}

/************************************************************************/

bool Mover::strategyBasicVel()
{
    int state;
    std::vector<double> x;
    iCartesianControl->stat(state,x);
    iCartesianControl->movj(x);

    CD_DEBUG("***************DOWN*****************\n");
    std::vector<double> xdot(6,0.0);
    xdot[2] = -0.0025;
    bool okMove = iCartesianControl->movv(xdot);
    if( okMove ) {
        CD_DEBUG("Begin move arm down.\n");
    } else {
        CD_WARNING("Failed to begin move arm down.\n");
    }

    double force = 0;
    while( force > forceThreshold )
    {
        yarp::os::Bottle b;
        rightArmFTSensorPort.read(b);
        force = b.get(2).asDouble();
        CD_DEBUG("Moving arm down, %f\n",b.get(2).asDouble());
    }
    iCartesianControl->stopControl();

    CD_DEBUG("***************ADVANCE*****************\n");
    for(int i=0;i<24;i++)
    {
        yarp::os::Bottle b;
        x[1] += 0.005;
        bool okMove = iCartesianControl->movj(x);

        rightArmFTSensorPort.read(b);

        if( okMove ) {
            CD_DEBUG("[i:%d of 24] Moved arm advance, %f\n",i,b.get(2).asDouble());
        } else {
            CD_WARNING("[i:%d of 24] Failed to move arm advance, %f\n",i,b.get(2).asDouble());
        }
    }

    CD_DEBUG("***************UP*****************\n");

    for(int i=0;i<24;i++)
    {
        yarp::os::Bottle b;
        x[2] += 0.005;
        bool okMove = iCartesianControl->movj(x);

        rightArmFTSensorPort.read(b);

        force = b.get(2).asDouble();

        if( okMove ) {
            CD_DEBUG("[i:%d of 24] Moved arm up, %f\n",i,b.get(2).asDouble());
        } else {
            CD_WARNING("[i:%d of 24] Failed to move arm up, %f\n",i,b.get(2).asDouble());
        }
    }

    CD_DEBUG("***************DONE*****************\n");

    return true;
}

/************************************************************************/

}  // namespace teo
