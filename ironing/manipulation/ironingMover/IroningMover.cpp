// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "IroningMover.hpp"

namespace teo
{

/************************************************************************/

bool IroningMover::configure(yarp::os::ResourceFinder &rf) {

    cartesianControl = rf.check("cartesianControl",yarp::os::Value(DEFAULT_CARTESIAN_CONTROL),"full name of arm to be used").asString();
    robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();
    targetForce = rf.check("targetForce",yarp::os::Value(DEFAULT_TARGET_FORCE),"target force").asDouble();
    strategy = rf.check("strategy",yarp::os::Value(DEFAULT_STRATEGY),"strategy").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("Mover options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--cartesianControl: %s [%s]\n",cartesianControl.c_str(),DEFAULT_CARTESIAN_CONTROL);
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        printf("\t--targetForce: %f [%f]\n",targetForce,DEFAULT_TARGET_FORCE);
        printf("\t--strategy: %s [%s] (position, velocity, velocityForce, velocityForceTraj)\n",strategy.c_str(),DEFAULT_STRATEGY);
        ::exit(0);
    }

    if( ! openPortsAndDevices(rf) )
        return false;

    if( ! preprogrammedInitTrajectory() )
        return false;

    if(strategy == "position")
        return strategyPosition();
    else if (strategy == "velocity")
        return strategyVelocity();
    else if (strategy == "velocityForce")
        return strategyVelocityForce();
    else if (strategy == "velocityForceTraj")
        return strategyVelocityForceTraj();
    else
    {
        CD_ERROR("Unknown strategy. Init program with the --help parameter to see possible --strategy.\n");
        return false;
    }
}

/************************************************************************/
double IroningMover::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool IroningMover::updateModule() {
    //printf("StateMachine in state [%d]. Mover alive...\n", stateMachine.getMachineState());
    CD_INFO("Mover alive...\n");
    return true;
}

/************************************************************************/

bool IroningMover::interruptModule() {
    printf("Mover closing...\n");
    cartesianControlDevice.close();
    rightArmDevice.close();
    trunkDevice.close();
    return true;
}


/************************************************************************/

bool IroningMover::openPortsAndDevices(yarp::os::ResourceFinder &rf)
{
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

    //-- Connect to FT sensor device to read values.
    rightArmFTSensorPort.open("/ironingMover/force:i");
    CD_DEBUG("Wait to connect to FT sensor.");
    while( rightArmFTSensorPort.getInputCount() < 1 )
    {
        CD_DEBUG_NO_HEADER(".");
        fflush(stdout);
        yarp::os::Time::delay(0.5);
    }
    CD_SUCCESS("Connected to FT sensor.\n");

    //-- Connect to traj to read values.
    if("velocityForceTraj" == strategy)
    {
        trajPort.open("/ironingMover/traj:i");
        CD_DEBUG("Wait to connect to traj.");
        while( trajPort.getInputCount() < 1 )
        {
            CD_DEBUG_NO_HEADER(".");
            fflush(stdout);
            yarp::os::Time::delay(0.5);
        }
        CD_SUCCESS("\nConnected to  traj.\n");
    }

    yarp::os::Time::delay(1);

    return true;
}

/************************************************************************/

bool IroningMover::preprogrammedInitTrajectory()
{
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
        yarp::os::Time::delay(0.1);
    }
    CD_DEBUG_NO_HEADER("\n");

    //-- Pan head
    headIPositionControl->positionMove(0,DEFAULT_HEAD_PAN);
    //-- Tilt head forward/down
    headIPositionControl->positionMove(1,DEFAULT_HEAD_TILT);

    //-- Right arm
    if(robot=="/teoSim")
        for(int i=0;i<6;i++)
            rightArmIPositionControl->setRefSpeed(i,35);
    rightArmIPositionControl->setPositionMode();
    yarp::os::Time::delay(1);
    {
        std::vector<double> q(7,0.0);
        rightArmJointsMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        q[0] = -40;
        q[1] = -30;
        q[3] = 30;
        rightArmJointsMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        q[0] = -40;
        q[1] = -70;
        q[3] = 30;
        rightArmJointsMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        q[0] = 30;
        q[1] = -70;
        q[3] = 30;
        rightArmJointsMoveAndWait(q);
    }

    {
        std::vector<double> q(7,0.0);
        double qd[7]={-6.151142, -65.448151, 9.40246, 97.978912, 72.664323, -48.400696, 0.0};
        for(int i=0;i<7;i++) q[i]=qd[i];
        rightArmJointsMoveAndWait(q);
    }

    return true;
}

/************************************************************************/

bool IroningMover::rightArmJointsMoveAndWait(std::vector<double>& q)
{
    rightArmIPositionControl->positionMove( q.data() );
    CD_DEBUG("Waiting for right arm.");
    bool done = false;
    while(!done)
    {
        rightArmIPositionControl->checkMotionDone(&done);
        CD_DEBUG_NO_HEADER(".");
        fflush(stdout);
        yarp::os::Time::delay(0.1);
    }
    CD_DEBUG_NO_HEADER("\n");
    return true;
}

/************************************************************************/

bool IroningMover::strategyPosition()
{
    int state;
    std::vector<double> x;
    iCartesianControl->stat(state,x);
    iCartesianControl->movj(x);
    CD_DEBUG("***************DOWN*****************\n");
    double force = 0;
    while( force > targetForce )
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

bool IroningMover::strategyVelocity()
{
    int state;
    std::vector<double> x;
    iCartesianControl->stat(state,x);
    iCartesianControl->movj(x);

    CD_DEBUG("***************DOWN*****************\n");
    std::vector<double> xdot(6,0.0);
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = -0.03;
    bool okMove = iCartesianControl->movv(xdot);
    if( okMove ) {
        CD_DEBUG("Begin move arm down.\n");
    } else {
        CD_WARNING("Failed to begin move arm down.\n");
    }

    double force = 0;
    while( force > targetForce )
    {
        yarp::os::Bottle b;
        rightArmFTSensorPort.read(b);
        force = b.get(2).asDouble();
        CD_DEBUG("Moving arm down, %f\n",b.get(2).asDouble());
    }

    CD_DEBUG("***************ADVANCE*****************\n");
    xdot[0] = 0;
    xdot[1] = +0.015;
    xdot[2] = 0;

    bool okMove2 = iCartesianControl->movv(xdot);
    if( okMove2 ) {
        CD_DEBUG("Begin move arm advance.\n");
    } else {
        CD_WARNING("Failed to begin move arm advance.\n");
    }

    for(int i=0;i<50;i++)
    {
        yarp::os::Time::delay(0.5);
        yarp::os::Bottle b;

        rightArmFTSensorPort.read(b);

        CD_DEBUG("[i:%d of 50] Moved arm advance, %f\n",i,b.get(2).asDouble());
    }

    CD_DEBUG("***************UP*****************\n");
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = +0.03;

    bool okMove3 = iCartesianControl->movv(xdot);
    if( okMove3 ) {
        CD_DEBUG("Begin move arm up.\n");
    } else {
        CD_WARNING("Failed to begin move arm up.\n");
    }

    for(int i=0;i<7;i++)
    {
        yarp::os::Time::delay(0.5);
        yarp::os::Bottle b;

        rightArmFTSensorPort.read(b);

        CD_DEBUG("[i:%d of 7] Moved arm up, %f\n",i,b.get(2).asDouble());
    }
    iCartesianControl->stopControl();

    CD_DEBUG("***************DONE*****************\n");

    return true;
}

/************************************************************************/

bool IroningMover::strategyVelocityForce()
{
    int state;
    std::vector<double> x;
    iCartesianControl->stat(state,x);
    iCartesianControl->movj(x);

    CD_DEBUG("***************DOWN*****************\n");
    std::vector<double> xdot(6,0.0);
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = -0.03;
    bool okMove = iCartesianControl->movv(xdot);
    if( okMove ) {
        CD_DEBUG("Begin move arm down.\n");
    } else {
        CD_WARNING("Failed to begin move arm down.\n");
    }

    double force = 0;
    while( force > targetForce )
    {
        yarp::os::Bottle b;
        rightArmFTSensorPort.read(b);
        force = b.get(2).asDouble();
        CD_DEBUG("Moving arm down, %f\n",b.get(2).asDouble());
    }

    CD_DEBUG("***************ADVANCE*****************\n");
    xdot[0] = 0;
    xdot[1] = +0.015;
    xdot[2] = 0; //-- Change this to make some noise (e.g. +-0.002, even -0.01 with 0.1 gain)!

    for(int i=0;i<50;i++)
    {
        bool okMove2 = iCartesianControl->movv(xdot);

        yarp::os::Time::delay(0.5);
        yarp::os::Bottle b;

        rightArmFTSensorPort.read(b);

        double fe = b.get(2).asDouble()-targetForce;
        xdot[2] -= 0.05 * fe;  // 0.05 conservative but good, 0.1 works, but 0.5 too much.

        if( okMove2 ) {
            CD_DEBUG("[i:%d of 50] Moved arm advance, f:%f fd:%f fe:%f vz:%f\n",i,b.get(2).asDouble(),targetForce,fe,xdot[2]);
        } else {
            CD_WARNING("[i:%d of 50] Failed to move arm advance, f:%f fd:%f fe:%f vz:%f\n",i,b.get(2).asDouble(),targetForce,fe,xdot[2]);
        }
    }

    CD_DEBUG("***************UP*****************\n");
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = +0.03;

    bool okMove3 = iCartesianControl->movv(xdot);
    if( okMove3 ) {
        CD_DEBUG("Begin move arm up.\n");
    } else {
        CD_WARNING("Failed to begin move arm up.\n");
    }

    for(int i=0;i<7;i++)
    {
        yarp::os::Time::delay(0.5);
        yarp::os::Bottle b;

        rightArmFTSensorPort.read(b);

        CD_DEBUG("[i:%d of 7] Moved arm up, %f\n",i,b.get(2).asDouble());
    }
    iCartesianControl->stopControl();

    CD_DEBUG("***************DONE*****************\n");

    return true;
}

/************************************************************************/

bool IroningMover::strategyVelocityForceTraj()
{
    int state;
    std::vector<double> x;
    iCartesianControl->stat(state,x);
    iCartesianControl->movj(x);
    CD_DEBUG("* at: %f,%f\n",x[0],x[1]);

    yarp::os::Bottle trajectory;
    trajPort.read(trajectory);

    CD_DEBUG("*****MOVE ARM OVER TRAJECTORY POINT0****************\n");
    yarp::os::Bottle* point0 = trajectory.get(0).asList();
    x[0] = point0->get(0).asDouble();
    x[1] = point0->get(1).asDouble();
    CD_DEBUG("* going to: %f,%f\n",x[0],x[1]);
    bool okHover = iCartesianControl->movj(x);
    if( okHover ) {
        CD_DEBUG("Moved arm over point0.\n");
    } else {
        CD_WARNING("Failed to move arm over point0.\n");
    }

    CD_DEBUG("***************DOWN*****************\n");
    std::vector<double> xdot(6,0.0);
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = -0.03;
    bool okMove = iCartesianControl->movv(xdot);
    if( okMove ) {
        CD_DEBUG("Begin move arm down.\n");
    } else {
        CD_WARNING("Failed to begin move arm down.\n");
    }

    double force = 0;
    while( force > targetForce )
    {
        yarp::os::Bottle b;
        rightArmFTSensorPort.read(b);
        force = b.get(2).asDouble();
        CD_DEBUG("Moving arm down, %f\n",b.get(2).asDouble());
    }

    CD_DEBUG("***************FOLLOW TRAJECTORY*****************\n");
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = 0; //-- Change this to make some noise (e.g. +-0.002, even -0.01 with 0.1 gain)!

    const double trajectoryVelocity = 0.03;

    int pointIterator = 1;
    while(pointIterator < trajectory.size())
    {
        yarp::os::Bottle* point = trajectory.get(pointIterator).asList();
        iCartesianControl->stat(state,x);

        double xe = point->get(0).asDouble() - x[0];
        double ye = point->get(1).asDouble() - x[1];

        if( (xe  < 0.0005) && (ye < 0.0005) )
        {
            pointIterator++;
            continue;
        }

        double angle = atan2( ye, xe );

        xdot[0] = trajectoryVelocity * cos(angle);
        xdot[1] = trajectoryVelocity * sin(angle);

        yarp::os::Bottle b;
        rightArmFTSensorPort.read(b);
        double fe = b.get(2).asDouble()-targetForce;
        xdot[2] -= 0.01 * fe;  // 0.05 conservative but good with delay=0.5, 0.1 works, but 0.5 too much.

        bool okMove2 = iCartesianControl->movv(xdot);

        yarp::os::Time::delay(0.1);

        if( okMove2 ) {
            CD_DEBUG_NO_HEADER("[%d of %d] fe:%f x: %f y: %f vz:%f xe:%f ye:%f\n",pointIterator,trajectory.size(),fe,x[0],x[1],xdot[2],xe,ye);
        } else {
            CD_WARNING("[%d of %d] Failed to move arm, fe:%f vz:%f xe:%f ye:%f\n",pointIterator,trajectory.size(),fe,xdot[2],xe,ye);
        }
    }

    CD_DEBUG("***************UP*****************\n");
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = +0.03;

    bool okMove3 = iCartesianControl->movv(xdot);
    if( okMove3 ) {
        CD_DEBUG("Begin move arm up.\n");
    } else {
        CD_WARNING("Failed to begin move arm up.\n");
    }

    for(int i=0;i<7;i++)
    {
        yarp::os::Time::delay(0.5);
        yarp::os::Bottle b;

        rightArmFTSensorPort.read(b);

        CD_DEBUG("[i:%d of 7] Moved arm up, %f\n",i,b.get(2).asDouble());
    }
    iCartesianControl->stopControl();

    CD_DEBUG("***************DONE*****************\n");

    return true;
}

/************************************************************************/

}  // namespace teo
