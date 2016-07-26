// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IRONING_MOVER_HPP__
#define __IRONING_MOVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>
#include <string>

#include "ICartesianControl.h"

#include "ColorDebug.hpp"

#define DEFAULT_CARTESIAN_CONTROL "CartesianControlClient"
#define DEFAULT_ROBOT "/teo"

#define DEFAULT_TARGET_FORCE -0.2

#define DEFAULT_STRATEGY "position"

#define DEFAULT_HEAD_PAN -45.0
#define DEFAULT_HEAD_TILT 7.0

#define DEFAULT_TRUNK_PAN 45.0
#define DEFAULT_TRUNK_TILT 30.0

namespace teo
{

class IroningMover : public yarp::os::RFModule
{
    public:
        bool configure(yarp::os::ResourceFinder &rf);

    private:

        /** Strategy: Position.
         *  - Down: Moves down (negative of root Z) in 5 mm increments of movj, until targetForce check in each iteration.
         *  - Advance: Advances (positive of root Y) in 5 mm increments of movj, done after 24 iterations.
         *  - Up:  Moves up (positive of root Z) in 5 mm increments of movj, done after 24 iterations. */
        bool strategyPosition();

        /** Strategy: Velocity.
         *  - Down: Moves down (negative of root Z) at 30 mm/s using movv, until targetForce check in each iteration with no delay.
         *  - Advance: Advances (positive of root Y) at 15 mm/s using movv, done after 50 iterations with 0.5 s delays.
         *  - Up:  Moves up (positive of root Z) at 30 mm/s using movv, done after 7 iterations with 0.5 s delays. */
        bool strategyVelocity();

        /** Strategy: Velocity Force.
         *  - Down: Moves down (negative of root Z) at 30 mm/s using movv, until targetForce check in each iteration with no delay.
         *  - Advance: Advances (positive of root Y) at 15 mm/s using movv, modifying root Z component proportional to force error
         *    at each iteration with 0.5 s delays, done after 50 iterations.
         *  - Up:  Moves up (positive of root Z) at 30 mm/s using movv, done after 7 iterations with 0.5 s delays. */
        bool strategyVelocityForce();

        /** Target force, used in all strategies for now. */
        double targetForce;

        /** Robot port prefix, such as /teo or /teoSim. */
        std::string robot;

        /** Open Ports and Devices */
        bool openPortsAndDevices(yarp::os::ResourceFinder &rf);

        /** Preprogrammed Initialization Trajectory */
        bool preprogrammedInitTrajectory();

        /** Right arm joints move and wait (auxiliary function due to many calls) */
        bool rightArmJointsMoveAndWait(std::vector<double>& q);

        /** Port to read from force sensor */
        yarp::os::Port rightArmFTSensorPort;

        /** Port to read from vision */
        yarp::os::Port visionPort;

        /** Cartesian Control Name */
        std::string cartesianControl;
        /** Cartesian Control Device */
        yarp::dev::PolyDriver cartesianControlDevice;
        /** Cartesian Control Interface */
        teo::ICartesianControl *iCartesianControl;

        /** Right Arm Device */
        yarp::dev::PolyDriver rightArmDevice;
        /** Right Arm Position Interface */
        yarp::dev::IPositionControl *rightArmIPositionControl;

        /** Trunk Device */
        yarp::dev::PolyDriver trunkDevice;
        /** Trunk Position Interface */
        yarp::dev::IPositionControl *trunkIPositionControl;

        /** Head Device */
        yarp::dev::PolyDriver headDevice;
        /** Head Position Interface */
        yarp::dev::IPositionControl *headIPositionControl;

        /** RFModule interruptModule. */
        bool interruptModule();
        /** RFModule getPeriod. */
        double getPeriod();
        /** RFModule updateModule. */
        bool updateModule();

};

}  // namespace teo

#endif  // __IRONING_MOVER_HPP__
