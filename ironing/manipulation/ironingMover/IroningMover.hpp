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

        double targetForce;

        bool strategyPosition();
        bool strategyVelocity();
        bool strategyVelocityForce();

        bool qMoveAndWait(std::vector<double>& q);

        /** Port to read from force sensor */
        yarp::os::Port rightArmFTSensorPort;

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
