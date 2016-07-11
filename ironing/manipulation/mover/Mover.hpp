// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_MOVER_HPP__
#define __TEXTILES_MOVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>
#include <string>

#include "ICartesianControl.h"

#include "ColorDebug.hpp"

#define DEFAULT_CARTESIAN_CONTROL "CartesianControlClient"
#define DEFAULT_ROBOT "/teo"

#define DEFAULT_TARGET_FORCE -0.2

#define DEFAULT_STRATEGY "basicPosition"

#define DEFAULT_HEAD_PAN -45.0
#define DEFAULT_HEAD_TILT 7.0

#define DEFAULT_TRUNK_PAN 45.0
#define DEFAULT_TRUNK_TILT 30.0

namespace teo
{

/**
 * @ingroup mover
 *
 * @brief Execution Core 1.
 *
 */
class IroningMover : public yarp::os::RFModule {
    public:
        bool configure(yarp::os::ResourceFinder &rf);

    protected:

        double targetForce;

        bool strategyBasicPosition();
        bool strategyBasicVelocity();
        bool strategyAdvancedVelocity();

        yarp::dev::PolyDriver cartesianControlDevice;
        teo::ICartesianControl *iCartesianControl;

        yarp::dev::PolyDriver rightArmDevice;
        yarp::dev::IPositionControl *rightArmIPositionControl;

        yarp::dev::PolyDriver trunkDevice;
        yarp::dev::IPositionControl *trunkIPositionControl;

        yarp::dev::PolyDriver headDevice;
        yarp::dev::IPositionControl *headIPositionControl;

        yarp::os::Port rightArmFTSensorPort;

        bool qMoveAndWait(std::vector<double>& q);

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace teo

#endif  // __TEXTILES_MOVER_HPP__
