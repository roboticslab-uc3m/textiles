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
#define DEFAULT_ROBOT "/robot"

#define DEFAULT_TRUNK_TILT 15.0
#define DEFAULT_HEAD_TILT 7.0

namespace teo
{

/**
 * @ingroup mover
 *
 * @brief Execution Core 1.
 *
 */
class Mover : public yarp::os::RFModule {
    public:
        bool configure(yarp::os::ResourceFinder &rf);

    protected:
        yarp::dev::PolyDriver cartesianControlDevice;
        teo::ICartesianControl *iCartesianControl;

        yarp::dev::PolyDriver rightArmDevice;
        yarp::dev::IPositionControl *rightArmIPositionControl;

        yarp::dev::PolyDriver trunkDevice;
        yarp::dev::IPositionControl *trunkIPositionControl;

        yarp::dev::PolyDriver headDevice;
        yarp::dev::IPositionControl *headIPositionControl;

        yarp::os::Port rightArmFTSensorPort;

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace teo

#endif  // __TEXTILES_MOVER_HPP__
