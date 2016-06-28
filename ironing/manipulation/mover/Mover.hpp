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
#define DEFAULT_ROBOT "remote_controlboard"

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

        yarp::dev::PolyDriver robotDevice;
        yarp::dev::IPositionControl *iPositionControl;

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace teo

#endif  // __TEXTILES_MOVER_HPP__
