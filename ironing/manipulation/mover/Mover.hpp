// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_MOVER_HPP__
#define __TEXTILES_MOVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>
#include <string>

#include "ICartesianControl.h"

#define DEFAULT_ARM "/teoSim/rightArm"

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

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace teo

#endif  // __TEXTILES_MOVER_HPP__
