// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_MOVER_HPP__
#define __TEXTILES_MOVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>
#include <string>

#include "InCommandPort.hpp"

#define DEFAULT_ARM "/teoSim/rightArm"

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup mover
 *
 * @brief Execution Core 1.
 *
 */
class Mover : public RFModule {
    public:
        bool configure(ResourceFinder &rf);

    protected:
        InCommandPort inCommandPort;
        BufferedPort<Bottle> inCvPort;
        yarp::dev::PolyDriver armDevice;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::os::RpcClient cartesianPort;

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace teo

#endif  // __TEXTILES_MOVER_HPP__
