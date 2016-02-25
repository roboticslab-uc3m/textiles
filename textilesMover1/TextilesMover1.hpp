// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEXTILES_MOVER_HPP__
#define __TEXTILES_MOVER_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include "InCommandPort.hpp"

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup textilesMover1
 *
 * @brief Execution Core 1.
 *
 */
class TextilesMover1 : public RFModule {
    public:
        bool configure(ResourceFinder &rf);

    protected:
        InCommandPort inCommandPort;
        BufferedPort<Bottle> inCvPort;
        yarp::dev::PolyDriver handDevice;
        yarp::dev::IPositionControl *iPositionControl;
        yarp::os::RpcClient cartesianPort;

        bool interruptModule();
        double getPeriod();
        bool updateModule();

};

}  // namespace teo

#endif  // __TEXTILES_MOVER_HPP__
