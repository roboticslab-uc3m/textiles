// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IN_SR_PORT_HPP__
#define __IN_SR_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#include <kdl/frames.hpp>

#define VOCAB_GO VOCAB2('g','o')

// thanks! https://web.stanford.edu/~qianyizh/projects/scenedata.html
#define DEFAULT_FX_D          525.0  // 640x480
#define DEFAULT_FY_D          525.0  //
#define DEFAULT_CX_D          319.5  //
#define DEFAULT_CY_D          239.5  //
#define DEFAULT_FX_RGB        525.0  //
#define DEFAULT_FY_RGB        525.0  //
#define DEFAULT_CX_RGB        319.5  //
#define DEFAULT_CY_RGB        239.5  //

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup textilesMover1
 *
 * @brief Input port of speech recognition data.
 *
 */
class InCommandPort : public BufferedPort<Bottle> {
    public:
        void setInCvPortPtr(BufferedPort<Bottle> *inCvPortPtr) {
            this->inCvPortPtr = inCvPortPtr;
        }

        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;
        }

    protected:
        /** Callback on incoming Bottle. **/
        virtual void onRead(Bottle& b);

        BufferedPort<Bottle>* inCvPortPtr;

        yarp::dev::IPositionControl *iPositionControl;
};

}  // namespace teo

#endif  // __IN_SR_PORT_HPP__
