// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IN_CV_PORT_HPP__
#define __IN_CV_PORT_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <stdlib.h>

#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup textilesMover1
 *
 * @brief Input port of computer vision data.
 *
 */
class InCvPort : public BufferedPort<Bottle> {
    public:

        InCvPort(): follow(false) {}

        void setIPositionControl(yarp::dev::IPositionControl *iPositionControl) {
            this->iPositionControl = iPositionControl;
        }

        void setFollow(bool value);

protected:
        bool follow;

        /** Callback on incoming Bottle. **/
        virtual void onRead(Bottle& b);

        yarp::dev::IPositionControl *iPositionControl;
};

}  // namespace teo

#endif  // __IN_CV_PORT_HPP__
