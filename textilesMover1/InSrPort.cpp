// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

namespace teo
{

/************************************************************************/

void InSrPort::onRead(Bottle& b) {
    switch ( b.get(0).asVocab() ) {
        case VOCAB_FOLLOW_ME:
            printf("follow\n");
            if( x > 50 ) iPositionControl->relativeMove(0, -2);
            if( x < -50 ) iPositionControl->relativeMove(0, 2);
            //
            if( y > 50 ) iPositionControl->relativeMove(1, 2);
            if( y < -50 ) iPositionControl->relativeMove(1, -2);

            break;
        case VOCAB_STOP_FOLLOWING:
            printf("stopFollowing\n");
            break;
        default:
            break;
    }
}

/************************************************************************/

}  // namespace teo
