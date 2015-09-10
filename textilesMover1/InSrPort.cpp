// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

namespace teo
{

/************************************************************************/

void InSrPort::onRead(Bottle& b) {
    switch ( b.get(0).asVocab() ) {
        case VOCAB_FOLLOW_ME:
            printf("follow\n");
            inCvPortPtr->setFollow(true);
            break;
        case VOCAB_STOP_FOLLOWING:
            printf("stopFollowing\n");
            inCvPortPtr->setFollow(false);
            break;
        default:
            break;
    }
}

/************************************************************************/

}  // namespace teo
