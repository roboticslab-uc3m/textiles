// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InSrPort.hpp"

namespace teo
{

/************************************************************************/

void InSrPort::onRead(Bottle& b) {
    if ((b.get(0).asString() == "go")||(b.get(0).asVocab() == VOCAB_GO))  // go //
    {
        Bottle* cvb = inCvPortPtr->read();
        int x1 = cvb->get(0).asInt();
        int y1 = cvb->get(1).asInt();
        double z1 = cvb->get(2).asDouble();
        int x2 = cvb->get(3).asInt();
        int y2 = cvb->get(4).asInt();
        double z2 = cvb->get(5).asDouble();
    }
}

/************************************************************************/

}  // namespace teo
