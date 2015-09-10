// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InCommandPort.hpp"

namespace teo
{

/************************************************************************/

void InCommandPort::onRead(Bottle& b) {
    if ((b.get(0).asString() == "go")||(b.get(0).asVocab() == VOCAB_GO))  // go //
    {
        Bottle* cvb = inCvPortPtr->read();
        int pxX1 = cvb->get(0).asInt();
        int pxY1 = cvb->get(1).asInt();
        double mmZ1 = cvb->get(2).asDouble();  // Points forward.
        int pxX2 = cvb->get(3).asInt();
        int pxY2 = cvb->get(4).asInt();
        double mmZ2 = cvb->get(5).asDouble();  // Points forward.

        double mmX1 = -(1000.0 * ( (pxX1 - DEFAULT_CX_D) * mmZ1/1000.0 ) / DEFAULT_FX_D);  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
        double mmX2 = -(1000.0 * ( (pxX2 - DEFAULT_CX_D) * mmZ2/1000.0 ) / DEFAULT_FX_D);  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
        double mmY1 = 1000.0 * ( (pxY1 - DEFAULT_CY_D) * mmZ1/1000.0 ) / DEFAULT_FY_D;  // Points down.
        double mmY2 = 1000.0 * ( (pxY2 - DEFAULT_CY_D) * mmZ2/1000.0 ) / DEFAULT_FY_D;  // Points down.

    }
}

/************************************************************************/

}  // namespace teo
