// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// -----------------------------------------------------------------------------

bool TextilesHand::open(Searchable& config) {
    char serialport[13] = "/dev/ttyUSB0";
    int baudrate = B9600;  // default
    char buf[256];
    int rc,n;

    fd = serialport_init(serialport, baudrate);
    if(!fd) {
        printf("NULL fd, bye!\n");
        ::exit(-1);
    }
    printf("[success] Skymegabot open(), fd: %d\n",fd);

    return true;
}

// -----------------------------------------------------------------------------

bool TextilesHand::getAxes(int *axes) {
    *axes = DEFAULT_NUM_MOTORS;
    return true;
}

// -----------------------------------------------------------------------------

bool TextilesHand::positionMove(int j, double ref) {
    printf("[TextilesHand] velocityMove(%d, %f)\n",j,ref);
    if (j>DEFAULT_NUM_MOTORS) return false;
    unsigned char cmdByte = (unsigned int)ref/46;  // 3000/64 = 46; usamos 6 bits -> 64
    //printf("cmdByte (pre-dir): 0x%x\n",cmdByte);
    unsigned char direccion = j;
    direccion <<= 6;  // shift 6 positions
    cmdByte |= direccion;
    //printf("cmdByte (post-dir): 0x%x\n",cmdByte);
    int res = serialport_writebyte(fd, cmdByte);
    if(res==-1) return false;
    return true;
}

// -----------------------------------------------------------------------------

bool TextilesHand::positionMove(const double *refs) {
    bool ok = true;
    for(int i=0;i<DEFAULT_NUM_MOTORS;i++)
        ok &= positionMove(i,refs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool TextilesHand::setRefAcceleration(int j, double acc) {
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::setRefAccelerations(const double *accs) {
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::getRefAcceleration(int j, double *acc) {
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::getRefAccelerations(double *accs) {
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::stop(int j) {
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::stop() {
    return false;
}

