// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_body_programs
 * \defgroup mover mover
 *
 * @brief Creates an instance of teo::Mover.
 *
 * @section mover_legal Legal
 *
 * Copyright: 2015 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at mover
 *
 */

#include <yarp/os/all.h>

#include "Mover.hpp"

using namespace yarp::os;

YARP_DECLARE_PLUGINS(TeoYarp)

int main(int argc, char **argv) {

    YARP_REGISTER_PLUGINS(TeoYarp);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("mover");
    rf.setDefaultConfigFile("mover.ini");
    rf.configure(argc, argv);

    teo::IroningMover mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"%s --help\" for options.\n",argv[0]);
    printf("%s checking for yarp network... ",argv[0]);
    fflush(stdout);
    Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr,"[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n",argv[0]);
        return 1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}

