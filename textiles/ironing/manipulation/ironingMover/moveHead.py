#! /usr/bin/env python

import yarp

yarp.Network.init()
if yarp.Network.checkNetwork() != True:
    print "[error] Please try running yarp server"
    quit()
options = yarp.Property()
options.put('device','remote_controlboard')
options.put('local','/python')
options.put('remote','/teo/head')
dd = yarp.PolyDriver(options)

pos = dd.viewIPositionControl()
pos.setPositionMode()
pos.setRefSpeed(1,2)
pos.setRefSpeed(0,2)

pos.positionMove(1,-6)  # Down extreme w/o occlusions
pos.positionMove(0,-45)  # Right extreme
yarp.Time.delay(1)

pos.positionMove(0,-30)  # Left enough
yarp.Time.delay(3)

pos.positionMove(1,-8)  # More up
yarp.Time.delay(1)

pos.positionMove(0,-45)  # Right extreme
yarp.Time.delay(3)

yarp.Network.fini()


