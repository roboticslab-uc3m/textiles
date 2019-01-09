#! /usr/bin/env python

from textiles.common.errors import DependencyNotInstalled
try:
    import yarp
except ImportError as e:
    raise DependencyNotInstalled(
        ("{}. (HINT: you need to install YARP for this to work," +
         "check https://github.com/roboticslab-uc3m/textiles for more info.)").format(e))

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
pos.setRefSpeed(1,10)
pos.setRefSpeed(0,10)

print "go to init"
pos.positionMove(1,-6)  # Down extreme w/o occlusions
pos.positionMove(0,-45)  # Right extreme
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass  # So much better than yarp.Time.delay(1)

print "go to left"
pos.positionMove(0,-30)  # Left enough
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

print "go upper"
pos.positionMove(1,-10)  # More up
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

print "go to right"
pos.positionMove(0,-45)  # Right extreme
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

print "go upper"
pos.positionMove(1,-12)  # More up
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

print "go to left"
pos.positionMove(0,-30)  # Left enough
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

print "go upper"
pos.positionMove(1,-12)  # More up
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

print "go to right"
pos.positionMove(0,-45)  # Right extreme
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

print "go to init"
pos.positionMove(1,-6)  # Down extreme w/o occlusions
pos.positionMove(0,-45)  # Right extreme
yarp.Time.delay(0.1)
while not pos.checkMotionDone(): pass

dd.close()

yarp.Network.fini()


