#! /usr/bin/env python

print "WARNING: only works with RaveBot in YARP yarpmods instead of in ASIBOT RlPlugins"

import yarp

yarp.Network.init()
if yarp.Network.checkNetwork() != True:
    print "[error] Please try running yarp server"
    quit()
options = yarp.Property()
options.put('device','ravebot')
dd = yarp.PolyDriver(options)

pos = dd.viewIPositionControl()
pos.setPositionMode()

print "test positionMove(1,-35)"
pos.positionMove(1,-35)

print "test delay(5)"
yarp.Time.delay(5)

enc = dd.viewIEncoders()
v = yarp.DVector(enc.getAxes())
enc.getEncoders(v)
v[2]

vel = dd.viewIVelocityControl()
vel.setVelocityMode()
print "test velocityMove(0,10)"
vel.velocityMove(0,10)

print "test delay(5)"
yarp.Time.delay(5)

yarp.Network.fini()

