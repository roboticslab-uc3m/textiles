// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "InCommandPort.hpp"

namespace teo
{

/************************************************************************/

void InCommandPort::onRead(Bottle& b) {
    printf("InCommandPort: %s\n",b.toString().c_str());
    if ((b.get(0).asString() == "go")||(b.get(0).asVocab() == VOCAB_GO))  // go //
    {
        Bottle* cvb = inCvPortPtr->read();
        printf("inCvPortPtr: %s\n",cvb->toString().c_str());
        int pxX0 = cvb->get(0).asInt();
        int pxY0 = cvb->get(1).asInt();
        double mmZ0 = cvb->get(2).asDouble();  // Points forward.
        int pxX1 = cvb->get(3).asInt();
        int pxY1 = cvb->get(4).asInt();
        double mmZ1 = cvb->get(5).asDouble();  // Points forward.

        double mmX0 = -(1000.0 * ( (pxX0 - DEFAULT_CX_D) * mmZ0/1000.0 ) / DEFAULT_FX_D);  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
        double mmX1 = -(1000.0 * ( (pxX1 - DEFAULT_CX_D) * mmZ1/1000.0 ) / DEFAULT_FX_D);  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
        double mmY0 = 1000.0 * ( (pxY0 - DEFAULT_CY_D) * mmZ0/1000.0 ) / DEFAULT_FY_D;  // Points down.
        double mmY1 = 1000.0 * ( (pxY1 - DEFAULT_CY_D) * mmZ1/1000.0 ) / DEFAULT_FY_D;  // Points down.

        KDL::Frame H_root_kinect;
        H_root_kinect.p.data[0] = 700; // x [mm]
        H_root_kinect.p.data[2] = 1205 - 55; // z [mm]
        H_root_kinect.M = KDL::Rotation::RotY(M_PI);  // 180 deg, to point down

        KDL::Frame H_kinect_point0, H_kinect_point1;
        H_kinect_point0.p.data[0] = mmX0;
        H_kinect_point0.p.data[1] = mmY0;
        H_kinect_point0.p.data[2] = mmZ0;
        H_kinect_point1.p.data[0] = mmX1;
        H_kinect_point1.p.data[1] = mmY1;
        H_kinect_point1.p.data[2] = mmZ1;

        printf("H_kinect_point0: %f %f %f\n",H_kinect_point0.p.x(),H_kinect_point0.p.y(),H_kinect_point0.p.z());
        printf("H_kinect_point1: %f %f %f\n",H_kinect_point1.p.x(),H_kinect_point1.p.y(),H_kinect_point1.p.z());

        KDL::Frame H_root_point0 = H_root_kinect * H_kinect_point0;
        KDL::Frame H_root_point1 = H_root_kinect * H_kinect_point1;

        printf("H_root_point0: %f %f %f\n",H_root_point0.p.x(),H_root_point0.p.y(),H_root_point0.p.z());
        printf("H_root_point1: %f %f %f\n",H_root_point1.p.x(),H_root_point1.p.y(),H_root_point1.p.z());

        KDL::Frame H_root_point0_safe( H_root_point0 );
        H_root_point0_safe.p.data[2] += 50;  // [mm]

        char c;

        stat();

        printf("Press enter to move home...\n");
        scanf("%c",&c);
        {
            double jointTargets[7] = {0,0,0,0,0,0,GRIPPER_CLOSE};
            jointsWithWait(jointTargets);
        }
        printf("Press enter to continue movement...\n");
        scanf("%c",&c);
        {
            double jointTargets[7] = {4.305,-0.949,2.636,0.087,-0.509,10.456,GRIPPER_CLOSE};
            jointsWithWait(jointTargets);
        }
        printf("Press enter to continue movement...\n");
        scanf("%c",&c);
        {
            double jointTargets[7] = {-32.495,-34.165,9.314,42.530,-1.476,12.917,GRIPPER_CLOSE};
            jointsWithWait(jointTargets);
        }
        printf("Press enter to continue movement...\n");
        scanf("%c",&c);
        {

            double jointTargets[7] = {-22.038,-53.848,15.992,50.527,-0.685,12.917,GRIPPER_CLOSE};
            jointsWithWait(jointTargets);
        }
        printf("Press enter to continue movement...\n");
        scanf("%c",&c);
        {
            double jointTargets[7] = {1.493,-50.685,0.966,71.704,0.000,12.390,GRIPPER_CLOSE};
            jointsWithWait(jointTargets);
        }
        printf("Press enter to continue movement...\n");
        scanf("%c",&c);
        {
            double jointTargets[7] = {34.358,-32.934,-29.595,71.792,-0.597,10.808435,GRIPPER_CLOSE};
            jointsWithWait(jointTargets);
        }

        /*movjWithWait(H_root_point0_safe);
        yarp::os::Time::delay(5);

        movjWithWait(H_root_point0);
        yarp::os::Time::delay(1);

        gripper(GRIPPER_CLOSE);

        movjWithWait(H_root_point0_safe);
        yarp::os::Time::delay(1);*/

        printf("Reached current end of program.\n");

    }
}

/************************************************************************/

void InCommandPort::movjWithWait(KDL::Frame& frame)
{
    KDL::Vector rotVector = frame.M.GetRot();
    double angle = frame.M.GetRotAngle(rotVector);  // Normalizes as colateral effect
    Bottle cmd,res;
    cmd.addVocab(VOCAB_MOVJ);
    cmd.addDouble(frame.p.x());
    cmd.addDouble(frame.p.y());
    cmd.addDouble(frame.p.z());
    cmd.addDouble(rotVector[0]);
    cmd.addDouble(rotVector[1]);
    cmd.addDouble(rotVector[2]);
    cmd.addDouble(angle);
    cmd.addString("wait");
    cmd.addInt(1);
    cartesianPortPtr->write(cmd,res);
    printf("[movj] got response: %s\n", res.toString().c_str());
}

/************************************************************************/

void InCommandPort::jointsWithWait(double* targets)
{
    iPositionControl->positionMove(targets);
}

/************************************************************************/

void InCommandPort::stat()
{
    Bottle cmd,res;
    cmd.addVocab(VOCAB_STAT);
    cartesianPortPtr->write(cmd,res);
    printf("[stat] got response: %s\n", res.toString().c_str());
}

/************************************************************************/

void InCommandPort::gripper(const int& value)
{
    printf("[gripper] %d.\n",value);
    iPositionControl->positionMove(6,value);
}

/************************************************************************/
}  // namespace teo
