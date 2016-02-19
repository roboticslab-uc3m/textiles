
// yarpdev --device OpenNI2DeviceServer --depthVideoMode 4 --colorVideoMode 9 --noMirror
// yarp connect /OpenNI2/imageFrame:o /rgb:i
// yarp connect /OpenNI2/depthFrame:o /depth:i

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <stdio.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;

int main(int argc, char** argv) {

    Network yarp;
    if (!yarp.checkNetwork()) {
        printf("[fail] found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    }

    BufferedPort<ImageOf<PixelMono16> > inDepth;
    BufferedPort<ImageOf<PixelRgb> > inRgb;
    inDepth.open("/depth:i");
    inRgb.open("/rgb:i");

    Time::delay(0.5);

    Network::connect("/OpenNI2/depthFrame:o","/depth:i");
    Network::connect("/OpenNI2/imageFrame:o","/rgb:i");

    ImageOf<PixelMono16> *inYarpDepth = NULL;
    ImageOf<PixelRgb> *inYarpRgb = NULL;

    while (inYarpDepth==NULL) {
        inYarpDepth = inDepth.read(false);
        printf("No depth yet...\n");
    };
    //yarp::os::Time::delay(1);
    while (inYarpRgb==NULL) {
        inYarpRgb = inRgb.read(false);
        printf("No rgb yet...\n");
    };

    if(inYarpRgb->width()<400)
    {
        printf("PLEASE INCREASE SENSOR RESOLUTION!!!!\n");
        return 1;
    }
    /*IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg->width(), inYarpImg->height()),
                                         IPL_DEPTH_16U, 1 );
    inIplImage = (IplImage *)inYarpImg->getIplImage();
    Mat inCvMat(inIplImage);*/

    yarp::sig::file::write(*inYarpRgb,"rgb.ppm");
    //yarp::sig::file::write(*inYarpDepth,"depth.ppm");

    FILE * pFile = fopen ("depth.mat","w");
    for (int i = 0;i<inYarpDepth->width();i++)
    {
        for (int j = 0;j<inYarpDepth->height();j++)
        {
            fprintf(pFile,"%d ",inYarpDepth->pixel(i,j));
        }
        fprintf(pFile,"\n");
    }

   fclose (pFile);

    return 0;
}

