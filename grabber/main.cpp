
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

    FILE * pFile = fopen ("myfile.txt","w");

    Network yarp;
    if (!yarp.checkNetwork()) {
        printf("[fail] found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    }

    BufferedPort<ImageOf<PixelMono16> > inImg;
    inImg.open("/in");

    ImageOf<PixelMono16> *inYarpImg = inImg.read(false);
    while (inYarpImg==NULL) {
        inYarpImg = inImg.read(false);
        printf("No img yet...\n");
        yarp::os::Time::delay(1);
    };
    /*IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg->width(), inYarpImg->height()),
                                         IPL_DEPTH_16U, 1 );
    inIplImage = (IplImage *)inYarpImg->getIplImage();
    Mat inCvMat(inIplImage);*/
    //yarp::sig::file::write(&inYarpImg,"test.ppm");
    for (int i = 0;i<inYarpImg->width();i++)
    {
        for (int j = 0;j<inYarpImg->height();j++)
        {
            fprintf(pFile,"%d ",inYarpImg->pixel(i,j));
        }
        fprintf(pFile,"\n");
    }

   fclose (pFile);

    return 0;
}

