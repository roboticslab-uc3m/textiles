Nice program to display a Kinect RGB and depth stream. Saving frames to files is supported.

This is heavily based on the glview.c example from the OpenKinect Project. Therefore, we are keeping the original contrib file in this folder as required by the OpenKinect original license.

Author: [David Estevez](https://github.com/David-Estevez)

Dependencies:

 * [libfreenect](https://github.com/OpenKinect/libfreenect/)
 * OpenGL and GLUT
   
       sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
 
 * pthreads-win32 (Windows)