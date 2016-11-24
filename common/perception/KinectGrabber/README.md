Nice program to display a Kinect RGB and depth stream. Saving frames to files is supported.

This is heavily based on the glview.c example from the OpenKinect Project. Therefore, we are keeping the original contrib file in this folder as required by the OpenKinect original license.

Author: [David Estevez](https://github.com/David-Estevez)

## Dependencies:

 * [libfreenect](https://github.com/OpenKinect/libfreenect/)
 * OpenGL and GLUT
   
       sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
 
 * pthreads-win32 (Windows)

## Commands

| Key | Command |
| --- | ------  |
| ESC | Exit    |
| w   | Increment tilt (up to 30º) |
| s   | Reset title (Set to 0º) |
| f   | Select video mode (RGB, YUV or IR8bit) |
| x   | Decrement titl (up to -30 º) |
| e   | Toggle auto exposure (on/off) |
| b   | Toggle auto white balance (on/off) |
| r   | Toggle raw color (on/off) | 
| m   | Toggle mirror image (on/off) |
| n   | Toggle near mode (on/off) |
| +   | Increment IR brightness (+2units) |
| -   | Decrement IR brightness (-2units) |
| 1   | LED mode: green |
| 2   | LED mode: red |
| 3   | LED mode: yellow |
| 4   | LED mode: blink green |
| 5   | LED mode: blink green |
| 6   | LED mode: blink red yellow |
| 0   | LED mode: off |
| o   | Toogle camera rotation (on/off) |
| g   | Save RGB image to file |
