# Textiles: Manipulation and Perception of Deformable Objects

<br><br>
<img src="images/roboticslab.png" height="100px" align= "left"> <img src="images/uc3m.png" height="100px" align="right"><br><br><br><br>

Main repository for the research related to textile perception and manipulation at the University Carlos III of Madrid. The Textiles repository currently hosts two main lines of research:

## Garment Unfolding
In [[1]](#1) we developed a garment-agnostic algorithm to unfold clothes that works using 3D sensor information. The depth information provided by the sensor is converted into a grayscale image. This image is segmented using watershed algorithm. This algorithm provide us with labeled regions, each having a different height. In this labeled image, we assume that the highest height region belongs to the fold. Starting on this region, and ending in the garment border, tentative paths are created in several directions to analyze the height profile. For each profile, a bumpiness value is computed, and the lowest one is selected as the unfolding direction. A final extension on this line is performed to create a pick point on the fold border, and a place point outside the garment. In [[2]](#2) we offer an extended description of the same original algorithm. 

Later on, we wrote about future trends we expect in this field [[3]](#3).

We have been developing this algorithm and extend its application to other kind of robots, such as industrial manipulators. A novel enhanced version of the algorithm with extended experimental validation has been sent and is currently under review.

## Garment Ironing
We have developed an algorithm that allows a humanoid robot to iron garments with unmodified human tools. This work has been submitted for peer-reviewed publication and it is currently under review.

# What can I find in this repository?
Structure of the repository:

* **/images**
* **/ironing** - robotic garment ironing applications developed with [TEO the humanoid robot](https://github.com/roboticslab-uc3m/teo-main)
* **/unfolding** - robotic garment unfolding applications developed with [TEO the humanoid robot](https://github.com/roboticslab-uc3m/teo-main).
* **/unfolding-industrial** - robotic garment unfolding applications developed with industrial robots. (Under development).

# What are the requirements to install this software?

For the base installation:

* [OpenCV 3](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-opencv.md#install-opencv-3-with-contrib-and-python-3-support). If you want to run the code in `textiles.ironing.perception.Li`, you will need to install a custom [OpenCV 3](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-opencv.md#install-opencv-3-with-contrib-python-3-support-and-fix-for-svm_load) with support for SVMs.

* [YARP](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md#install-bindings). just make sure you install it with the Python 3 bindings.

If you want to use the guis (`textiles_unfolding_evaluation_gui`):

* **PySide 1**: run `sudo apt install python3-pyside`

If you want to use the apps for `industrial-manipulation` (with ABB robots):

* [open_abb](https://github.com/roboticslab-uc3m/open_abb) 


## Publications
Here you can find listed all publications related to the code hosted in this repository. 

<a id="1">[1]</a> David Estevez, Juan G. Victores, Santiago Morante, Carlos Balaguer. Towards Robotic Garment Folding: A Vision Approach for Fold Detection. International Conference on Autonomous Robot Systems and Competitions (ICARSC). 2016. [[PDF]](http://roboticslab.uc3m.es/roboticslab/sites/default/files/estevez2016towards-preprint.pdf) [[poster]](http://www.slideshare.net/JuanGVictores/estevez2016towardsposter) [[bib]](doc/bib/estevez2016towards.bib) [[URL]](http://icarsc2016.ipb.pt/docs/ProgramaICARSC.pdf)

<a id="2">[2]</a> David Estevez. Towards Robotic Clothes Folding: A Garment-Agnostic Unfolding Algorithm. Master's Thesis. 2016. [[PDF]](https://github.com/David-Estevez/master-thesis/blob/develop/estevez2016msc_thesis.pdf) [[slides]](http://www.slideshare.net/DavidEstevez11/estevez2016mscpresentation) [[bib]](doc/bib/estevez2016msc.bib) [[URL]](https://github.com/David-Estevez/master-thesis)

<a id="3">[3]</a> David Estevez, Juan G. Victores, Carlos Balaguer. Future Trends in Perception and Manipulation for Unfolding and Folding Garments. Open Conference on Future Trends in Robotics (RoboCity 2016). 2016. [[PDF]](http://roboticslab.uc3m.es/roboticslab/sites/default/files/estevez2016future-preprint.pdf) [[slides]](http://www.slideshare.net/JuanGVictores/estevez2016futurepresentation) [[bib]](doc/bib/estevez2016future.bib)  [[URL]](http://roboticslab.uc3m.es/roboticslab/book/robocity16-open-conference-future-trends-robotics-1)
