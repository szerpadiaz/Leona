# LeoNao
*LeoNao Da Vinci* is a student project in the Humanoid Robotic Systems lecture of the TUM within the Winter Semester 2022/23.

###  Students of Group A:
* Sergio Zerpa
* Ingo Blakowski
* David Gastager
* Michael Sodamin

You can get access to the project report via this link: https://sharelatex.tum.de/read/mjjtkfrcrbvy 

## General Setup

The main difference to regular setups is that you have to run one python file in a python 3 environment (*imageProcessing.py*) compared ot python 2.7 for the rest. 

The two different python environments communicate via the watchfolder (leonao/src/leonao/watchfolder). This also allows to simply drop .jpg files with the correct name (*detect_face.jpg* or *sketch_face.jpg*) and if the *imageProcessing.py* runs, the results will be written in a txt and pickle file respectively (result.txt and sketcher_results.pkl). We left in some examples for reference.

## How to make the code run

##### 1. imageProcessing.py (src/leonao/libraries/imageProcessing.py):
Run the file in a python 3.8 or newer terminal. Make sure the `DIRECTORY_TO_WATCH = "/home/hrsa/leonao/src/leonao/watchfolder"` is set correctly. 
For the Labs PC (A) you can source the `project_setup_sketcher.bash` file from the project root.

Required packages are:

    - opencv
    - mediapipe
    - numpy
    - scipy
    - matplotlib
    - torch
    - torchvision
    - watchdog
    - pillow / PIL
    - tkinter
    - potrace   
(Exact requirements available in Test-Setup and Archive/sketcher/requirements.txt)

After setting up the terminal with:
```console
source project_setup_sketcher.bash
```
you can run:
```console
python imageProcessing.py
```
to run the image processing pipeline.

##### 2. Run leonao.launch:
In order to run this make sure you installed `KDL` (https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md) and `kdl_parser` (http://wiki.ros.org/kdl_parser).

Then run the following commands in python2 terminals. You first need to bring up the nao:
```console
roslaunch nao_bringup nao_full_py.launch
```
To start our program you can then run the following in another terminal:
```console
roslaunch leonao leonao.launch
```
After some general startup you will get a prompt in the terminal that asks you if you want calibrate the drawing plane (see next section). By answering 'n' the LeoNao will load the last configuration and starts main program. The rest of the interaction can be controlled through LeoNaos head buttons when prompted to do so (Front: Starting to take a picture and accepting the picture to start drawing, Back: Canceling the sketch and take a new picture)

## Optional: (Re-)calibrating the drawing canvas

The canvas usually does not have to be recalibrated between drawings or even startups, since the last calibration is stored in the `leonao/src/leonao/config/plane_config.pkl`. If the drawing canvas is moved relative to the robot, the plane has to be recalibrated. 

There are two options to calibrate the plane:
##### 1. Set the 3 points manually (more robust than method 2):
Inside the `get_configuration` function of `canvas.py` you can manually set the points and check if they lie on the drawing plane (by letting the robot move to these points). You can then either adjust the points or the drawing plane.

To start the calibration run the following command (after `catkin_make` and sourcing `devel/setup.bash`): 
```console
roslaunch leonao leonao.launch
``` 
You will get a prompt in the terminal that asks you if you want to calibrate the drawing plane. By answering with `y`, the calibration process will start. 
- The program loads the points you set in the `get_configuration` function of `canvas.py`. 
- Everytime you press enter the pen will be moved to the next point (it starts at the 1st point).
- After one round you will be asked if you want to do another round.
- With this process you can check the calibration and manually adjust the 3 points in the `get_configuration` function or the drawing plane if needed. If you decide to adjust the points, you need to re-start the program to see the changes. 

##### 2. Record 3 new calibration points:
**_NOTE:_** This method is currently not used. If you want to use this method, you need to uncomment the current `get_configuration` function and add the uncommented `get_configuration` + `get_current_point` functions in the `canvas.py`).

To start the calibration run the following command (after `catkin_make` and sourcing `devel/setup.bash`): 
```console
roslaunch leonao leonao.launch
```
You will get a prompt in the terminal that asks you if you want to calibrate the drawing plane. By answering with `y`, the calibration process will start. 
- Move the pen to the 1st point (origin) and press enter. 
- Move the pen to the 2nd point (the z-axis is defined by the direction from the 1st to the 2nd point) and press enter.
- Move the pen to the 3rd point (take care that you choose a linearly independent point, the 3 points should build a big triangle) and press enter to store the points in the `plane_config.pkl` and end the calibration process.


