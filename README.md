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

1. imageProcessing.py (src/leonao/libraries/imageProcessing.py): Run the file in a python 3.8 or newer terminal. Make sure the *DIRECTORY_TO_WATCH = "/home/hrsa/leonao/src/leonao/watchfolder"* is set correctly. 
For the Labs PC (A) you can source the *project_setup_sketcher.bash* file from the project root.
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

2. In python2 terminals you can run roscore, the nao bringup and *leonao.launch*. After some general startup you can press enter in the terminal (will see a prompt to do so) to start LeoNaos interaction with you. The rest of the interaction can be controlled through LeoNaos head buttons when prompted to do so (Front: Starting to take a picture and accepting the picture to start drawing, Back: Canceling the sketch and take a new picture)

### Optional: (Re-)calibrating the drawing canvas

The canvas usually does not have to be recalibrated between drawings or even startups, since the last calibration is stored in the `leonao/src/leonao/config/plane_config.pkl`. If the drawing canvas is moved relative to the robot, the plane has to be recalibrated. 

There are two options to calibrate the plane:
#### 1. Record 3 new calibration points and store them inside the `plane_config.pkl`:
To start the calibration run the following command (after `catkin_make` and sourcing `devel/setup.bash`):  
```console
roslaunch/rosrun .... do we still have the code for this? :/
``` 
You will get a prompt in the terminal that asks you if you want to calibrate the drawing plane. By answering with `y`, the calibration process will start. 
- Move the pen to the 1st point (origin) and press enter. 
- Move the pen to the 2nd point (the z-axis is defined by the direction from the 1st to the 2nd point) and press enter.
- Move the pen to the 3rd (take care that you choose a linearly independent point, the 3 points should build a big triangle) point and press enter to store the points in the `plane_config.pkl` and end the calibration process.

#### 2. Manually change the 3 calibration points in the `plane_config.pkl` and check if they lay on the drawing plane (by letting the robot move to these 3 points):  
To start the calibration run the following command (after `catkin_make` and sourcing `devel/setup.bash`): 
```console
roslaunch leonao setup_example.launch
``` 
You will get a prompt in the terminal that asks you if you want to calibrate the drawing plane. By answering with `y`, the calibration process will start. 
- The program loads the current `plane_config.pkl`. 
- By pressing enter the pen will be moved to the next point (it starts at origin/1st point) (origin).
- After one round you will be asked if you want to do another round.
- With this process you can check the calibration and manually adjust the `plane_config.pkl` or the drawing plane if needed.
<!--- #TODO-Ingo could you add how to do this? --->
