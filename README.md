# uarm_kinect

In order to use the Kinect with this package, there are three libraries you will need to install in addition to the openni_tracker package: OpenNI, SensorKinect, and NITE. Follow the instructions below for the installation process.

First, there are a few dependencies to install:

```
sudo apt-get install git build-essential python libusb-1.0-0-dev freeglut3-dev openjdk-7-jdk
sudo apt-get install doxygen graphviz mono-complete
pip install pyuarm
```

Installing OpenNI 1.5.4
-----------------------

```
cd ~
git clone https://github.com/OpenNI/OpenNI.git
cd OpenNI
git checkout Unstable-1.5.4.0
cd Platform/Linux/CreateRedist
./RedistMaker
cd ../Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.4.0/ (if you are using 32-bit Linux, replace the 64 with 86)
sudo ./install.sh
```

Installing SensorKinect
-----------------------

```
cd ~
git clone https://github.com/avin2/SensorKinect
cd SensorKinect
cd Platform/Linux/CreateRedist
./RedistMaker
cd ../Redist/Sensor-Bin-Linux-x64-v5.1.2.1/ (same as above)
sudo ./install.sh
```

Installing NITE 1.5.2
---------------------

```
cd ~
git clone https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23.git
cd NITE-Bin-Dev-Linux-v1.5.2.23/x64
sudo ./install.sh
```

Installing openni_tracker (ROS package)
---------------------------------------

```
cd ~/your_catkin_ws/src/
git clone https://github.com/ros-drivers/openni_tracker.git
catkin build openni_tracker
```

Though the openni_tracker package was intended for Hydro, it seems to work fine on Kinetic.

Finally, download and build this package:

```
git clone https://github.com/daniel-s-ingram/uarm_kinect.git
catkin build kinect_pyuarm
source ~/your_catkin_ws/devel/setup.bash
roslaunch kinect_pyuarm kinect_uarm.launch
```
