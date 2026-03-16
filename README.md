3D Systems Geomagic Touch ROS Driver
============

ROS Packages for connecting *one* or *more* 3D Systems Geomagic Touch (previously known as Phantom Omni) haptic devices, **USB** & **HID** versions.

For **HID Devices**, please read this [issue](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers/issues/17#issue-3540716107) and follow the guide below.

This repository has been forked from the original repository by Francisco Suárez Ruiz, [http://fsuarez6.github.io](http://fsuarez6.github.io) for the Sensable PHANToM haptic device (https://github.com/fsuarez6/phantom_omni).

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). 

## Installation

### 1. Install Dependencies

```bash
sudo apt-get install --no-install-recommends freeglut3-dev g++ libdrm-dev libexpat1-dev libglw1-mesa libglw1-mesa-dev libmotif-dev libncurses5-dev libraw1394-dev libx11-dev libxdamage-dev libxext-dev libxt-dev libxxf86vm-dev tcsh unzip x11proto-dri2-dev x11proto-gl-dev
```

### 2. Download and Extract OpenHaptics and Haptic Device Drivers:

Download drivers and OpenHaptics [here](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US) by clicking on the corresponding links.

### 3. Install OpenHaptics

In your ``Downloads`` folder there should be zipped archive named ``openhaptics_3.4-0-developer-edition-amd64.tar.gz``. Unpack the archive and execute the following
commands from terminal:
```bash
cd ~/Downloads/openhaptics_3.4-0-developer-edition-amd64/
sudo ./install
# OpenHaptics gets installed in the following directory
/opt/OpenHaptics/ 
```
**Note:** The folder also contains a readme file. If something fails during the installation, please refer to it.

### 4. Install Geomagic Touch Drivers:

In the ``Downloads`` folder on your PC there should be a zipped archive named ``TouchDriver_2025_12_10+1``. Unpack the archive and execute the following commands
from terminal:
```bash
cd ~/Downloads/TouchDriver_2025_12_10+1/TouchDriver_2025_12_10/
./install_haptic_driver
```
This should install the drivers for the Geomagic Touch.

**Note:** The version of the drivers may change in time. Please adjust the instructions accordingly to the version of the drivers which you download.

### 5. **(Only for 64-bit Systems)** Create Symbolic Links to OpenHaptics SDK Libraries:
```bash
sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11.0.1 /usr/lib/libraw1394.so.8
sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
sudo ln -s /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0
sudo ln -s /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0 
```

### 6. Device Setup:

To complete the setup of the device we need to modify the rules contained in ``/etc/udev/rules.d/99-3dsystems.rules``.

To do so, open a terminal and type the following command:
```bash
sudo gedit /etc/udev/rules.d/99-3dsystems.rules
```
This should open a text editor with the device rules. Delete everyhting found in that file and replace it with the following
new rules:
```bash
# ---- USB device node (libusb access) ----
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
  ATTR{idVendor}=="XXXX", ATTR{idProduct}=="YYYY", \
  MODE="0660", GROUP="plugdev", TAG+="uaccess"

# Disable autosuspend (prevents disconnect on app exit)
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="XXXX", ATTR{idProduct}=="YYYY", \
  TEST=="power/control", ATTR{power/control}="on"

# ---- HID raw node (hidraw access + stable name) ----
ACTION=="add", SUBSYSTEM=="hidraw", KERNEL=="hidraw*", \
  ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", \
  MODE="0660", GROUP="plugdev", SYMLINK+="geomagic_touch", TAG+="uaccess"
```
Remember to substitute the correct ``idVendor`` and ``idProduct`` for your device in place of `XXXX` and `YYYY`.

You can find the vendor and product id by plugging in your device and using the ``lsusb`` command.

Save and close the rules file and update the rules with the following commands:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# Reboot the system to make changes
sudo reboot
```

### 7. Testing the Device

Before connecting the device via USB, remove all other external HID devices from your USB ports (mouses, joysticks, etc.) as these may cause
conflicts with the haptic feedback of the Geomagic Touch as explained in the [Troubleshooting](#troubleshooting) section of this readme.

To test the device run the following command in a terminal:
```bash
./Downloads/TouchDriver_2025_12_10+1/TouchDriver_2025_12_10/bin/TouchCheckup 
```

This opens up a GUI which allows you to test and see data being logged from the Touch as well as to test haptics.

## Usage with ROS

1. Create a ros2 [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html),
and clone this repository inside the ``src`` folder. Then, build and source your workspace:

```bash
cd ~/<ROS_workspace>/src
git clone https://github.com/AleBarte/Geomagic_Touch_ROS2_Drivers.git
cd ..
colcon build --symlink-install
source install/setup.bash 
```

2. Start the node with the following command:
```bash
ros2 run omni_common master
```

Data from the haptic device can be read from the following topics:

  /touch0/buttons
  
  /touch/force_feedback
  
  /touch/joint_states
  
  /touch0/pose

  /touch0/twist
  
  /touch/state 


## Resources

https://3dsystems.teamplatform.com/pages/102863?t=fptvcy2zbkcc

https://fsuarez6.github.io/projects/geomagic-touch-in-ros/

https://github.com/fsuarez6/phantom_omni

http://dsc.sensable.com/viewtopic.php?t=5730&sid=9866fe798e24bc745fdb7fce08ee99eb

**Old device drivers** https://drive.google.com/drive/folders/1WJY6HpdtGh5zeyASfb4FYJFFG-QGItd6?usp=sharing

## Troubleshooting
1. On ubuntu 22.04 with the latest device drivers, the haptic rendering of the Touch seems to be susceptible to the presence of other
HID devices (e.g. external mouse). When commanding a force to the touch, if the latter starts to vibrate or shake violently try to unplug
all other usb devices from your PC and reconnect the haptic device. This should fix the issue.

## Citation

Please cite these papers in your publications if this repository helps your research.

```
@INPROCEEDINGS{8941790,
  author={Mathur, Bharat and Topiwala, Anirudh and Schaffer, Saul and Kam, Michael and Saeidi, Hamed and Fleiter, Thorsten and Krieger, Axel},
  booktitle={2019 IEEE 19th International Conference on Bioinformatics and Bioengineering (BIBE)}, 
  title={A Semi-Autonomous Robotic System for Remote Trauma Assessment}, 
  year={2019},
  volume={},
  number={},
  pages={649-656},
  doi={10.1109/BIBE.2019.00122}}
  
@inbook{doi:10.1137/1.9781611975758.2,
author = {B. Mathur and A. Topiwala and H. Saeidi and T. Fleiter and A. Krieger},
title = {Evaluation of Control Strategies for a Tele-manipulated Robotic System for Remote Trauma Assessment},
booktitle = {2019 Proceedings of the Conference on Control and its Applications (CT)},
chapter = {},
pages = {7-14},
doi = {10.1137/1.9781611975758.2},
URL = {https://epubs.siam.org/doi/abs/10.1137/1.9781611975758.2},
eprint = {https://epubs.siam.org/doi/pdf/10.1137/1.9781611975758.2}}
```



