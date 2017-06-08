# phantomx_reactor_arm

<h2>Description</h2>
The PhantomX Reactor Robot Arm is the first in Interbotix Labs' offering of Arbotix based research grade robotic arms. The Reactor Arm was designed with reach and agility in mind, but it still boasts considerable strength for an arm of its size. The PhantomX Reactor Robot Arm was designed with entry-level research and university use in mind, providing one of the highest featured arms on the market today while not breaking one's budget.

<h2>Supported Hardware</h2>

![Image of PhantomX reactor arm](http://www.trossenrobotics.com/resize/Shared/images/PImages/reactor/reactor-1a.jpg?bw=1000&bh=1000)


# phantomx_reactor_arm_description
This package contains the PhantomX reactor arm model (urdf, meshes, ...).

# phantomx_reactor_arm_controller
This package contains the configuration files for the controllers used by the model.
The arm can be controller either through the Arbotix-M controller or the USB2Dynamixel.

## Installation and configuration 

### Setting up the Arbotix-M board

In order to work with ROS it is necessary to upload the firmware into the Arbotix-M board.

* Download Arduino ide from https://downloads.arduino.cc/arduino-1.0.6-linux64.tgz
  * wget https://downloads.arduino.cc/arduino-1.0.6-linux64.tgz
  
* Extract it into a folder.
* Download the firmware archives from https://github.com/trossenrobotics/arbotix/archive/master.zip
  * wget https://github.com/trossenrobotics/arbotix/archive/master.zip
* Extract it into a folder like ~/Documents/Arduino
* Run arduino from the folder you extracted it previously
  * cd ~/Downloads/arduino-1.0.6
  * ./arduino
* Once Arduino IDE is running, change the Sketchbook folder location to /Documents/Arduino/arbotixmaster or the one you extracted it previously.
  * File->Preferences->Sketchbook Location
  * Tools->Board->Arbotix
  * Tools->Serial Port->/dev/ttyUSBX
  * File->Sketchbook->Arbotix Sketches ->ros
  * Verify + Upload
* The Arbotix is ready to work with ROS!!

### Downloading the package

clone the repo into your workspace and compile it.
```
git clone https://github.com/RobotnikAutomation/widowx_reactor_arm.git
```
### Creating the udev rule for the device

#### For the Arbotix-M and USB2Dynamixel

In the phantomx_reactor_arm_controller/config folder there's the file 57-reactor_arbotix.rules/reactor_usb2dynamixel.rules. You have to copy it into the /etc/udev/rules.d folder.

```
sudo cp 57-reactor_arbotix.rules /etc/udev/rules.d
```

You have to set the attribute ATTRS{serial} with the current serial number of the ftdi device

```
udevadm info -a -n /dev/ttyUSB0 | grep serial 
```
Once modified you have to reload and restart the udev daemon

```
sudo service udev reload
sudo service udev restart
sudo udevadm trigger
```

### Running the controller

#### Arbotix-M 

* For the arm with wrist
```
roslaunch phantomx_reactor_arm_controller arbotix_phantomx_reactor_arm_wrist.launch
```
* For the arm without wrist
```
 roslaunch phantomx_reactor_arm_controller arbotix_phantomx_reactor_arm_no_wrist.launch
```
#### USB2Dynamixel Controller

* Run the controller
```
roslaunch phantomx_reactor_arm_controller dynamixel_phantomx_reactor_arm_wrist.launch 
```
* Load description and run the robot state publisher
```
roslaunch phantomx_reactor_arm_description phantomx_reactor_wrist_load_description.launch
```
* Run Moveit plugin for RVIZ
Moveit only works (for now) for the USB2Dynamixel option.
If you run the moveit demo, the previous launch with the description is not necessary.
Runnig demo with fake controllers (no real arm needed).
```
roslaunch phantomx_reactor_arm_moveit_config demo.launch
```
Running demo with the real controllers
```
roslaunch phantomx_reactor_arm_moveit_config demo_real.launch
```

### Commanding the controller 

#### Arbotix-M 


**Be carefull with dependency of j2-j3 and j4-j5!!! Try not to command directly the joints topics processed by the arbotix node.**

Use the command topic subscribed by the node phantomx_reactor_parallel_motor_joints.py

```
rostopic pub /phantomx_reactor_controller/shoulder_yaw_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /phantomx_reactor_controller/shoulder_pitch_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /phantomx_reactor_controller/elbow_pitch_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /phantomx_reactor_controller/wrist_yaw_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /phantomx_reactor_controller/wrist_pitch_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /phantomx_reactor_controller/gripper_revolute_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /phantomx_reactor_controller/gripper_prismatic_joint/command std_msgs/Float64 "data: 0.0"
```
#### USB2Dynamixel Controller

```
rostopic pub /shoulder_yaw_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /shoulder_pitch_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /elbow_pitch_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /wrist_yaw_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /wrist_pitch_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /gripper_revolute_joint/command std_msgs/Float64 "data: 0.0"
rostopic pub /gripper_prismatic_joint/command std_msgs/Float64 "data: 0.0"
```
