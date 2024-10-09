# Robot Remote Control in Virtual Reality Using MQTT

This Unity project enables the remote control of a robot in virtual reality (VR) using the MQTT (Message Queuing Telemetry Transport) protocol. It provides a seamless interface for controlling robot movement, manipulating robotic arms or appendages, and receiving sensor data back from the physical robot.

Press below to see the video:

[![Video](https://img.youtube.com/vi/xSnyjV5pLr4/0.jpg)](https://www.youtube.com/watch?v=xSnyjV5pLr4)

A video of the execution of experiments presented in the research paper can be seen [here](https://drive.google.com/file/d/1-PHIBYJ98Cfk90Qt5onyo6cpvmSN6yz8/view).

## Getting started with Node-RED and ROS2

Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html).

Follow the installation guidelines for [Integration Service](https://integration-service.docs.eprosima.com/en/latest/installation_manual/installation_manual.html#installation-manual) from eProsima.

Further on, clone the [node-red-ros2-plugin repo](https://github.com/eProsima/node-red-ros2-plugin.git) to your home folder:

```bash
cd ~
git clone https://github.com/eProsima/node-red-ros2-plugin.git
```

Then complete the last steps:

```bash
cd ~/.node-red
npm install ~/node-red-ros2-plugin/
```

When running Node-RED, remember to source ROS2 and the Integration-Service workspace:

```bash
# Source ROS2
. /opt/ros/humble/setup.bash
# Source Integration-Services
. is-workspace/install/setup.bash
```

## MQTT Broker Communication Protocol

The messages being sent through the MQTT broker is in JSON format.

**Open and close gripper:**

Topic: /til-tak/drammen/production/line/gripper/control

```json
{"state":"close"}

{"state":"open"}
```

**Drive conveyor belt:**

Topic: /til-tak/drammen/production/line/conveyor/control

```json
{"speed":0.0}

{"speed":100.0}
```

**Sensor data from robot:**

/til-tak/drammen/production/line/robot/data

```json
{
	"joint_1": 145.34, 
	"joint_2": 145.34,	
	"joint_3": 145.34, 
	"joint_4": 145.34, 
	"joint_5": 145.34, 
	"joint_6": 145.34
}
```

**Control signals to robot:**

/til-tak/drammen/production/line/robot/control

```json
{
	"x": 145.34, 
	"y": 145.34, 
	"z": 145.34,
	"roll": 145.34, 
	"pitch": 145.34,	
	"yaw": 145.34 
}
```


## Virtual Reality User Manual

**Right or left side trigger** to move control proxy

**Right trigger** to send waypoints

Hold the **A button** to close the gripper

**Left trigger** controls conveyor belt with variable speed.
