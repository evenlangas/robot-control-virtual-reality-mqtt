# Robot Remote Control in Virtual Reality Using MQTT

This Unity project enables the remote control of a robot in virtual reality (VR) using the MQTT (Message Queuing Telemetry Transport) protocol. It provides a seamless interface for controlling robot movement, manipulating robotic arms or appendages, and receiving sensor data back from the physical robot.

Features

VR Robot Control: Intuitive control of a virtual robot representation within a Unity-based VR environment.
MQTT Communication: Leverages MQTT, a lightweight messaging protocol ideal for IoT devices and robotics, to transmit commands and potentially receive data.
Customizable Robot Model: Easily replace the default robot model with your own robot design.
Sensor Integration: Integration of sensor data from the physical robot back into the VR environment.

Video can be seen on the following link:
https://drive.google.com/file/d/1-PHIBYJ98Cfk90Qt5onyo6cpvmSN6yz8/view

## Getting started with Node-RED and ROS2

Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html).

Follow the installation guidelines for [Integration Service](https://integration-service.docs.eprosima.com/en/latest/installation_manual/installation_manual.html#installation-manual) from eProsima.

Further on, clone the [node-red-ros2-plugin repo](https://github.com/eProsima/node-red-ros2-plugin.git) to your home folder:

```
cd ~
git clone https://github.com/eProsima/node-red-ros2-plugin.git
```

Then complete the last steps:

```
cd ~/.node-red
npm install ~/node-red-ros2-plugin/
```
