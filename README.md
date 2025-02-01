# EMC Robotic Arm Control
Packages created to be used as a supplement to WaveShare open source files in order to mirror Roarm M2-S movement.

## Necessary Packages First
Follow instructions 1.* at https://github.com/waveshareteam/roarm_ws_em0 to install WaveShare packages and dependencies.

## EMC Developed Packages and Performing Teleoperation
Navigate within roarm_ws_em0/src and install emc_arm_controller folder. Build roarm_ws_em0/src in order to run ros2 command: \
&nbsp;&nbsp;&nbsp;&nbsp;run ros2 emc_controller masterArm \
which will read position data from port /dev/ttyUSB1 and publish to JointStates topic. This topic will be subscribed to and executed by arm connected to port /dev/ttyUSB0 if you run:\
&nbsp;&nbsp;&nbsp;&nbsp;ros2 run roarm_driver roarm_driver \

## Arduino File

getMasterPosition.ido is an Arduino C++ file that does not need to be imported to the ROS workspace.
