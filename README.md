# Simulator
A Gazebo simulator for Vortex NTNU's ROVs.

## Preparation
* `curl gazebo` to install Gazebo.
* `bash create_plugin.sh` to compile interface plugin.

## Launching
* `roslaunch Simulator simple_rov.launch` to start the simulator listening for rov_forces topic.
* `roslaunch vortex pc.launch` to connect to Vortex control system.
