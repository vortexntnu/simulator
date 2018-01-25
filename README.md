# Simulator
A Gazebo simulator for Vortex NTNU's ROVs.

## Preparation
* `curl -ssL http://get.gazebosim.org | sh` to install Gazebo.

## Launching
* `roslaunch Simulator rov.launch` to start the simulator listening for rov_forces topic.
* `roslaunch vortex pc.launch` to connect to Vortex control system.
