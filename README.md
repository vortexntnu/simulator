# simulator
The repository for Vortex NTNU's ROV. Contains a ROS package for dynamics, a description folder for the ROV model, a Simulink model of the ROV, as well as a plugin for Gazebo.

## Launching
* `roslaunch simulator.launch` to launch the ROS node

## Dependencies

### Armadillo 
The [Armadillo](http://arma.sourceforge.net/) C++ linear algebra library

`sudo apt install libarmadillo-dev`

## Preferred workflow
* Create a feature branch out of `master` for each new feature, solved issue, significant refactor, etc.
* Open a pull request to have your branch merged back into `master`.
