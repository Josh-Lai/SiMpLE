# ROS2 Node for SiMpLE 
A Simultaneous Localisation and Mapping implementation ported to a ROS2 node. The intention of this node is to provide localisation updates using the SiMpLE algorithm.
The algorithm is tuned via ROS2 parameters which are most easily changed via a [configuration file](https://roboticsbackend.com/ros2-yaml-params/).

## Dependencies
See `package.xml` for ROS2 Dependeencies

## Configuration Parameters
See the example launch file and associated configuration files in `launch/` and `config/` respectivley. These parameters control the ROS2 outputs and the SiMpLE algorithm functionality. 

See the [SiMpLE](https://doi.org/10.1177/02783649241235325) paper for more information regarding the algorithm!

## Usage
To build the package with `colcon` run 
```
colcon build --packages-up-to simple
``` 

This will build the package with all dependencies located in the `package.xml`
