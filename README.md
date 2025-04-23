# iroc_bridge
 *  Documentation: [Signaling Protocol Specification.](https://fly4future.github.io/iroc_bridge/)
## Functionality

* Once activated, the nodelet listens to the `uavX/mrs_uav_status/uav_status` topic and translates the messages from ROS to a JSON format.
* Documentation of the relevant message type `mrs_msgs/UavStatus` is available [here](https://ctu-mrs.github.io/mrs_msgs/msg/UavStatus.html).

## Dependencies 
You must have installed the MRS system on your PC (ROS1).
The steps are in the [README](https://github.com/ctu-mrs/mrs_uav_system/blob/master/README.md).

### Ubuntu 20.04
The recommended way is to use native installation and a stable version
https://github.com/ctu-mrs/mrs_uav_system/blob/master/README.md#native-installation.  

### Other OS
https://github.com/ctu-mrs/mrs_singularity 

## How to run
You can run the whole simulation, including the iroc_bridge package, using the prepared tmux session

```bash
./tmux/one_drone/start.sh
```

There are more sessions ready to be used.

We prepared sessions running mrs_multirotor_simulator (a lightway simulator that allows to run many drones at once without the high computation requirements).
Those sessions have `mrs_` appendix in the names of the folders. For example, session for the one drone:

```bash
./tmux/mrs_one_drone/start.sh
```

NOTE: Not all sensor data are simulated in the mrs_multirotor_simulator!

The call the services prepared in the terminal window.

## How to control the drones using the iroc_bridge
When the iroc_bridge is running and drones are ready you can call takeoff or land for all available drones using a web browser by visiting the following links:

```url
http://localhost:8080/takeoff_all
```

```url
http://localhost:8080/land_all
```

## Package structure

See [ROS packages](http://wiki.ros.org/Packages)

* `src` directory contains all source files
* `include` directory contains all header files
* `launch` directory contains `.launch` files which are used to parametrize the nodelet. Command-line arguments, as well as environment variables, can be loaded from the launch files, the nodelet can be put into the correct namespace, config files are loaded, and parameters passed to the nodelet. See [.launch files](http://wiki.ros.org/roslaunch/XML)
* `config` directory contains parameters in `.yaml` files. See [.yaml files](http://wiki.ros.org/rosparam)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](http://wiki.ros.org/catkin/package.xml)
* `tmux` directory contains prepared tmux sessions  
