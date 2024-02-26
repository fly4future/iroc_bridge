# iroc_bridge
 
## Functionality

* Once activated, the nodelet will listen mrs_uav_status message and translate it to JSON format

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

The call the services prepared in the terminal window.

## Package structure

See [ROS packages](http://wiki.ros.org/Packages)

* `src` directory contains all source files
* `include` directory contains all header files
* `launch` directory contains `.launch` files which are used to parametrize the nodelet. Command-line arguments, as well as environment variables, can be loaded from the launch files, the nodelet can be put into the correct namespace, config files are loaded, and parameters passed to the nodelet. See [.launch files](http://wiki.ros.org/roslaunch/XML)
* `config` directory contains parameters in `.yaml` files. See [.yaml files](http://wiki.ros.org/rosparam)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](http://wiki.ros.org/catkin/package.xml)
