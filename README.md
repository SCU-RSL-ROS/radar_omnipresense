# OPS-ShortRangeRadar
Robotic Operating System (ROS) publisher and service for the OmniPreSense short range radar (originally OPS241-A)

You will need to download and install:

	[rapidJson]( https://github.com/Tencent/rapidjson)
	
	[LinuxCommConnection](https://github.com/RyanLoringCooper/LinuxCommConnection)
	
### Running the package with roslaunch
To run the package you will need to ensure that you have two OPS short range radar devices plugged into your machine. Once this is done utilize the roslaunch comman in the terminal. To do so type the following command in your terminal. Note: This was developed on the Lunar release of ROS on a Linux machine. We recommend installing the ROS Kinetic release or later on a Linux machine. 
```
roslaunch [pakcage_name] radar.launch
```	
This will run the package on the topics declare on the roslaunch file for this package. To view these topics please type the following command into a terminal.
```	
rostopic echo /radar_1/radar
```	
Then in another terminal please type the following command to view the topic that the second radar is publishing to.
```
rostopic echo /radar_2/radar
```
### Running one radar at a time and using the ROS service to activate FFT output. 		
To enable the radar devices to publish FFT data, this can most easily be done by running one radar at a time (i.e. not utilizing the roslaunch method above). for this case ensure a radar device is plugged into your machine. Open a new terminal and type the following to start the radar node for one device.
```
roscore
```	
Then in a new terminal type the follwing:
```
rosrun [package_name] radar_publisher
```
To enable the FFT output please open a new terminal and type the following command.
```
rosservice call /send_api_command "command: 'OF'"
```	
Then type in the terminal(can be same terminal as previous one.
```
rostopic echo [package_name] radar
```	


	
