# OPS-ShortRangeRadar
Robotic Operating System (ROS) publisher and service for the OmniPreSense short range radar (originally OPS241-A)

You will need to download and install:


* [rapidJson]( https://github.com/Tencent/rapidjson)
	
* [LinuxCommConnection](https://github.com/RyanLoringCooper/LinuxCommConnection)

	
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
Note: If you only have one radar device the roslaunch command will still work, but you will see "error 2 opening /dev/ttyACM#", and # will be the port the device is connected to. The one radar will still publish data to the topic and the associated service can still be used.
### Running the ROS service to activate and deactivate FFT output
If you would like for the radar devices to output the FFT data utilize the ROS service after you have executed the previous commands listed above. We recommend the following commands be typed into a new terminal. 
```
rosservice call /radar_#/send_api_command "command: 'OF'"
```
Where # would be equal to either 1 or 2 depending of which radar device you would like to output FFT data. If you would like both radar devices to output FFT data, just give the following commands:
```
rosservice call /radar_1/send_api_command "command: 'OF'"
rosservice call /radar_2/send_api_command "command: 'OF'"
```
This will enable FFT data to be output from both radar devices.

If you would like to turn the FFT data output off, then give the following command:
```
rosservice call /radar_#/send_api_command "command: 'Of'"
```
If both radar devices are outputing FFT data and you would like to turn the FFT data output off, the following commands with turn the FFT data output off:
```
rosservice call /radar_1/send_api_command "command: 'Of'"
rosservice call /radar_2/send_api_command "command: 'Of'"
```
### Running one radar at a time and using the ROS service to activate FFT output. 		
If you do not want to use the roslaunch command and file, the following commands will allow you to do so:
```
roscore
```	
Then in a new terminal type the follwing:
```
rosrun [package_name] radar_publisher
```
To see the data being published to the topic type: 
```	
rostopic echo /radar
```	
To enable the FFT output please open a new terminal and type the following command.
```
rosservice call /send_api_command "command: 'OF'"
```	
To disable the FFT output please type the following command: 
```
rosservice call /send_api_command "command: 'Of'"
```	



	
