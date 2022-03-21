# radar_omnipresense, a package for ROS
A Robotic Operating System (ROS) publisher and service for the OmniPreSense short range radar.
 
### Package Dependencies

In order to run (and build) this package, a system has to have the at least the ROS-base package.
* [ROS]( http://www.ros.org/install/ )

You can test if you have ROS installed correctly by attempting to run roscore, a required program for all subsequent examples.  If roscore is already running, it is safe to run roscore again and ignore any warnings about it already running. 
```
roscore
```

### Running the package with roslaunch
To run the package you will need to ensure that you have at least one OmniPreSense short range radar device plugged into your USB port(s).  Once this is done, use the roslaunch command in a shell terminal.  To do so, enter the following command. Note:  (This was developed on the Lunar release of ROS on a Linux machine, and tested with the ROS Kinetic release on Raspberry Pis running Ubuntu.  We recommend installing the ROS Kinetic release or later on a Linux machine.) 
```
roslaunch radar_omnipresense single_radar.launch
```	
or, if you have multiple radar modules, edit launch/multi_radar.launch as required and enter
```
roslaunch radar_omnipresense multi_radar.launch
```

This will run the package on the topics as declared in the .launch file you specified.  (For example, "radar_1")

#### Success?
Note: roslaunch stays running.  You will need to have another shell open to enter other commands while the radar_omnipresense node is running.  The unix utility "screen" is a very valuable utility if you are accessing the ROS linux instance over ssh, or more generally do not have the ability to have multiple terminals.

#### Errors?
Permission problems accessing the port (e.g. /dev/ttyACM0) are common.  If this happens, so long as you have sudo privileges to your system, either of two approaches to solving this problem can be followed.  
1) If your userId is not in the ```dialout``` group and the sensor is in the dialout group (investigate by ```ls -l /dev/ttyACM0```), run the following command:
```
sudo adduser `whoami` dialout
```
2) Alternately you can configure Linux to grant relaxed privileges to the sensor upon being plugged in.  This is done by making a udev rule.  Run the following command:
```
echo 'KERNEL=="ttyACM[01]", MODE="0666"' | sudo tee -a /etc/udev/rules.d/50-ACM.rules
```
There is relevant information posted (here)[https://omnipresense.github.io/knowledge-base/USB_issues], though ROS users might prefer keeping the name as /dev/ttyACM0  

### Viewing reports
To view these topics please type the following command into a terminal.
```
rostopic echo /radar_report
```
The above comman will for if you are specifically using the single_radar.launch file. If you are using the multi_radar. launch file, use the following commands. 
```	
rostopic echo /radar_1/radar_report
```	
Then, if running a second radar module, in another terminal please type the following command to view the topic that the second radar is publishing to.
```
rostopic echo /radar_2/radar_report
```

Note: If you only have one radar device but use the multi_radar.launch file, the roslaunch command will still work, but you will see "error 2 opening /dev/ttyACM#", and # will be the port the device is connected to. The one radar will still publish data to the topic and the associated service can still be used.

### Running the ROS service to send API calls
You can issue API commands to the sensor using the rosservice call.  For instance, you can change the sampling rate with the sample rate commands (e.g. ```SX,S2,SL```) 
We recommend the rosservice calls be sent in another ROS-initialized terminal so you can observe the sensor response in the rostopic terminal. 
```
rosservice call /send_api_command "command: 'S2'"
```

(If you find errors in this README, feel free to file an issue and we'll make corrections as needed, or issue a pull request if you have corrections you've tested.)






