/*Copyright 2018 Santa Clara University Robotic Systems Lab 
Licensed under the Educational Community License, Version 2.0 (the "License"); 
you may not use this fileexcept in compliance with the License. 
You may obtain a copy of the License at

http://opensource.org/licenses/ECL-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
License for the specific language governing permissions and limitations under
the License.
*/

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "radar_omnipresense/radar_data.h"
#include "radar_omnipresense/SendAPICommand.h"
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream> 
#include <string>
#include <thread>
#include <LinuxCommConnection/CommConnection.h>
#include <LinuxCommConnection/SerialConnection.h>
#include "rapidjson/document.h"
using namespace rapidjson;

SerialConnection * con;

//#########################################################################################################################################################//
//#########################################################################################################################################################//

//service function to send api commands to radar device
bool api(radar_omnipresense::SendAPICommand::Request &req, radar_omnipresense::SendAPICommand::Response &res) 
{
  res.response = "false";
  if (req.command == "Oj")
  {
    ROS_INFO("Cannot turn JSON mode off");
    return true;
  }
  //writes the api input to the serial port
  con->write(std::string(req.command.c_str())); 
  con->clearBuffer();
  //output for user to see that correct input was sent to the radar
  ROS_INFO("API command sent: %s", req.command.c_str()); 
  con->waitForData();
  con->waitForData();
  std::string command_response = con->readString();
  ROS_INFO("Command_response is: %s", command_response.c_str());
  std::size_t found_open_brace = command_response.find("{");
  std::size_t found_close_brace = command_response.find("}");
  bool open_brace = found_open_brace == std::string::npos;
  bool close_brace = found_close_brace == std::string::npos;
  if ((open_brace) && (close_brace))
  {
    ROS_INFO("ERROR: Response not recieved");
    return true;
  }
  std::string whole_msg = command_response.substr(found_open_brace,found_close_brace+1);
  //default template parameter uses UTF8 and MemoryPoolAllocator. //creates a document DOM instant called doc
  Document doc; 
  //doc.Parse(msg.data.c_str()); //parsing the json string msg.data with format{"speed":#.##,"direction":"inbound(or outbound)","time":###,"tick":###}
  doc.Parse(whole_msg.c_str());
  //case for when the radar outputs the JSON string {"OutputFeature":"@"}. This is not compatible with parsing into the message.
  if (doc.HasMember("OutputFeature")) 
  {
    ROS_INFO("Recieved: %s", whole_msg.c_str());
    res.response = "true";
  }
  return true;
}

//#########################################################################################################################################################//
//#########################################################################################################################################################//

void process_json(radar_omnipresense::radar_data *data, std::string single_msg, std::string serialPort)
{
  //default template parameter uses UTF8 and MemoryPoolAllocator. //creates a document DOM instant called document
  Document document; 
 	//document.Parse(msg.data->c_str()); //parsing the json string msg.data with format{"speed":#.##,"direction":"inbound (or outbound)","time":###,"tick":###}
 	document.Parse(single_msg.c_str());
 	ROS_INFO("Msg passed to parser is %s", single_msg.c_str());
 	assert(document.IsObject());
 	//ROS_INFO("Passed assertion");
 	//case for when the radar outputs the JSON string {"OutputFeature":"@"}. This is not compatible with parsing into the ROS message.
 	if (document.HasMember("OutputFeature")) 
	{
    ROS_INFO("OutputFeature");
		return;
	}
	
	bool fft = document.HasMember("FFT");
	bool dir = document.HasMember("direction");
	//fills in info.direction with a converted c++ string and only does so if direction is not empty.
	if (dir)  
	{
    //define a constant character array(c language)
		const char* direction; 
	  //Place the string(character array) of the direction into const char* direction
	 	direction = document["direction"].GetString();  
		std::string way(direction);
		data->direction = way;
		//accesses the decimal value for speed and assigns it to info.speed	
		data->speed = document["speed"].GetFloat();   
		//accesses the numerical value for time and assigns it to info.time
		data->time = document["time"].GetInt(); 
		//accesses the numerical value for tick and assigns it to info.tick
		data->tick = document["tick"].GetInt();	
		//place holder for field sensorid
		data->sensorid = serialPort; 
		//place holder for field range for field that will be added soon
		data->range = 0;
		//place holder for field that will be added soon
		//info.angle = 0;
		//place holder for field
		data->objnum = 1;
		data->metadata.stamp = ros::Time::now(); 
	}
	//indexes and creates fft field for publishing.
	else if (fft)
	{
    for (int i = 0; i < document["FFT"].Size(); i++)
		{
      //FFT is an array of 1x2 array, each element represent a different channel. Either i or q.
			const Value& a = document["FFT"][i].GetArray();  
			data->fft_data.i.push_back(a[0].GetFloat());
			data->fft_data.q.push_back(a[1].GetFloat());
 		}	
	}
	else 
	{
    ROS_INFO("ERROR: Unsupported message type.");
	}
}
//#########################################################################################################################################################//
//#########################################################################################################################################################//

std::string getMessage(CommConnection *connection) 
{
  std::string msg;
	bool startFilling = false; 
	while(true) 
	{
		if(connection->available()) 
		{
      char c = connection->read();
			if(c == '{') 
			{
				if (startFilling)
				{
					return std::string();
				}
        startFilling = true;
			}
			else if(c == '}' && startFilling) 
			{
				msg += c;
				break;
			}
			if(startFilling)
			{
				msg += c;
			}
		}
	}
	if (msg.find("direction") == std::string::npos && msg.find("FFT") == std::string::npos)
	{
			return std::string();
			ROS_INFO("msg is does not have direction or FFT");
	}
	else
	{
		return msg;
	}
}

//#########################################################################################################################################################//
//#########################################################################################################################################################//

int main(int argc, char** argv)
{
  //the name of this node: radar_publisher
  ros::init(argc, argv, "radar_publisher"); 
  ros::NodeHandle nh;//("~");
  std::string serialPort;
  // sets serialPort to "serialPort". "serialPort" is defined in package launch file. "/dev/ttyACM0" is the default value if launch does not set 			  		"serialPort" or launch is not used.
  nh.param<std::string>("serialPort", serialPort,"/dev/ttyACM0");
  //the node is created. node publishes to topic "radar" using radar_omnipresense::radar_data messages and will buffer up to 1000 messages before beginning to throw 		away old ones. 																																								     		
  ros::Publisher radar_pub = nh.advertise<radar_omnipresense::radar_data>("radar",1000); 
  //the service "send_api_commands" is created and advertised over ROS
  ros::ServiceServer radar_srv = nh.advertiseService("send_api_command", api);  
  //ROS loop rate, currently sent to 60Hz.
  ros::Rate loop_rate(1000); 
  //Open USB port serial connection for two way communication
  SerialConnection connection = SerialConnection(serialPort.c_str(), B19200, 0);  
  con = &connection;
  //continues the while loop as long as ros::ok continues to continue true
  int count = 0;
  while (ros::ok())
  {
  	bool connected = connection.isConnected();
  	if (connected == 1)
  	{
  		if (count == 0)
  		{
				//assuming radar is being started with no fft output.
				//Forces KeepReading to continously read data from the USB serial port "connection".
				//Then begin reading. Then set radar device to output Json format  data
				//string of format {"speed":#.##,"direction":"inbound(or outbound)","time":###,"tick":###}. 
				//Pause 100ms. Sets radar device to output FFT msg 
				//{"FFT":[ [#.###, #.###],[#.###, #.###],............[#.###, #.###],[#.###, #.###] ]} pause 100ms
				ROS_INFO("setting radar settings");
				connection.clearBuffer();
				bool KeepReading = true; 
				connection.begin();
				connection.write("OJ");
				connection.write("Of"); 
				connection.clearBuffer();
			}
			ROS_INFO("Connected");
			radar_omnipresense::radar_data info; 
			radar_omnipresense::radar_data info_out;
			//creats an instant of the radar_data structure named info. format: 
			//package_name::msg_file_name variable_instance_name; 	
			std::string msg_one = getMessage(&connection);
			std::string msg_two = getMessage(&connection);
			ROS_INFO("msg_one: %s", msg_one.c_str());
			ROS_INFO("msg_two: %s", msg_two.c_str());
			if (msg_one.empty() && msg_two.empty())
			{
				ROS_INFO("In the both msgs empty condition");
				ros::spinOnce();
				loop_rate.sleep();
				radar_pub.publish(info_out);
				++count;
				continue;
			}
			else if (msg_one.empty() && !(msg_two.empty()) || !(msg_one.empty()) && msg_two.empty())
			{
				ROS_INFO("In the one msg is empty condition");
				if (msg_one.empty())
				{
			      process_json(&info, msg_two, serialPort);
			      radar_pub.publish(info);
						//becomes neccessary for subscriber callback functions
						ros::spinOnce();  
						// forces loop to wait for the remaining loop time to finish before starting over
						loop_rate.sleep();
				}
				else
				{
			      process_json(&info, msg_one, serialPort);
			      radar_pub.publish(info);
						//becomes neccessary for subscriber callback functions
						ros::spinOnce();  
						// forces loop to wait for the remaining loop time to finish before starting over
						loop_rate.sleep();
				}
			}
			else //(!(msg_one.empty()) && !(msg_two.empty()))
			{
				process_json(&info, msg_one, serialPort);
				ROS_INFO("info.tick after msg_one is added: %d",info.tick);
				ROS_INFO("info.speed after msg_one is added: %f",info.speed);
				process_json(&info, msg_two, serialPort);
				ROS_INFO("info.tick after msg_two is added: %d",info.tick);
				ROS_INFO("info.speed after msg_two is added: %f",info.speed);
				radar_pub.publish(info);
				//becomes neccessary for subscriber callback functions
				ros::spinOnce();  
				// forces loop to wait for the remaining loop time to finish before starting over
				loop_rate.sleep();
			}
		}
		else if (connected == 0)
		{
			ROS_INFO("Not Connected");
			connection.begin();
			count = 0;
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		++count;
	}
  ros::shutdown();
  return 0;
}
