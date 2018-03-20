#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OPS241A_radar/radar_data.h"
#include "OPS241A_radar/SendAPICommand.h"
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
#include "LinuxCommConnection/src/CommConnection.h"
#include "LinuxCommConnection/src/SerialConnection.h"
#include "LinuxCommConnection/src/NetworkConnection.h"
#include "rapidjson/document.h"

using namespace rapidjson;

SerialConnection * con;
bool fft_on;

//#########################################################################################################################################################//
//#########################################################################################################################################################//

//service function to send api commands to radar device
bool api(OPS241A_radar::SendAPICommand::Request &req, OPS241A_radar::SendAPICommand::Response &res) 
{
	res.response = "false";
  if (req.command == "Oj")
  {
     ROS_INFO("Cannot turn JSON mode off");
     return true;
  }
  else if (req.command == "OF")
  	{ 
  		fft_on = 1;
  	}
  else if (req.command == "Of")
  	{
  		fft_on = 0;
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

void process_json(OPS241A_radar::radar_data *data, std::string single_msg)
{
	//ROS_INFO("Msg: %s", single_msg.c_str());
	
	//default template parameter uses UTF8 and MemoryPoolAllocator. //creates a document DOM instant called document
	Document document; 
 	//document.Parse(msg.data->c_str()); //parsing the json string msg.data with format{"speed":#.##,"direction":"inbound (or outbound)","time":###,"tick":###}
 	document.Parse(single_msg.c_str());
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
		data->sensorid = "to be determined"; 
		//place holder for field range for field that will be added soon
		data->range = 0;
		//place holder for field that will be added soon
		//info.angle = 0;
		//place holder for field
		data->objnum = 1;
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

std::string getMessage(CommConnection *connection) {
	std::string msg;
	bool startFilling = false; 
	while(true) {
		if(connection->available()) {
			char c = connection->read();
			if(c == '{') {
				startFilling = true;
			} else if(c == '}' && startFilling) {
				msg += c;
			//	ROS_INFO("msg = %s", msg.c_str());
				return msg;
			}
			if(startFilling) {
				msg += c;
			}
		}
	}
/*	int needToRead = connection->waitForDelimitor('{');    // Waits for data to be sent to the serial port.
	//stores string retrieved from serial port into msg.
	if(needToRead > 1) 
	{
		//ROS_INFO("need to read: %d",needToRead);
		char * dirt;
		connection->readUntil(&dirt, '{', needToRead);
		//ROS_INFO("getting junk %s", dirt);
		delete[] dirt;  
	}
	needToRead = connection->waitForDelimitor('}');
	msg = connection->readString(needToRead);
	//ROS_INFO("DERP %d strlen=%d", needToRead, msg.size());  
	*/
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
  //Open USB port serial connection for two way communication
  SerialConnection connection = SerialConnection(serialPort.c_str(), B19200, 0);  
  con = &connection;
  //the node is created. node publishes to topic "radar" using OPS241A::radar_data messages and will buffer up to 1000 messages before beginning to throw 		away old ones. 																																								     		
  ros::Publisher radar_pub = nh.advertise<OPS241A_radar::radar_data>("radar",1000); 
  //the service "send_api_commands" is created and advertised over ROS
  ros::ServiceServer radar_srv = nh.advertiseService("send_api_command", api);  
  //ROS loop rate, currently sent to 60Hz.
  ros::Rate loop_rate(1000); 
  //assuming radar is being started with no fft output.
  fft_on = 0;  
  connection.clearBuffer();
  //Forces KeepReading to continously read data from the USB serial port "connection".Then begin reading. Then set radar device to output Json format  data
  //string of format {"speed":#.##,"direction":"inbound(or outbound)","time":###,"tick":###}. Pause 100ms. Sets radar device to output FFT msg 
  //{"FFT":[ [#.###, #.###],[#.###, #.###],............[#.###, #.###],[#.###, #.###] ]} pause 100ms
  bool KeepReading = true; 
  connection.begin();
  connection.write("OJ"); 
  //usleep(100000); 
  connection.write("Of"); 
  //usleep(100000);
  connection.clearBuffer();
  
  //build_data array is used when an incomplete msg is recieved and stores that incomplete msg, builds it, and once complete
  std::string build_data; 
  bool out_ready = 0;
  int num_processed = 0;
  int expected_msgs;
  //creats an instant of the radar_data structure named info. format: package_name::msg_file_name variable_instance_name;
  
  // contines the while loop as long as ros::ok continues to continue true
  while (ros::ok()) 
  {
  	OPS241A_radar::radar_data info; 
	  OPS241A_radar::radar_data info_out;
  	expected_msgs = (int)fft_on+1;
  	//ROS_INFO("Expected Msgs: %i",expected_msgs);
  	 	
  	std::string msg = getMessage(&connection);
  	std::string msg_fft = getMessage(&connection);

 		if (msg.empty())
 		{
 			//ROS_INFO("Entered empty msg");
 			ros::spinOnce();
 			loop_rate.sleep();
 			if (out_ready)
			{ 
				//broadcast the message to any subscriber
				//radar_pub.publish(info_out); 
			}
			radar_pub.publish(info_out);
 			continue;
 		}
 		process_json(&info, msg);
 		//ROS_INFO("info returned");
 		process_json(&info, msg_fft);
 		//ROS_INFO("info returned again");
 		//ROS_INFO("fft: %s",msg_fft);
 		/*
 		else
 		{  		
 			std::size_t found_open_brace = msg.find("{");
 			int found_close_brace = needToRead-1;
 			//std::size_t found_close_brace = msg.find("}");
 		  bool open = (found_open_brace != std::string::npos);
 		  bool close = (found_close_brace != std::string::npos);
 		  bool greater = (found_open_brace < found_close_brace);

 			if ((open) && (close) && (greater))
 				{
 				  ROS_INFO("Case 1");
 				 //Process message and populate.
					std::string whole_msg = msg.substr(found_open_brace,found_close_brace+1);
					info = process_json(info, whole_msg);
					num_processed = num_processed+1;
					
					//Catch the start to the second message if it exists.
					std::string start_second_msg = msg.substr(found_close_brace+2, msg.size());
					std::size_t second_open_brace = start_second_msg.find("{");
					if ( second_open_brace != std::string::npos)
					{
						build_data.clear();
						build_data = start_second_msg.substr(second_open_brace, start_second_msg.size());
					}
 				}
 			else if ((open) && !(close))
 				{
 				  //Make a substring from the open brace position to the end of msg and set the build data to the substring. not processing
 					ROS_INFO("Case 2");
 					build_data.clear();
 					build_data = build_data + msg.substr(found_open_brace, msg.size());
 					
 				}
 			else if (!(open) && !(close))
 				{
 					ROS_INFO("Case 3");
 					//append new information/data. Not processing
 					build_data = build_data + msg;
 				}
 			else if (!(open) && (close))
 				{
 					ROS_INFO("Case 4");
 					//appending the last of the message 
 					build_data = build_data + msg.substr(0,found_close_brace+1);
 					info = process_json(info, build_data);
 					num_processed = num_processed+1;
 					//for data after the close brace
 					build_data.clear();
 				}
 			//case where the end of a current message is found and the start of the next one - need to check
 			//within the 2nd message to determine whether or not a full message has been found.
 			else if ((open) && (close))
 			{
 					ROS_INFO("Case 5");
 					ROS_INFO("Location of open brace: %i", open);
 			 		build_data = build_data + msg.substr(0,found_close_brace+1);
 					info = process_json(info, build_data);
 					num_processed = num_processed+1;
 					//find the very last close brace of the msg. Could be part of 1st and only msg, maybe part of a second complete msg.
 					std::size_t last_close_brace = msg.find_last_of("}");
 					//check to see if the very last close brace is the same as the close brace of the first msg. It would this is the only msg.
 					bool same_brace = last_close_brace == found_close_brace;
 					
 					if (same_brace) //there is .......}{....... there MAY BE a second message that is not complete
 					{
 						ROS_INFO("Case 6");
 						//create a string for the second msg. If there is no second msg then this will be empty
	 					std::string second_msg = msg.substr(found_open_brace,msg.size());
	 					build_data.clear();
	 					build_data = second_msg;
					}
					//when last_close_brace != found_close_brace }{} there is another complete msg we need to process
					else 
					{
						ROS_INFO("Case 7");
						std::string second_msg = msg.substr(found_open_brace,last_close_brace);
 					  info = process_json(info, second_msg);
 					  num_processed = num_processed+1;
					}
 			}
 			else
 				{
 				 ROS_ERROR("Code should never reach this point");
 				}
 			*/	
 		if (num_processed >= expected_msgs)
		{
			num_processed = 0;
			out_ready = 1;
			info_out = info;
		}
		/*
		if (out_ready)
		{ 
			//broadcast the message to any subscriber
			radar_pub.publish(info); 
		}
		*/
		radar_pub.publish(info);
		//becomes neccessary for subscriber callback functions
		ros::spinOnce(); 
		// forces loop to wait for the remaining loop time to finish before starting over
		loop_rate.sleep(); 
		//}
  }
  
  return(0);
}
