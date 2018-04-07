^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package radar_omnipresense
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2018-04-07)
------------------
* Merge branch 'master' of https://github.com/SCU-RSL-ROS/radar_omnipresense
* added info to package.xml
* added url to package.xml
* Update README.md
* added readme back in
* added doxygen detectable documentation for the function prototypes
* added doxygen detectable documentation for the function prototypes
* Merge pull request `#8 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/8>`_ from jimwhitfield/master
  Tweak to default to use of one sensor and to remove redundant fft part
* Tweak to default to use of one sensor and to remove redundant fft section
* changed the topic name from 'radar' to 'radar_report'
* sensor id feild shows the serial port that the radar is connected to
* took out the remapping as this is only applicable to subscriber nodes
* added the rosluanch file for a single radar sensor and then have a rosluanch file that will launch multiple radar sensors, currently it launches 2
* Merge pull request `#7 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/7>`_ from SCU-RSL-ROS/catching_msgs
  Catching msgs
* Added unix time stamp to msg data
* This code attempts to chatch all messages
* Adding folder udev to contain device rules for usb radar device. also adding a bash script to move the rules to proper filesystem location.
* Update radar_publisher.cpp
  deleted fft_on boolean variable because it was no longer being used
* Update radar_publisher.cpp
* Update fft.msg
* Update fft.msg
* Update radar_publisher.cpp
* Update radar_publisher.cpp
* Update radar_publisher.cpp
* Merge pull request `#6 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/6>`_ from SCU-RSL-ROS/add-license-1
  Create LICENSE
* Create LICENSE
* Update radar_publisher.cpp
  Removed second process_json() function call 'process_json(&info, msg_fft)
* Update radar.launch
  Added a missing " on line 9
* Update README.md
* Merge pull request `#5 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/5>`_ from SCU-RSL-ROS/ReadmeFix
  Readme fix
* made links unordered list
* made links unordered list
* made links actual links
* Update package.xml
* Update CMakeLists.txt
* Update radar_publisher.cpp
* Update radar.launch
* Update README.md
* Update radar_publisher.cpp
* Update README.md
* Merge pull request `#4 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/4>`_ from jimwhitfield/patch-1
  markdown typography changes
* markdown typography changes
* Update radar_publisher.cpp
* Update README.md
* Merge pull request `#3 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/3>`_ from SCU-RSL-ROS/garrenhendricks-1
  Update radar_publisher.cpp
* Update radar_publisher.cpp
* Merge pull request `#2 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/2>`_ from SCU-RSL-ROS/garrenhendricks
  Update radar_publisher.cpp
* Update radar_publisher.cpp
* Update README.md
* Merge pull request `#1 <https://github.com/SCU-RSL-ROS/radar_omnipresense/issues/1>`_ from SCU-RSL-ROS/noLCC
  No LinuxCommConnection src in the repo
* updated README.md
* Merge branch 'master' into noLCC
* LCC no longer has to have its src in this repo
* Update README.md
* Update README.md
* readded submodule with CMakeLists.txt
* removed bad submodule
* added Findrapidjson.cmake
* first commit
* first commit
* Initial commit
* Contributors: Jim Whitfield, Matthew Condino, Pillager225, RyanLoringCooper, Your Name, garrenhendricks, jimwhitfield
