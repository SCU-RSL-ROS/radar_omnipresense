^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package radar_omnipresense
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added the lib file so that linuxcommconnection is no longer a depedency issue
* address RapidJSON dependency
* added raw msgs
* Contributors: Jim Whitfield, Your Name

0.0.1 (2018-04-07)
------------------
* Merge branch 'master' of https://github.com/SCU-RSL-ROS/radar_omnipresense
* added doxygen detectable documentation for the function prototypes
* changed the topic name from 'radar' to 'radar_report'
* sensor id field shows the serial port that the radar is connected to
* added the rosluanch file for a single radar sensor and then have a rosluanch file that will launch multiple radar sensors, currently it launches 2
* Added unix time stamp to msg data
* Adding folder udev to contain device rules for usb radar device. also adding a bash script to move the rules to proper filesystem location.
* Create LICENSE
* added Findrapidjson.cmake
* Contributors: Jim Whitfield, Matthew Condino, Pillager225, RyanLoringCooper, Your Name, garrenhendricks, jimwhitfield
