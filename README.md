# Env Setup
* git clone -b v4.3.1  --recursive https://github.com/espressif/esp-idf.git esp-idf
* esp-idf$ ./install.sh
* esp-idf$ . ./export.sh 
* pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources

* git clone https://github.com/MS71/ESP32microROSImagePublisher.git
* cd ESP32microROSImagePublisher
* ESP32microROSImagePublisher$ idf.py menuconfig
* 1.) change “micro ROS middleware to micro-ROS over embeddedRTPS”
* 2.) enter WIFI name and password
* 3.) ESC; ESC; Save
* ESP32microROSImagePublisher$ idf.py build



