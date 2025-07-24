
This project is based on the head_control_pneumatic_endoscope_ros-master package by Yoeko Mak.  
Small modifications have been made to adapt it to my use case.

To use this package, first you need to make sure the ros-igtl-bridge is running (see the ros-igtl-bridge package on GitRAM). 
Secondly, you should connect two Arduino's, one connected to the pneumatic regulators and another one connected to the footswitch.
Make sure the regulator arduino is connected inside Ubuntu as dev/ttyACM0 and the footswitch arduino as dev/ttyACM1.
Now, you can open a terminal in ROS and launch endoscope_middleware using:

```roslaunch endoscope_middleware endoscope_middleware```

Here, the xsens_awinda_imu package is launched and tries to connect the xsens dongle (USB) to the IMU sensor. This package can also be found on GitRAM and should be installed in this src folder.


Next, open a second terminal and launch path_middleware:

```roslaunch endoscope_middleware path_middleware```


Then, open a third terminal and launch feedback_middleware:

```roslaunch endoscope_middleware feedback_middleware```

Make sure that the correct feedback method is selected in feedback_middleware.launch beforehand.


To record a run along the tract, simply launch record_rosbag.launch:

```roslaunch endoscope_middleware record_rosbag.launch```




