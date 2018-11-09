# zed_opencv_ros
This is a ROS node to run the ZED stereo camera natively with OpenCV without their SDK. 
Note that you'll need the serial number of your zed camera. You can obtain that from the ZED explorer. First time you run the node, it'll connect to the internet and grab the calibration file from their database. Afterwards, it should work fine with the downloaded one.
