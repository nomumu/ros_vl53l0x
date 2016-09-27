# ros_vl53l0x

���������c�����s���铯�l���uROS�ł͂��߂�z�r�[���{�b�g�v�̃T���v���R�[�h�ł��D
This is a sample code of "ROS de hajimeru hobby robot".
VL53L0X(pololu)���g�p���ċ�����z�M���邽�߂�ROS�p�b�P�[�W�ł��D
It is a ros package for delivering distance by the VL53L0X(pololu).
���̃p�b�P�[�W��Intel Edison��œ��삷��I2C��ROS�̂��߂ɍ쐬����Ă��܂��B
This package has been created for the I2C and ROS that runs on Intel Edison.

##Setup
���̃p�b�P�[�W�͑��̃u�����`����\�[�X�R�[�h���擾����K�v������܂��D
This package need to obtain the source code from the other branch.
���̂悤�ɃZ�b�g�A�b�v�����s���܂��B
Run the setup as follows.
    $ cd catkin_ws/src
    catkin_ws/src$ git clone https://github.com/nomumu/ros_vl53l0x.git
    catkin_ws/src$ cd ros_vl53l0x
    catkin_ws/src/ros_vl53l0x$ ./setup
    catkin_ws/src/ros_vl53l0x$ cd ../../
    catkin_ws$ catkin_make

##Usage
I2C���g�p���邽�߂ɂ�root�Ŏ��s����K�v������܂��B
You need to run as root in order to use the I2C.
����͎��s�R�}���h�̗�ł��B
It is execution command example.
    catkin_ws$ sudo su
    catkin_ws# LD_LIBRARY_PATH=/usr/local/lib/i386-linux-gnu
    catkin_ws# export LD_LIBRARY_PATH
    catkin_ws# rosrun ros_vl53l0x ros_vl53l0x

�ȉ��̃p�����[�^�ݒ肪���p�\�ł��B
The following parameter settings are available.
    rosparam set /ros_vl53l0x/frame_id  (string)
    rosparam set /ros_vl53l0x/period_ms (integer)

���̃p�b�P�[�W�͎��̃g�s�b�N��z�M���܂��B
This package publish the following topics.
    /ros_vl53l0x/Range

##Links
[pololu]: https://www.pololu.com/product/2490/ "Pololu VL53L0X"
[arduino]: https://github.com/pololu/vl53l0x-arduino "VL53L0x-arduino"
