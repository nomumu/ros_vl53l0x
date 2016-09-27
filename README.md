# ros_vl53l0x

##Overview
こそこそ団が発行する同人誌「ROSではじめるホビーロボット」のサンプルコードです．  
This is a sample code of "ROS de hajimeru hobby robot".  
VL53L0X(pololu)を使用して距離を配信するためのROSパッケージです．  
It is a ros package for delivering distance by the VL53L0X(pololu).  
このパッケージはIntel Edison上で動作するI2CとROSのために作成されています．  
This package has been created for the I2C and ROS that runs on Intel Edison.  

##Setup
このパッケージは他のブランチからソースコードを取得する必要があります．  
This package need to obtain the source code from the other branch.  
次のようにセットアップを実行します。  
Run the setup as follows.  

    $ cd catkin_ws/src  
    catkin_ws/src$ git clone https://github.com/nomumu/ros_vl53l0x.git  
    catkin_ws/src$ cd ros_vl53l0x  
    catkin_ws/src/ros_vl53l0x$ sh setup.sh
    catkin_ws/src/ros_vl53l0x$ cd ../../  
    catkin_ws$ catkin_make  

##Usage
I2Cを使用するためにはrootで実行する必要があります．  
You need to run as root in order to use the I2C.  
これは実行コマンドの例です．  
It is execution command example.  

    catkin_ws$ sudo su  
    catkin_ws# LD_LIBRARY_PATH=/usr/local/lib/i386-linux-gnu  
    catkin_ws# export LD_LIBRARY_PATH  
    catkin_ws# source devel/setup.sh
    catkin_ws# rosrun ros_vl53l0x ros_vl53l0x  

以下のパラメータ設定が利用可能です。  
The following parameter settings are available.  

    rosparam set /ros_vl53l0x/frame_id  (string)  
    rosparam set /ros_vl53l0x/period_ms (integer)  

このパッケージは次のトピックを配信します．  
This package publish the following topics.  

    /ros_vl53l0x/Range  

##Links
Pololu VL53L0X <https://www.pololu.com/product/2490/>  
VL53L0x-arduino <https://github.com/pololu/vl53l0x-arduino>  
