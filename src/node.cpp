#include <cstdlib>
#include "ros/ros.h"
#include "ros_vl53l0x/VL53L0X.h"
#include <ros_vl53l0x/Range.h>

static VL53L0X driver;

#define PERIOD_MS_DEF  (50)
#define PERIOD_MS_MIN  (40)

#define FRAME_ID_DEF   ("vl53l0x")

#define CONTINUOUS

int main( int argc, char **argv )
{
  ros::init(argc, argv, "ros_vl53l0x");
  ros::NodeHandle n;
  int period_ms = PERIOD_MS_DEF;
  std::string frame_id_str = FRAME_ID_DEF;
  uint16_t range;
  ros_vl53l0x::Range range_msg;

  if( n.hasParam("/ros_vl53l0x/period_ms") )
  {
    n.getParam("/ros_vl53l0x/period_ms", period_ms );
    if( period_ms < PERIOD_MS_MIN )
    {
      period_ms = PERIOD_MS_MIN;
    }
  }

  if( n.hasParam("/ros_vl53l0x/frame_id") )
  {
    n.getParam("/ros_vl53l0x/frame_id", frame_id_str );
  }

  if( driver.init( true ) == false )
  {
    exit( 0 );
  }
  else
  {
#ifdef CONTINUOUS // Continuous mode
    driver.startContinuous( period_ms );
#else // Single mode
    driver.setTimeout(50);
    driver.setMeasurementTimingBudget(20000);
#endif
  }

  ros::Publisher range_pub = n.advertise<ros_vl53l0x::Range>("/ros_vl53l0x/Range", 100);

  // start message
#ifdef CONTINUOUS // Continuous mode
  ROS_INFO("ros_vl53l0x node [Continuous mode] period_ms=%d frame_id=%s",period_ms,frame_id_str.c_str());
#else
  ROS_INFO("ros_vl53l0x node [Single mode] period_ms=%d frame_id=%s",period_ms,frame_id_str.c_str());
#endif

  range_msg.header.frame_id = frame_id_str;

  // main loop
  while( ros::ok() )
  {
    ros::Rate r( 1000.0 / period_ms ); // period Hz
#ifdef CONTINUOUS // Continuous mode
    range_msg.range = driver.readRangeContinuousMillimeters();
#else // Single mode
    range_msg.range = driver.readRangeSingleMillimeters();
#endif
    range_msg.header.stamp = ros::Time::now();
    range_pub.publish( range_msg );

    r.sleep();
    ros::spinOnce();
  }

  return 0;
}


