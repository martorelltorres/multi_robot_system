#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <iostream>
#include <math.h>
#include <string>
#include <cola2_msgs/NavSts.h>
#include <sensor_msgs/Range.h>
#include <limits>

using namespace grid_map;
using namespace std;

class GridMapInfo
{
public:
    GridMapInfo(ros::NodeHandle nh);

protected:
    // Callbacks
    void AUVNavigationCb(const cola2_msgs::NavStsConstPtr& auv_navigation_msg);
    void GridMapPublisher();
    void getMapInfo(double& AUV_position_north, double& AUV_position_east);

    
private:
    
    double AUV_position_north;
    double AUV_position_east;
    double AUV_position_depth;
    double AUV_altitude;
    double length; 
    double size; 
    double resolution;
    string layer_name;
    string map_frame_id;
    GridMap * map;
    string node_name_;
    ros::Subscriber auv_navigation_sub_;
    ros::Publisher grid_map_pub_;
    ros::Publisher info_map_pub_;
    ros::NodeHandle nh_;

};

GridMapInfo::GridMapInfo(ros::NodeHandle nh):  nh_(nh)
{
    ros::NodeHandle nhp("~");
    node_name_ = ros::this_node::getName();

    nhp.param("length", length, 40.0);
    nhp.param("size", size, 40.0);
    nhp.param("resolution", resolution, 1.0);
    nhp.param("layer_name", layer_name, std::string("elevation"));
    nhp.param("map_frame_id", map_frame_id,  std::string("world_ned"));

    // Create GridMap object
    map = new GridMap({layer_name});

    ROS_INFO("MRSNavigation parameters");
	  ROS_INFO_STREAM("Grid map length: " << length);	
	  ROS_INFO_STREAM("Grid map size: " << size);	
	  ROS_INFO_STREAM("Grid map resolution: " << resolution);	
	  ROS_INFO_STREAM("Grid map layer name: " << layer_name);
    ROS_INFO_STREAM("Grid map frame_id: " << map_frame_id);	
    ROS_INFO("-----------------------------------");

    // Setup subscribers
    auv_navigation_sub_ = nhp.subscribe("/turbot/navigator/navigation", 1, &GridMapInfo::AUVNavigationCb, this);

    // Setup publishers
    grid_map_pub_ = nhp.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    info_map_pub_ = nhp.advertise<sensor_msgs::Range>("grid_map_data", 1, true); 

}

void GridMapInfo::AUVNavigationCb(const cola2_msgs::NavStsConstPtr& auv_navigation_msg)
{
    GridMapPublisher();
    AUV_position_north = auv_navigation_msg->position.north;
    AUV_position_east = auv_navigation_msg->position.east;
    AUV_position_depth = auv_navigation_msg->position.depth;
    AUV_altitude = auv_navigation_msg->altitude;
    getMapInfo(AUV_position_north, AUV_position_east);
}

void GridMapInfo::getMapInfo(double& AUV_position_north, double& AUV_position_east) 
{
    int north_position = trunc(AUV_position_north);
    int east_position = trunc(AUV_position_east);
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = "base_link";
    range_msg.range = map->atPosition(layer_name,{north_position,east_position});
    info_map_pub_.publish(range_msg);
}

void GridMapInfo::GridMapPublisher() 
{
  map->setFrameId(map_frame_id);
  map->setGeometry(Length(size, length), resolution);

  // Add data to grid map
  ros::Time time = ros::Time::now();
  for (GridMapIterator it(*map); !it.isPastEnd(); ++it) {
      Position position;
      map->getPosition(*it, position);
      map->at("elevation", *it) =   2 + std::sin(position.x()/3);
  }
  // Publish grid map
  map->setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(*map, message);
  grid_map_pub_.publish(message);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_map_info");

    ros::NodeHandle nh;

    GridMapInfo node(nh);
    
    ros::spin();

    ros::shutdown();

    return 0;
}