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

class GridMapHandler
{
public:
    GridMapHandler(ros::NodeHandle nh);

protected:
    // Callbacks
    void AUVNavigationCb(const cola2_msgs::NavStsConstPtr& auv_navigation_msg);
    void ASVNavigationCb(const cola2_msgs::NavStsConstPtr& asv_navigation_msg);
    void GridMapPublisher();
    void getMapInfo(double& position_north, double& position_east, string& robot);
    void createGridMapLayer(string& new_layer_name);
    void addDataToLayer(double& position_north, double& position_east, double& value, string& layer_name );

    
private:
    
    double AUV_position_north;
    double AUV_position_east;
    double AUV_position_depth;
    double AUV_altitude;
    double ASV_position_north;
    double ASV_position_east;
    double length; 
    double size; 
    double resolution;
    string layer_name;
    string new_layer_name;
    string map_frame_id;
    GridMap * map;
    string node_name_;
    string ASV = std::string("ASV");
    string AUV = std::string("AUV");
    ros::Subscriber auv_navigation_sub_;
    ros::Subscriber asv_navigation_sub_;
    ros::Publisher grid_map_pub_;
    ros::Publisher info_map_ASV_pub_;
    ros::Publisher info_map_AUV_pub_;
    ros::NodeHandle nh_;
    double AUV_read_value;
    double ASV_read_value;

};

GridMapHandler::GridMapHandler(ros::NodeHandle nh):  nh_(nh)
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
    auv_navigation_sub_ = nhp.subscribe("/turbot/navigator/navigation", 1, &GridMapHandler::AUVNavigationCb, this);
    asv_navigation_sub_ = nhp.subscribe("/xiroi/navigator/navigation", 1, &GridMapHandler::ASVNavigationCb, this);

    // Setup publishers
    grid_map_pub_ = nhp.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    info_map_ASV_pub_ = nhp.advertise<sensor_msgs::Range>("grid_map_ASV_data", 1, true); 
    info_map_AUV_pub_ = nhp.advertise<sensor_msgs::Range>("grid_map_AUV_data", 1, true);

}

void GridMapHandler::AUVNavigationCb(const cola2_msgs::NavStsConstPtr& auv_navigation_msg)
{
    GridMapPublisher();
    AUV_position_north = auv_navigation_msg->position.north;
    AUV_position_east = auv_navigation_msg->position.east;
    AUV_position_depth = auv_navigation_msg->position.depth;
    AUV_altitude = auv_navigation_msg->altitude;
    getMapInfo(AUV_position_north, AUV_position_east, AUV );
    createGridMapLayer(AUV);
    addDataToLayer(AUV_position_north, AUV_position_east, AUV_read_value, AUV);
}


void GridMapHandler::ASVNavigationCb(const cola2_msgs::NavStsConstPtr& asv_navigation_msg)
{
    GridMapPublisher();
    ASV_position_north = asv_navigation_msg->position.north;
    ASV_position_east = asv_navigation_msg->position.east;
    getMapInfo(ASV_position_north, ASV_position_east, ASV);
    createGridMapLayer(ASV);
    addDataToLayer(ASV_position_north, ASV_position_east, ASV_read_value, ASV);
}


void GridMapHandler::addDataToLayer(double& position_north, double& position_east, double& value, string& layer_name)
{
    map->at(layer_name,{position_north,position_east}) = value;

}

void GridMapHandler::getMapInfo(double& position_north, double& position_east, string& robot ) 
{
    int north_position = trunc(position_north);
    int east_position = trunc(position_east);
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = "base_link";
    range_msg.range = map->atPosition(layer_name,{north_position,east_position});

        if (robot=="AUV"){
            info_map_AUV_pub_.publish(range_msg);
            AUV_read_value =range_msg.range;
        }
        else{
            info_map_ASV_pub_.publish(range_msg);
            ASV_read_value =range_msg.range;
        }
    
}
void GridMapHandler::createGridMapLayer(string& new_layer_name)
{
//Grid Map layer initialization  
  map->add(new_layer_name,0.0);
}

void GridMapHandler::GridMapPublisher() 
{
  map->setFrameId(map_frame_id);
  map->setGeometry(Length(size, length), resolution);

  // Add data to grid map
  ros::Time time = ros::Time::now();
  for (GridMapIterator it(*map); !it.isPastEnd(); ++it) {
      Position position;
      map->getPosition(*it, position);
      map->at("elevation", *it) =  10 + (5 * std::sin(position.x()/7));
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

    GridMapHandler node(nh);
    
    ros::spin();

    ros::shutdown();

    return 0;
}