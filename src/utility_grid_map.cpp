#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <map>
#include <sstream>
#include <iostream>
#include "multi_robot_system/BufferedData.h"
#include <ros/ros.h>
// #include <grid_map/grid_map.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>


class UtilityGridMap {
public:
    UtilityGridMap(double resolution = 1.0, int width = 200, int height = 200)
        : resolution_(resolution), width_(width), height_(height), nh_("~") {
        
        ros::NodeHandle private_nh("~");
        private_nh.getParam("number_of_robots", number_of_robots_);
        private_nh.getParam("number_of_auvs", number_of_auvs_);
        private_nh.getParam("number_of_asvs", number_of_asvs_);
        private_nh.getParam("pickle_path", pickle_path_);

        tf_broadcaster_ = new tf::TransformBroadcaster();
        listener_ = new tf::TransformListener();

        map_.resize(width_ * height_, 0);
        robots_information_.resize(number_of_auvs_, {0, 0, 0, 0, 0});
        acquired_data_.resize(number_of_auvs_, 0);
        comm_signal_.resize(number_of_auvs_, 0);
        
        readAreaInfo();
        
        grid_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);        
        
        ros::Subscriber data_sub = nh_.subscribe("/mrs/asv0_data_buffered", 1, &UtilityGridMap::updateAcquiredData, this);
        for (int i = 0; i < number_of_auvs_; i++) {
            std::string topic = "/robot" + std::to_string(i) + "/acoustic_communication";
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
                topic, 1, boost::bind(&UtilityGridMap::updateAcousticInfo, this, _1, i));
        }

        // Create grid map.
        GridMap map({"utility"});
        map.setFrameId("map");
        map.setGeometry(Length(400, 400), 1);
        ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));

// Work with grid map in a loop.
ros::Rate rate(30.0);
        
        ros::Rate rate(1);
        while (ros::ok()) {
            publishGridMap();
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    void readAreaInfo() {
        // Aquí se leería la información del archivo pickle (requiere una solución alternativa en C++)
    }
    
    void updateAcousticInfo(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, int robot_agent) {
        double roll, pitch, yaw;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        double normalized_covariance = normalize(msg->pose.covariance[0], 0, 4.5, 1, 0);
        robots_information_[robot_agent] = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw, normalized_covariance};
        
        double rssi = getCommunicationSignal(0, robot_agent);
        double utility = 1 + acquired_data_[robot_agent] + rssi;
        
        updateGrid(robot_agent, utility);
    }
    
    void updateAcquiredData(const multi_robot_system::BufferedData::ConstPtr& msg) {
        for (int i = 0; i < number_of_auvs_; i++) {
            double rssi = getCommunicationSignal(0, i);
            acquired_data_[i] = msg->storage[i];
            double utility = 1 + acquired_data_[i] + rssi;
            updateGrid(i, utility);
        }
    }
    
    double getCommunicationSignal(int asv_id, int auv_id) {
        double distance = getDistance(asv_id, auv_id);
        double rssi = -47.537 - (0.368 * distance) + (0.00132 * distance * distance) - (0.0000016 * distance * distance * distance);
        double normalized_value = (rssi + 45) / (-85 + 45);
        comm_signal_[auv_id] = normalized_value;
        return normalized_value;
    }
    
    double getDistance(int asv_id, int auv_id) {
        double x_diff = asvs_positions_[asv_id][0] - robots_information_[auv_id][0];
        double y_diff = asvs_positions_[asv_id][1] - robots_information_[auv_id][1];
        return sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    
    void updateGrid(int auv_id, double utility) {
        // Add data to grid map.
        for (PolygonIterator it(map); !it.isPastEnd(); ++it) {
            Position position;
            map.getPosition(*it, position);
            map.at("utility", *it) = utility;
            }
        }
    
    void publishGridMap() {
        // Publish grid map.
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);
        grid_map_pub.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
        message.info.header.stamp.toSec());

        // Wait for next cycle.
        rate.sleep();
    }
    
private:
    ros::NodeHandle nh_;
    ros::Publisher grid_pub_;
    tf::TransformBroadcaster* tf_broadcaster_;
    tf::TransformListener* listener_;
    
    double resolution_;
    int width_, height_;
    int number_of_robots_, number_of_auvs_, number_of_asvs_;
    std::string pickle_path_;
    std::vector<double> map_;
    std::vector<std::vector<double>> robots_information_;
    std::vector<double> acquired_data_;
    std::vector<double> comm_signal_;
    std::vector<std::vector<double>> asvs_positions_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "utility_gridmap_node");
    UtilityGridMap gridmap;
    return 0;
}
