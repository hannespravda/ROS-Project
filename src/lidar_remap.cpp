#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "dynamic_reconfigure/server.h"
#include "first_project/LidarRemapConfig.h"

class LidarRemapNode {
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Publisher remapped_pub;
    std::string frame_id;

public:
    LidarRemapNode() : nh_("~") {
        // Subscribe to lidar data
        lidar_sub = nh.subscribe("/os_cloud_node/points", 1, &LidarRemapNode::lidarCallback, this);

        // Publish remapped point cloud
        remapped_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);

        // Initialize dynamic reconfigure server
        dynamic_reconfigure::Server<first_project::LidarRemapConfig>::CallbackType config_callback;
        config_callback = boost::bind(&LidarRemapNode::reconfigureCallback, this, _1, _2);
        dynamic_reconfigure::Server<first_project::LidarRemapConfig> server;
        server.setCallback(config_callback);
    }

    // Callback for lidar data
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Remap frame id if necessary
        sensor_msgs::PointCloud2 remapped_msg = *msg;
        remapped_msg.header.frame_id = frame_id;

        // Publish remapped point cloud
        remapped_pub.publish(remapped_msg);
    }

    // Dynamic reconfigure callback
    void reconfigureCallback(first_project::LidarRemapConfig &config, uint32_t level) {
        // Update frame id based on dynamic reconfigure parameters
        if (config.use_wheel_odom) {
            frame_id = "wheel_odom";
        } else {
            frame_id = "gps_odom";
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");
    LidarRemapNode node;
    ros::spin();
    return 0;
}
