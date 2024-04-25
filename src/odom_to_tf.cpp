#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Quaternion.h>

using namespace std;

class odom_to_tf {
	private:
		ros::Subscriber sub;
		tf::TransformBroadcaster tf_broadcaster;
		string node_name;
		string root_frame;
		string child_frame;

	public:
		odom_to_tf() {
			ros::NodeHandle nh;
			node_name = ros::this_node::getName();

			// Retrieve parameters
			nh.getParam(node_name + "/root_frame", root_frame);
            nh.getParam(node_name + "/child_frame", child_frame);

			// Subscribe to odometry topic
			sub = nh.subscribe("input_odom", 10, &odom_to_tf::odomCallback, this);
		}

		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
			// Create transform
			tf::Transform transform;
            transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

            tf::Quaternion rotation;
            tf::quaternionMsgToTF(msg->pose.pose.orientation, rotation);
            transform.setRotation(rotation);

			// Publish transform
			tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
		}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf");
    odom_to_tf odom_to_tf;
    ros::spin();
    return 0;
}