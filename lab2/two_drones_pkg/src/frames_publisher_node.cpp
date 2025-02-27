#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>

class FramesPublisherNode {
 private:
  ros::NodeHandle nh;
  ros::Time startup_time;
  ros::Timer heartbeat;
  tf2_ros::TransformBroadcaster tf_broadcaster;  // Instantiate the transform broadcaster

 public:
  FramesPublisherNode() {
    // Initialize the startup time and set up the timer
    startup_time = ros::Time::now();
    heartbeat =
        nh.createTimer(ros::Duration(0.02), &FramesPublisherNode::onPublish, this);
    heartbeat.start();
  }

  void onPublish(const ros::TimerEvent&) {
    // Compute the elapsed time in seconds since the node started
    double time = (ros::Time::now() - startup_time).toSec();

    // Declare and initialize the TransformStamped objects
    geometry_msgs::TransformStamped AV1World;
    geometry_msgs::TransformStamped AV2World;

    // Set identity rotation for both transforms initially
    AV1World.transform.rotation.w = 1.0;
    AV2World.transform.rotation.w = 1.0;

    // Populate AV1World transform
    AV1World.header.stamp = ros::Time::now();
    AV1World.header.frame_id = "world";
    AV1World.child_frame_id = "av1";

    // Set origin based on time
    AV1World.transform.translation.x = cos(time);
    AV1World.transform.translation.y = sin(time);
    AV1World.transform.translation.z = 0.0;

    // Set orientation so the y-axis is tangent to the trajectory
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, time);  // Roll and pitch are 0, yaw is time
    AV1World.transform.rotation.x = quat.x();
    AV1World.transform.rotation.y = quat.y();
    AV1World.transform.rotation.z = quat.z();
    AV1World.transform.rotation.w = quat.w();

    // Populate AV2World transform
    AV2World.header.stamp = ros::Time::now();
    AV2World.header.frame_id = "world";
    AV2World.child_frame_id = "av2";

    // Set origin based on time
    AV2World.transform.translation.x = sin(time);
    AV2World.transform.translation.y = 0.0;
    AV2World.transform.translation.z = cos(2 * time);

    // Publish both transforms
    tf_broadcaster.sendTransform(AV1World);
    tf_broadcaster.sendTransform(AV2World);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "frames_publisher_node");
  FramesPublisherNode node;
  ros::spin();
  return 0;
}
