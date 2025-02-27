#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <iostream>
#include <sstream>

#include <math.h>

ros::Subscriber desire_traj_vertices_sub;
ros::Publisher desired_state_pub;
tf::TransformBroadcaster* br;

/**
 * Callback function for listening to the desired vertices msgs
 */
void trajCB(const geometry_msgs::PoseArray& traj_msg) {
  // sanity check for traj_msg size
  if (traj_msg.poses.size() == 0) {
    ROS_ERROR_THROTTLE(1, "Empty trajectory vertices msg.");
    return;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 0 |  16.485 - Fall 2020  - Lab 4 coding assignment  (10 pts)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  //  As a simple warm up exercise before we get to the actual 'real deal',
  //  let's just make our quadcopter fly to the first gate in the course.
  //  In this section:
  //   1. Extract the first vertex of the trajectory
  //   2. Set the acceleration and velocities to zero
  //   3. Publish the desired MultiDOFJointTrajectoryPoint
  //   4. Create and publish TF transform of the desired pose
  // ~~~~ begin solution
  // Extract the first vertex of the trajectory
    const geometry_msgs::Pose& first_vertex = traj_msg.poses[0];

    // Set the acceleration and velocities to zero
    trajectory_msgs::MultiDOFJointTrajectoryPoint desired_point;
    desired_point.transforms.resize(1);
    desired_point.velocities.resize(1);
    desired_point.accelerations.resize(1);

    // Set position from the first vertex
    desired_point.transforms[0].translation.x = first_vertex.position.x;
    desired_point.transforms[0].translation.y = first_vertex.position.y;
    desired_point.transforms[0].translation.z = first_vertex.position.z;

    // Set orientation from the first vertex
    desired_point.transforms[0].rotation = first_vertex.orientation;

    // Set velocity and acceleration to zero
    desired_point.velocities[0].linear.x = 0.0;
    desired_point.velocities[0].linear.y = 0.0;
    desired_point.velocities[0].linear.z = 0.0;
    desired_point.velocities[0].angular.x = 0.0;
    desired_point.velocities[0].angular.y = 0.0;
    desired_point.velocities[0].angular.z = 0.0;

    desired_point.accelerations[0].linear.x = 0.0;
    desired_point.accelerations[0].linear.y = 0.0;
    desired_point.accelerations[0].linear.z = 0.0;
    desired_point.accelerations[0].angular.x = 0.0;
    desired_point.accelerations[0].angular.y = 0.0;
    desired_point.accelerations[0].angular.z = 0.0;

    // Publish the desired MultiDOFJointTrajectoryPoint
    desired_state_pub.publish(desired_point);

    // Create and publish TF transform of the desired pose
    br->sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(first_vertex.orientation.x, 
                                      first_vertex.orientation.y, 
                                      first_vertex.orientation.z, 
                                      first_vertex.orientation.w),
                      tf::Vector3(first_vertex.position.x, 
                                   first_vertex.position.y, 
                                   first_vertex.position.z)),
        ros::Time::now(), 
        "world",  // Parent frame
        "quadcopter"  // Child frame
    ));

    ROS_INFO("Published desired state for first vertex.");
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 0
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_planner");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);
  ros::Time start(ros::Time::now());

  // deired traj vertices subscriber
  desire_traj_vertices_sub = n.subscribe("desired_traj_vertices", 10, trajCB);

  // publisher for desired states for controller
  desired_state_pub =
      n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "desired_state", 1);

  br = new tf::TransformBroadcaster();

  ros::spin();
}
