/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define N_DRONES_MAX  20          /* maximum number of drones */

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_controller");

  ROS_INFO("Started global_controller.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  ros::NodeHandle pnh("~");
  int droneCount;
  if (pnh.getParam("droneCount", droneCount)){
      ROS_INFO("Got param 'droneCount': %d", droneCount);
  }
  else
     ROS_ERROR("Failed to get param 'droneCount'");

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  //waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));

  ros::NodeHandle nhq[N_DRONES_MAX] = { // NodeHandles for each drone (separate namespace)
    ros::NodeHandle("/crazyflie2_0"),
    ros::NodeHandle("/crazyflie2_1"),
    ros::NodeHandle("/crazyflie2_2"),
    ros::NodeHandle("/crazyflie2_3"),
    ros::NodeHandle("/crazyflie2_4"),
    ros::NodeHandle("/crazyflie2_5"),
    ros::NodeHandle("/crazyflie2_6"),
    ros::NodeHandle("/crazyflie2_7"),
    ros::NodeHandle("/crazyflie2_8"),
    ros::NodeHandle("/crazyflie2_9"),
    ros::NodeHandle("/crazyflie2_10"),
    ros::NodeHandle("/crazyflie2_11"),
    ros::NodeHandle("/crazyflie2_12"),
    ros::NodeHandle("/crazyflie2_13"),
    ros::NodeHandle("/crazyflie2_14"),
    ros::NodeHandle("/crazyflie2_15"),
    ros::NodeHandle("/crazyflie2_16"),
    ros::NodeHandle("/crazyflie2_17"),
    ros::NodeHandle("/crazyflie2_18"),
    ros::NodeHandle("/crazyflie2_19")
  };
  ros::Publisher trajectory_pub[N_DRONES_MAX];
  ros::Publisher enable_pub[N_DRONES_MAX];
  for (size_t i = 0; i < droneCount; i++)
  {
    ROS_INFO("Setup publisher %s.", nhq[i].getNamespace().c_str());
    trajectory_pub[i] = nhq[i].advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    enable_pub[i] = nhq[i].advertise<std_msgs::Bool>("enable", 10);
  }

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  ros::Time::sleepUntil(ros::Time(5.0));
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  std_msgs::Bool enable_msg;

  ROS_INFO("Start publishing waypoints (global).");

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.3, 0.5, 1.0);
  double desired_yaw = 0.0;

  for (size_t i = 0; i < droneCount; i++)
  {
    trajectory_msg.header.stamp = ros::Time::now();

    desired_position(0) = ((float)(i%2))/2;
    desired_position(1) = floor((float)(i/2))/2;
    desired_position(2) = 1.5 + ((float)(i%2))/5;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f] waiting %f seconds.",
    nhq[i].getNamespace().c_str(), desired_position.x(),
    desired_position.y(), desired_position.z(), 0);

    trajectory_pub[i].publish(trajectory_msg);

  }

  ros::Duration(3.0).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < droneCount; i++)
  {
    enable_msg.data = true;
    ROS_INFO("Publishing enable on namespace %s: %d.", nhq[i].getNamespace().c_str(), enable_msg.data);
    enable_pub[i].publish(enable_msg);
  }

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position(0) = 0;
  desired_position(1) = 0;
  desired_position(2) = 1;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
  for (size_t i = 0; i < droneCount; i++)
  {
    ROS_INFO("Publishing swarm waypoint on namespace %s: [%f, %f, %f] waiting %f seconds.",
    nhq[i].getNamespace().c_str(), desired_position.x(),
    desired_position.y(), desired_position.z(), 0);
    trajectory_pub[i].publish(trajectory_msg);
  }

  ros::spin();

  ros::shutdown();

  return 0;
}
