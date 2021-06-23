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
#include <string>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define N_DRONES_MAX  20          /* maximum number of drones */

#define SWARM_DISABLED            0
#define SWARM_DECLARATIVE_SIMPLE  1
#define SWARM_REYNOLDS            2
#define SWARM_REYNOLDS_LIMITED    3
#define SWARM_REYNOLDS_VELOCITY   4


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

  ROS_INFO("global_controller: Started.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  ros::NodeHandle pnh("~");
  int droneCount;
  if (pnh.getParam("droneCount", droneCount))
    ROS_INFO("global_controller: Got param 'droneCount': %d", droneCount);
  else
    ROS_ERROR("global_controller: Failed to get param 'droneCount'");

  float spacingX;
  if (pnh.getParam("spacingX", spacingX))
    ROS_INFO("global_controller: Got param 'spacingX': %f", spacingX);
  else
    ROS_ERROR("global_controller: Failed to get param 'spacingX'");

  float spacingY;
  if (pnh.getParam("spacingY", spacingY))
    ROS_INFO("global_controller: Got param 'spacingY': %f", spacingY);
  else
    ROS_ERROR("global_controller: Failed to get param 'spacingY'");

  float spacingZ;
  if (pnh.getParam("spacingZ", spacingZ))
    ROS_INFO("global_controller: Got param 'spacingZ': %f", spacingZ);
  else
    ROS_ERROR("global_controller: Failed to get param 'spacingZ'");

  float offsetX = 0;
  if (pnh.getParam("offsetX", offsetX))
    ROS_INFO("global_controller: Got param 'offsetX': %f", offsetX);
  else
    ROS_INFO("global_controller: Failed to get param 'offsetX', using default: %f", offsetX);

  float offsetY = 0;
  if (pnh.getParam("offsetY", offsetY))
    ROS_INFO("global_controller: Got param 'offsetY': %f", offsetY);
  else
    ROS_INFO("global_controller: Failed to get param 'offsetY', using default: %f", offsetY);

  float offsetZ = 0;
  if (pnh.getParam("offsetZ", offsetZ))
    ROS_INFO("global_controller: Got param 'offsetZ': %f", offsetZ);
  else
    ROS_INFO("global_controller: Failed to get param 'offsetZ', using default: %f", offsetZ);

  int swarm_mode = SWARM_DISABLED;
  std::string swarmMode;
  if (pnh.getParam("swarmMode", swarmMode))
    ROS_INFO("global_controller: Got param 'swarmMode': %s", swarmMode.c_str());
  else
    ROS_ERROR("global_controller: Failed to get param 'swarmMode'");
  if(swarmMode == "mpc1")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_DECLARATIVE_SIMPLE");
    swarm_mode = SWARM_DECLARATIVE_SIMPLE;
  }
  if(swarmMode == "reynolds")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_REYNOLDS");
    swarm_mode = SWARM_REYNOLDS;
  }
  if(swarmMode == "reylimited")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_REYNOLDS_LIMITED");
    swarm_mode = SWARM_REYNOLDS_LIMITED;
  }
  if(swarmMode == "reyvelocity")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_REYNOLDS_VELOCITY");
    swarm_mode = SWARM_REYNOLDS_VELOCITY;
  }

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
    ROS_INFO("global_controller: Setup publisher %s.", nhq[i].getNamespace().c_str());
    trajectory_pub[i] = nhq[i].advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    enable_pub[i] = nhq[i].advertise<std_msgs::Int8>("enable", 10);
  }

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("global_controller: Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("global_controller: Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("global_controller: Unpaused the Gazebo simulation.");
  }

  ros::Time::sleepUntil(ros::Time(5.0));
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  std_msgs::Int8 enable_msg;

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.3, 0.5, 1.0);
  double desired_yaw = 0.0;

  int modulus = 5;
  if(droneCount < 10)
    modulus = 4;
  if(droneCount == 9)
    modulus = 3;
  if(droneCount == 4)
    modulus = 2;

  ROS_INFO("global_controller: Go to starting position (hovering).");
  for (size_t i = 0; i < droneCount; i++) // go to starting position (hovering)
  {
    trajectory_msg.header.stamp = ros::Time::now();

    desired_position(0) = ((float)(i%modulus)) * spacingX + offsetX; // * 0.5;
    desired_position(1) = floor((float)(i/modulus)) * spacingY + offsetY; // * 0.5;
    desired_position(2) = 1.4 + ((float)(i%2)) * spacingZ + offsetZ; //* 0.2;
    desired_yaw = 0; // not rotated
//    desired_yaw = ((float)(i%2)) * (3.141592 / 4); // 45 degrees rotated
//    desired_yaw = ((float)(i%2)) * (3.141592 / 2); // 90 degrees rotated
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

    ROS_INFO("global_controller: Publishing waypoint on namespace %s: [x=%f, y=%f, z=%f, yaw=%f].",
    nhq[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z(), desired_yaw);

    trajectory_pub[i].publish(trajectory_msg);

  }

  ros::Duration(3.0).sleep();
  ros::spinOnce();

  ROS_INFO("global_controller: Enable swarm mode.");
  for (size_t i = 0; i < droneCount; i++) // enable swarm mode
  {
    enable_msg.data = swarm_mode;
//    enable_msg.data = SWARM_REYNOLDS;
    ROS_INFO("global_controller: Publishing enable on namespace %s: %d.", nhq[i].getNamespace().c_str(), enable_msg.data);
    enable_pub[i].publish(enable_msg);
  }

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position(0) = 0;
  desired_position(1) = 0;
  desired_position(2) = 1.5;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0, &trajectory_msg);
  for (size_t i = 0; i < droneCount; i++) // send target point to swarm
  {
    ROS_INFO("global_controller: Publishing swarm target on namespace %s: [%f, %f, %f].",
    nhq[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z());
    trajectory_pub[i].publish(trajectory_msg);
  }

  ros::Duration(20.0).sleep();
  ros::spinOnce();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position(0) = 0;
  desired_position(1) = 5.0;
  desired_position(2) = 1.5;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0, &trajectory_msg);
  for (size_t i = 0; i < droneCount; i++) // send target point to swarm
  {
    ROS_INFO("global_controller: Publishing swarm target on namespace %s: [%f, %f, %f].",
    nhq[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z());
    trajectory_pub[i].publish(trajectory_msg);
  }

  ros::Duration(30.0).sleep();
  ros::spinOnce();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position(0) = 5.0;
  desired_position(1) = 0;
  desired_position(2) = 1.5;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0, &trajectory_msg);
  for (size_t i = 0; i < droneCount; i++) // send target point to swarm
  {
    ROS_INFO("global_controller: Publishing swarm target on namespace %s: [%f, %f, %f].",
    nhq[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z());
    trajectory_pub[i].publish(trajectory_msg);
  }

  ros::Duration(30.0).sleep();
  ros::spinOnce();
  //  ros::spin();
  ROS_INFO("global_controller: End simulation.");
  ros::shutdown(); // end simulation

  return 0;
}
