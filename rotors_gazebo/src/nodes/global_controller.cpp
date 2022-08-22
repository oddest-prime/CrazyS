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
#include <cstdlib>
#include <ctime>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <gazebo_msgs/SetModelState.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define N_DRONES_MAX  30          /* maximum number of drones */

#define SWARM_DISABLED            0
#define SWARM_DECLARATIVE_SIMPLE  1
#define SWARM_REYNOLDS            2
#define SWARM_REYNOLDS_LIMITED    4
#define SWARM_REYNOLDS_VELOCITY   8
#define SWARM_GRADIENT            16
#define SWARM_GRADIENT_ENUM       32
#define SWARM_PHASE_ESTABLISHED   64
#define SWARM_LANDING             32768
#define SWARM_DECLARATIVE_DISTANCES           1
#define SWARM_DECLARATIVE_DISTANCES_GROUND    2


#define OPERATION_MODE_NONE         0
#define OPERATION_MODE_TIMED        1
#define OPERATION_MODE_INTERACTIVE  2

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

/*
void publish_command(float cmd_x, float cmd_y, float cmd_z, float cmd_enable) {
  // Default desired position and yaw.
  Eigen::Vector3d desired_position(cmd_x, cmd_y, cmd_z);
  double desired_yaw = 0.0;

  for (size_t i = 0; i < droneCount; i++) // go to starting position (hovering)
  {
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

    ROS_INFO("global_controller: Publishing waypoint on namespace %s: [x=%f, y=%f, z=%f, yaw=%f].",
    nhq[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z(), desired_yaw);

    trajectory_pub[i].publish(trajectory_msg);
  }
}
*/

ros::Publisher trajectory_pub[N_DRONES_MAX];
ros::Publisher enable_pub[N_DRONES_MAX];
ros::NodeHandle* nhp;

int droneCount;
float spacingX;
float spacingY;
float spacingZ;
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;
float randNoise = 0.3;
int swarm_mode = SWARM_DISABLED;
int operation_mode = OPERATION_MODE_TIMED;

void keyboard_callback(const std_msgs::Int32Ptr& msg) {
  ROS_INFO("global_controller: Got data in keyboard_callback: %d", msg->data);

  if(operation_mode == OPERATION_MODE_INTERACTIVE)
  {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    std_msgs::Int32 enable_msg;

    // Default desired position and yaw.
    Eigen::Vector3d desired_position(0.3, 0.5, 1.0);
    double desired_yaw = 0.0;

    if(msg->data == 'h') // hovering
    {
      int modulus = 5;
      if(droneCount < 10)
        modulus = 4;
      if(droneCount == 9)
        modulus = 3;
      if(droneCount == 4 || droneCount == 5)
        modulus = 2;
      if(droneCount == 2)
        modulus = 1;

      ROS_INFO("global_controller: Go to starting position (hovering).");
      for (size_t i = 0; i < droneCount; i++) // go to starting position (hovering)
      {
        trajectory_msg.header.stamp = ros::Time::now();

        desired_position(0) = ((float)(i%modulus)) * spacingX + offsetX +( (float)(rand()) / ((float)(RAND_MAX/randNoise)) - randNoise/2 ); // * 0.5;
        desired_position(1) = floor((float)(i/modulus)) * spacingY + offsetY +( (float)(rand()) / ((float)(RAND_MAX/randNoise)) - randNoise/2 ); // * 0.5;
        desired_position(2) = 1.0 + ((float)(i%2)) * spacingZ + offsetZ +( (float)(rand()) / ((float)(RAND_MAX/randNoise)) - randNoise/2 ); //* 0.2;
        desired_yaw = 0; // not rotated
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

        ROS_INFO("global_controller: Publishing waypoint on namespace %s: [x=%f, y=%f, z=%f, yaw=%f].",
        nhp[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z(), desired_yaw);

        enable_msg.data = SWARM_DISABLED;

        trajectory_pub[i].publish(trajectory_msg);
        enable_pub[i].publish(enable_msg);
      }
    }
    if(msg->data == 'l') // landing
    {
      ROS_INFO("global_controller: Go to landing position (poweroff).");
      for (size_t i = 0; i < droneCount; i++) // go to starting position (hovering)
      {
        enable_msg.data = SWARM_LANDING;
        enable_pub[i].publish(enable_msg);
      }
      ros::Duration(2.0).sleep();
      ROS_INFO("global_controller: Terminate ROS and shutdown drones.");
      ros::shutdown();
    }
    if(msg->data == 'q') // quit without landing
    {
      ROS_INFO("global_controller: Quit without landing.");
      ROS_INFO("global_controller: Terminate ROS and shutdown drones.");
      ros::shutdown();
    }
    if(msg->data == 's') // swarm mode enable
    {
      ROS_INFO("global_controller: Enable swarm mode.");

      trajectory_msg.header.stamp = ros::Time::now();
      desired_position(0) = 0.0; // x
      desired_position(1) = 2.5; // y
      desired_position(2) = 2.5; // z
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0, &trajectory_msg);
      for (size_t i = 0; i < droneCount; i++) // send target point to swarm
      {
        ROS_INFO("global_controller: Publishing swarm target on namespace %s: [x=%f, y=%f, z=%f].",
        nhp[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z());
        trajectory_pub[i].publish(trajectory_msg);

        enable_msg.data = swarm_mode;
        enable_pub[i].publish(enable_msg);
      }
    }
    if(msg->data == '1' || msg->data == '2' || msg->data == '3' || msg->data == '4' || msg->data == '5') // go to positions
    {
      ROS_INFO("global_controller: Go to position.");

      trajectory_msg.header.stamp = ros::Time::now();
      // defaults
      desired_position(0) = 0.0; // x
      desired_position(1) = 2.5; // y
      desired_position(2) = 2.5; // z

      if(msg->data == '1')
      {
        desired_position(0) = 0.0; // x
        desired_position(1) = 2.5; // y
      }
      if(msg->data == '2')
      {
        desired_position(0) = 0.0; // x
        desired_position(1) = 10.5; // y
      }
      if(msg->data == '3')
      {
        desired_position(0) = 2.5; // x
        desired_position(1) = 6.5; // y
      }
      if(msg->data == '4')
      {
        desired_position(0) = -2.5; // x
        desired_position(1) = 6.5; // y
      }
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0, &trajectory_msg);
      for (size_t i = 0; i < droneCount; i++) // send target point to swarm
      {
        ROS_INFO("global_controller: Publishing swarm target on namespace %s: [x=%f, y=%f, z=%f].",
        nhp[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z());
        trajectory_pub[i].publish(trajectory_msg);
      }
    }
  }
}

void move_marker_beacon(ros::ServiceClient* gazebo_client, int index, float x, float y, float z)
{
  // move gazebo markers
  geometry_msgs::Point pr2_position_green;
  pr2_position_green.x = x;
  pr2_position_green.y = y;
  //pr2_position_green.z = z;
  pr2_position_green.z = 0.2; // fix z to simulate ground-based vehicle.
  geometry_msgs::Quaternion pr2_orientation;
  geometry_msgs::Pose pr2_pose;
  gazebo_msgs::ModelState pr2_modelstate;
  gazebo_msgs::SetModelState srv;
  pr2_orientation.x = 0.0;
  pr2_orientation.y = 0.0;
  pr2_orientation.z = 0.0;
  pr2_orientation.w = 1.0;
  pr2_pose.position = pr2_position_green;
  pr2_pose.orientation = pr2_orientation;
  pr2_modelstate.model_name = (std::string) "marker_green_beacon_" + std::to_string(index);
  pr2_modelstate.pose = pr2_pose;
  srv.request.model_state = pr2_modelstate;
  if(!gazebo_client->call(srv))
      ROS_ERROR("Failed to move green marker! Error msg:%s",srv.response.status_message.c_str());
}

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

  srand((unsigned) (time(0)));

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ros::NodeHandle nhq[N_DRONES_MAX];
  nhp = nhq;

  if (pnh.getParam("droneCount", droneCount))
    ROS_INFO("global_controller: Got param 'droneCount': %d", droneCount);
  else
    ROS_ERROR("global_controller: Failed to get param 'droneCount'");

  if (pnh.getParam("spacingX", spacingX))
    ROS_INFO("global_controller: Got param 'spacingX': %f", spacingX);
  else
    ROS_ERROR("global_controller: Failed to get param 'spacingX'");

  if (pnh.getParam("spacingY", spacingY))
    ROS_INFO("global_controller: Got param 'spacingY': %f", spacingY);
  else
    ROS_ERROR("global_controller: Failed to get param 'spacingY'");

  if (pnh.getParam("spacingZ", spacingZ))
    ROS_INFO("global_controller: Got param 'spacingZ': %f", spacingZ);
  else
    ROS_ERROR("global_controller: Failed to get param 'spacingZ'");

  if (pnh.getParam("offsetX", offsetX))
    ROS_INFO("global_controller: Got param 'offsetX': %f", offsetX);
  else
    ROS_INFO("global_controller: Failed to get param 'offsetX', using default: %f", offsetX);

  if (pnh.getParam("offsetY", offsetY))
    ROS_INFO("global_controller: Got param 'offsetY': %f", offsetY);
  else
    ROS_INFO("global_controller: Failed to get param 'offsetY', using default: %f", offsetY);

  if (pnh.getParam("offsetZ", offsetZ))
    ROS_INFO("global_controller: Got param 'offsetZ': %f", offsetZ);
  else
    ROS_INFO("global_controller: Failed to get param 'offsetZ', using default: %f", offsetZ);

  if (pnh.getParam("randNoise", randNoise))
    ROS_INFO("global_controller: Got param 'randNoise': %f", randNoise);
  else
    ROS_INFO("global_controller: Failed to get param 'randNoise', using default: %f", randNoise);

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
  else if(swarmMode == "reynolds")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_REYNOLDS");
    swarm_mode = SWARM_REYNOLDS;
  }
  else if(swarmMode == "reylimited")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_REYNOLDS_LIMITED");
    swarm_mode = SWARM_REYNOLDS_LIMITED;
  }
  else if(swarmMode == "reyvelocity")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_REYNOLDS_VELOCITY");
    swarm_mode = SWARM_REYNOLDS_VELOCITY;
  }
  else if(swarmMode == "gradient")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_GRADIENT");
    swarm_mode = SWARM_GRADIENT;
  }
  else if(swarmMode == "gradenum")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_GRADIENT_ENUM");
    swarm_mode = SWARM_GRADIENT_ENUM;
  }
  else if(swarmMode == "dist")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_DECLARATIVE_DISTANCES");
    swarm_mode = SWARM_DECLARATIVE_DISTANCES;
  }
  else if(swarmMode == "distgnd")
  {
    ROS_INFO("global_controller: 'swarmMode' recognized as SWARM_DECLARATIVE_DISTANCES_GROUND");
    swarm_mode = SWARM_DECLARATIVE_DISTANCES_GROUND;
  }
  else
    ROS_FATAL("global_controller: 'swarmMode' not recognized (%s)", swarmMode);


  std::string operationMode;
  if (pnh.getParam("operationMode", operationMode))
    ROS_INFO("global_controller: Got param 'operationMode': %s", operationMode.c_str());
  else
    ROS_INFO("global_controller: Failed to get param 'operationMode', defaulting to OPERATION_MODE_TIMED");
  if(operationMode == "interactive")
  {
    ROS_INFO("global_controller: 'operationMode' recognized as OPERATION_MODE_INTERACTIVE");
    operation_mode = OPERATION_MODE_INTERACTIVE;
  }

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  //waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));

  // To publish marker positions for gazebo visual
  ros::ServiceClient gazebo_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  auto keyboard_sub_ = nh.subscribe("/key", 1, &keyboard_callback);

  for (size_t i = 0; i < droneCount; i++)
  {
    nhq[i] = ros::NodeHandle(std::string("/crazyflie2_") + std::to_string(i));

    ROS_INFO("global_controller: Setup publisher %s.", nhq[i].getNamespace().c_str());
    trajectory_pub[i] = nhq[i].advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    enable_pub[i] = nhq[i].advertise<std_msgs::Int32>("enable", 10);
  }

  if(operation_mode == OPERATION_MODE_TIMED)
  {

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
    std_msgs::Int32 enable_msg;

    // Default desired position and yaw.
    Eigen::Vector3d desired_position(0.3, 0.5, 1.0);
    double desired_yaw = 0.0;

    int modulus = 5;
    if(droneCount < 10)
      modulus = 4;
    if(droneCount == 9)
      modulus = 3;
    if(droneCount == 4 || droneCount == 5)
      modulus = 2;

    move_marker_beacon(&gazebo_client_, 0, desired_position.x(), desired_position.y(), desired_position.z());
    ROS_INFO("global_controller (timed): Go to starting position (hovering).");
    for (size_t i = 0; i < droneCount; i++) // go to starting position (hovering)
    {
      trajectory_msg.header.stamp = ros::Time::now();

      desired_position(0) = ((float)(i%modulus)) * spacingX + offsetX +( (float)(rand()) / ((float)(RAND_MAX/randNoise)) - randNoise/2 ); // * 0.5;
      desired_position(1) = floor((float)(i/modulus)) * spacingY + offsetY +( (float)(rand()) / ((float)(RAND_MAX/randNoise)) - randNoise/2 ); // * 0.5;
      desired_position(2) = 2 + ((float)(i%2)) * spacingZ + offsetZ +( (float)(rand()) / ((float)(RAND_MAX/randNoise)) - randNoise/2 ); //* 0.2;
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
    desired_position(2) = 2;
    move_marker_beacon(&gazebo_client_, 0, desired_position.x(), desired_position.y(), desired_position.z());
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0, &trajectory_msg);
    for (size_t i = 0; i < droneCount; i++) // send target point to swarm
    {
      ROS_INFO("global_controller: Publishing swarm target on namespace %s: [%f, %f, %f].",
      nhq[i].getNamespace().c_str(), desired_position.x(), desired_position.y(), desired_position.z());
      trajectory_pub[i].publish(trajectory_msg);
    }

    ros::Duration(10.0).sleep();
    ros::spinOnce();

    ROS_INFO("global_controller: Consider swarm as established.");
    for (size_t i = 0; i < droneCount; i++) // enable swarm mode
    {
      enable_msg.data = swarm_mode + SWARM_PHASE_ESTABLISHED;
      ROS_INFO("global_controller: Publishing enable (established) on namespace %s: %d.", nhq[i].getNamespace().c_str(), enable_msg.data);
      enable_pub[i].publish(enable_msg);
    }

    ros::Duration(10.0).sleep();
    ros::spinOnce();


    trajectory_msg.header.stamp = ros::Time::now();
    desired_position(0) = 0;
    desired_position(1) = 5.0;
    desired_position(2) = 2;
    move_marker_beacon(&gazebo_client_, 0, desired_position.x(), desired_position.y(), desired_position.z());
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
    desired_position(2) = 2;
    move_marker_beacon(&gazebo_client_, 0, desired_position.x(), desired_position.y(), desired_position.z());
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
  }
  if(operation_mode == OPERATION_MODE_INTERACTIVE)
    ros::spin();

  return 0;
}
