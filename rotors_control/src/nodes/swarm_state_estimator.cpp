/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2020 Ria Sonecha, MIT, USA
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

#include "swarm_state_estimator.h"

#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <chrono>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <limits>
#include <random>

#include <sys/stat.h>
#include <sys/types.h>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
// #include "rotors_control/crazyflie_complementary_filter.h"

namespace rotors_control {

SwarmStateEstimator::SwarmStateEstimator() {

    ROS_INFO_ONCE("Started SwarmStateEstimator");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

    // Topics subscribe
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &SwarmStateEstimator::OdometryCallback, this);
    distances_sub_ = nh.subscribe("/drone_distances", 1, &SwarmStateEstimator::DistancesCallback, this);

    // To publish the current pose for gazebo visual
    visual_cnt_ = 0;
    gazebo_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

SwarmStateEstimator::~SwarmStateEstimator(){}

void SwarmStateEstimator::InitializeParams() {
    ros::NodeHandle pnh("~");

    ROS_INFO_ONCE("[SwarmStateEstimator] InitializeParams");

    //Reading the parameters come from the launch file
    std::string dataStoringActive;
    int droneCount;
    int droneNumber;
    double dataStoringTime;

    if (pnh.getParam("droneCount", droneCount)){
       ROS_INFO("Got param 'droneCount': %d", droneCount);
       droneCount_ = droneCount;
    }
    else
       ROS_ERROR("Failed to get param 'droneCount'");

    if (pnh.getParam("droneNumber", droneNumber)){
        ROS_INFO("Got param 'droneNumber': %d", droneNumber);
        droneNumber_ = droneNumber;
    }
    else
        ROS_ERROR("Failed to get param 'droneCount'");

    if (pnh.getParam("csvFilesStoring", dataStoringActive)){
        ROS_INFO("Got param 'csvFilesStoring': %s", dataStoringActive.c_str());

        dataStoring_active_ = true;
        if(dataStoringActive == "yes" || dataStoringActive == "distance")
            dataStoring_active_ = true;
    }
    else {
       ROS_ERROR("Failed to get param 'csvFilesStoring'");
       dataStoring_active_ = true;
    }

    if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
        ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
    }
    else {
       ROS_ERROR("Failed to get param 'csvFilesStoringTime'");
       dataStoringTime = 9999;
    }

    ros::NodeHandle nh;
    //timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &SwarmStateEstimator::CallbackSaveData, this, false, true);
}

void SwarmStateEstimator::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("SwarmStateEstimator got first odometry message.");

    // received message drone position information, store ground-truth pose
    odometry_gt_.timeStampSec = odometry_msg->header.stamp.sec;
    odometry_gt_.timeStampNsec = odometry_msg->header.stamp.nsec;
    odometry_gt_.position[0] = odometry_msg->pose.pose.position.x;
    odometry_gt_.position[1] = odometry_msg->pose.pose.position.y;
    odometry_gt_.position[2] = odometry_msg->pose.pose.position.z;
    ROS_INFO_ONCE("SwarmStateEstimator got odometry message: x=%f y=%f z=%f (%d)", odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2], droneNumber_);


/*  if(droneNumber_ == 0) // move red indicator sphere for testing purposes
    {
      geometry_msgs::Point pr2_position;
      pr2_position.x = odometry_msg->pose.pose.position.x + sin(visual_cnt_)* 0.1;
      pr2_position.y = odometry_msg->pose.pose.position.y + cos(visual_cnt_)* 0.1;
      pr2_position.z = odometry_msg->pose.pose.position.z + cos(visual_cnt_)* 0.1;
      geometry_msgs::Quaternion pr2_orientation;
      pr2_orientation.x = 0.0;
      pr2_orientation.y = 0.0;
      pr2_orientation.z = 0.0;
      pr2_orientation.w = 1.0;
      geometry_msgs::Pose pr2_pose;
      pr2_pose.position = pr2_position;
      pr2_pose.orientation = pr2_orientation;
      gazebo_msgs::ModelState pr2_modelstate;
      pr2_modelstate.model_name = (std::string) "marker_crazyflie2_" + std::to_string(droneNumber_);
      pr2_modelstate.pose = pr2_pose;
      gazebo_msgs::SetModelState srv;
      srv.request.model_state = pr2_modelstate;
      if(!gazebo_client_.call(srv))
        ROS_ERROR("Failed to move marker! Error msg:%s",srv.response.status_message.c_str());
      visual_cnt_ += 2*3.14159265358979323846 / 200;
    } */
}

void SwarmStateEstimator::DistancesCallback(const std_msgs::Float32MultiArray& distances_msg) {

    ROS_INFO("DistancesCallback got distances message.");

    for (size_t i = 0; i < droneCount_; i++)
    {
      odometry_estimate_[i].position[0] = 0;
      odometry_estimate_[i].position[1] = 0;
      odometry_estimate_[i].position[2] = 0;
    }

    float distances[N_DRONES_MAX][N_DRONES_MAX];

    for (size_t i = 0; i < droneCount_; i++)
      for (size_t j = 0; j < droneCount_; j++)
      {
        distances[i][j] = distances_msg.data[i*droneCount_ + j];
        ROS_INFO("DistancesCallback drone#%d -> drone#%d: distance=%f.", i, j, distances[i][j]);
      }

    if(droneNumber_ == 0) // move red indicator sphere for testing purposes
    {
          geometry_msgs::Point pr2_position;
          pr2_position.x = odometry_gt_.position[0] + sin(visual_cnt_)* 0.1;
          pr2_position.y = odometry_gt_.position[1] + cos(visual_cnt_)* 0.1;
          pr2_position.z = odometry_gt_.position[2];
          geometry_msgs::Quaternion pr2_orientation;
          pr2_orientation.x = 0.0;
          pr2_orientation.y = 0.0;
          pr2_orientation.z = 0.0;
          pr2_orientation.w = 1.0;
          geometry_msgs::Pose pr2_pose;
          pr2_pose.position = pr2_position;
          pr2_pose.orientation = pr2_orientation;
          gazebo_msgs::ModelState pr2_modelstate;
          pr2_modelstate.model_name = (std::string) "marker_crazyflie2_" + std::to_string(droneNumber_);
          pr2_modelstate.pose = pr2_pose;
          gazebo_msgs::SetModelState srv;
          srv.request.model_state = pr2_modelstate;
          if(!gazebo_client_.call(srv))
            ROS_ERROR("Failed to move marker! Error msg:%s",srv.response.status_message.c_str());
          visual_cnt_ += 2*3.14159265358979323846 / 200;
    }
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "swarm_state_estimator_node");

    ros::NodeHandle nh2;

    rotors_control::SwarmStateEstimator swarm_state_estimator_node;

    ros::spin();

    return 0;
}
