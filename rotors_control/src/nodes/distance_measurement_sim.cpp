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

#include "distance_measurement_sim.h"

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

DistanceMeasurementSim::DistanceMeasurementSim() {

    ROS_INFO_ONCE("Started DistanceMeasurementSim");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    distances_pub_ = nh.advertise<std_msgs::Float32MultiArray>("drone_distances", 1);

    InitializeParams();

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
    for (size_t i = 0; i < droneCount_; i++)
    {
      ROS_INFO("DistanceMeasurementSim: Setup subscriber %s/%s.", nhq[i].getNamespace().c_str(), mav_msgs::default_topics::ODOMETRY);
      odometry_sub_[i] = nhq[i].subscribe(mav_msgs::default_topics::ODOMETRY, 1, &DroneStateWithTime::OdometryCallback, &dronestate[i]);
    }
}

DistanceMeasurementSim::~DistanceMeasurementSim(){}

void DistanceMeasurementSim::InitializeParams() {
    ros::NodeHandle pnh("~");

    ROS_INFO_ONCE("[DistanceMeasurementSim] InitializeParams");

    GetRosParameter(pnh, "distances/distance_noise", (float)0.1, &distance_noise_);

    ROS_INFO_ONCE("[DistanceMeasurementSim] GetRosParameter values:");
    ROS_INFO_ONCE("  distances/distance_noise=%f", distance_noise_);

    //Reading the parameters come from the launch file
    std::string dataStoringActive;
    int droneCount;
    double dataStoringTime;

     if (pnh.getParam("droneCount", droneCount)){
        ROS_INFO("Got param 'droneCount': %d", droneCount);
        droneCount_ = droneCount;
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

    for (size_t i = 0; i < droneCount_; i++)
      dronestate[i].SetId((int)i, droneCount_, distance_noise_, dronestate, &distances_pub_);

    ros::NodeHandle nh;
    //timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &DistanceMeasurementSim::CallbackSaveData, this, false, true);
}

void DroneStateWithTime::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("DroneStateWithTime got first OdometryCallback.");
    // received message drone position information
    // odometry_gt_ = EigenOdometry();
    odometry_gt_.timeStampSec = odometry_msg->header.stamp.sec;
    odometry_gt_.timeStampNsec = odometry_msg->header.stamp.nsec;
    odometry_gt_.position[0] = odometry_msg->pose.pose.position.x;
    odometry_gt_.position[1] = odometry_msg->pose.pose.position.y;
    odometry_gt_.position[2] = odometry_msg->pose.pose.position.z;

    ROS_INFO_ONCE("DroneStateWithTime got odometry message: x=%f y=%f z=%f (droneNumber:%d)", odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2], droneNumber_);

    // gaussian random number generator
    std::normal_distribution<float> dist(0.0, distance_noise_);
    //rand_cnt_++;
    //if(rand_cnt_ > 10) // reduce frequency of noise

    for (size_t i = 0; i < droneCount_; i++)
    {
      distances_gt_[i] = sqrt(
        pow(odometry_gt_.position[0] - dronestate_[i].odometry_gt_.position[0], 2) +
        pow(odometry_gt_.position[1] - dronestate_[i].odometry_gt_.position[1], 2) +
        pow(odometry_gt_.position[2] - dronestate_[i].odometry_gt_.position[2], 2));
      float rand_value = std::max(-5*distance_noise_, std::min((float)dist(generator_), 5*distance_noise_));
      distances_[i] = distances_gt_[i] + rand_value;
      ROS_INFO_ONCE("DroneStateWithTime: distance_gt=%f distance=%f (droneNumber:%d, i:%d)", distances_gt_[i], distances_[i], droneNumber_, (int)i);
    }

    std_msgs::Float32MultiArray dat;
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "from_drone";
    dat.layout.dim[1].label = "do_drone";
    dat.layout.dim[0].size = droneCount_;
    dat.layout.dim[1].size = droneCount_;
    dat.layout.dim[0].stride = droneCount_*droneCount_;
    dat.layout.dim[1].stride = droneCount_;
    dat.layout.data_offset = 0;
    std::vector<float> vec(droneCount_*droneCount_, 0);
    for (int i=0; i<droneCount_; i++)
      for (int j=0; j<droneCount_; j++)
        vec[i*droneCount_ + j] = dronestate_[i].distances_gt_[j];
    dat.data = vec;
    distances_pub_->publish(dat);
}

void DroneStateWithTime::SetId(int droneNumber, int droneCount, float position_noise, DroneStateWithTime* dronestate, ros::Publisher* distances_pub)
{
    droneNumber_ = droneNumber;
    droneCount_ = droneCount;
    distance_noise_ = position_noise;

    dronestate_ = dronestate;
    distances_pub_ = distances_pub;
}
}

int main(int argc, char** argv){
    ros::init(argc, argv, "distance_measurement_sim_node");

    ros::NodeHandle nh2;

    rotors_control::DistanceMeasurementSim distance_measurement_sim_node;

    ros::spin();

    return 0;
}
