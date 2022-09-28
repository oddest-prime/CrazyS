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
#include "rotors_control/Eigen.h"

namespace rotors_control {

DistanceMeasurementSim::DistanceMeasurementSim() {

    ROS_INFO_ONCE("Started DistanceMeasurementSim");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    distances_pub_ = nh.advertise<std_msgs::Float32MultiArray>("drone_distances", 1);
    positions_pub_ = nh.advertise<std_msgs::Float32MultiArray>("drone_positions", 1);
    elevation_pub_ = nh.advertise<std_msgs::Float32MultiArray>("drone_elevation", 1);
    beacons_pub_ = nh.advertise<std_msgs::Float32MultiArray>("beacon_distances", 1);

    modelstate_sub_ = nh.subscribe("/gazebo/model_states", 1, &DistanceMeasurementSim::ModelstateCallback, this);

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
      enable_sub_[i] = nhq[i].subscribe("enable", 1, &DroneStateWithTime::EnableCallback, &dronestate[i]);
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
    int beaconCount;
    double dataStoringTime;

    if (pnh.getParam("droneCount", droneCount)){
        ROS_INFO("Got param 'droneCount': %d", droneCount);
        droneCount_ = droneCount;
    }
    else
        ROS_ERROR("Failed to get param 'droneCount'");
    if (pnh.getParam("beaconCount", beaconCount)){
       ROS_INFO("Got param 'beaconCount': %d", beaconCount);
       beaconCount_ = beaconCount;
    }
    else
       ROS_ERROR("Failed to get param 'beaconCount'");

    bool dataStoring_active;
    if (pnh.getParam("csvFilesStoring", dataStoringActive)){
        ROS_INFO("Got param 'csvFilesStoring': %s", dataStoringActive.c_str());

        dataStoring_active = true;
        if(dataStoringActive == "yes" || dataStoringActive == "distance")
            dataStoring_active = true;
    }
    else {
       ROS_ERROR("Failed to get param 'csvFilesStoring'");
       dataStoring_active = true;
    }

    if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
        ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
    }
    else {
       ROS_ERROR("Failed to get param 'csvFilesStoringTime'");
       dataStoringTime = 9999;
    }

    for (size_t i = 0; i < droneCount_; i++)
      dronestate[i].SetId((int)i, droneCount_, beaconCount_, distance_noise_, dronestate, &distances_pub_, &positions_pub_, &elevation_pub_, &beacons_pub_, dataStoring_active, beacon_gt_);

    ros::NodeHandle nh;
    timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &DistanceMeasurementSim::CallbackSaveData, this, false, true);
}

void DistanceMeasurementSim::ModelstateCallback(const gazebo_msgs::ModelStatesConstPtr& modelstates_msg)
{
    ROS_INFO_ONCE("DistanceMeasurementSim got first ModelstateCallback.");

    int b = 0;
    for (size_t i = 0; i < modelstates_msg->name.size(); i++) // iterate over models
    {
        if(modelstates_msg->name[i] == "marker_green_beacon_0") // todo adjust to parse all beacons!
        {
            // save beacon positions
            beacon_gt_[b][0] = modelstates_msg->pose[i].position.x;
            beacon_gt_[b][1] = modelstates_msg->pose[i].position.y;
            beacon_gt_[b][2] = modelstates_msg->pose[i].position.z;
            ROS_INFO_ONCE("model %d: %s (%d) at location %s", (int)i, modelstates_msg->name[i].c_str(), b, VectorToString(beacon_gt_[b]).c_str());
            b ++;
        }
    }
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

    // for logging into files
    std::stringstream tempDistance;
    tempDistance.precision(24);
    tempDistance << odometry_gt_.timeStampSec << "," << odometry_gt_.timeStampNsec << "," << enable_swarm_ << ",";
    std::stringstream tempMetrics;
    tempMetrics.precision(24);
    tempMetrics << odometry_gt_.timeStampSec << "," << odometry_gt_.timeStampNsec << "," << enable_swarm_ << ",";
    std::stringstream tempState;
    tempState.precision(24);
    tempState << odometry_gt_.timeStampSec << "," << odometry_gt_.timeStampNsec << "," << enable_swarm_ << ",";

    ROS_INFO_ONCE("DroneStateWithTime got odometry message: x=%f y=%f z=%f (droneNumber:%d)", odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2], droneNumber_);

    // gaussian random number generator
    std::normal_distribution<float> dist(0.0, distance_noise_);
    //rand_cnt_++;
    //if(rand_cnt_ > 10) // reduce frequency of noise

    Vector3f swarm_center_gt = {0,0,0};
    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
    {
        swarm_center_gt[0] += dronestate_[i].odometry_gt_.position[0];
        swarm_center_gt[1] += dronestate_[i].odometry_gt_.position[1];
        swarm_center_gt[2] += dronestate_[i].odometry_gt_.position[2];
    }
    swarm_center_gt /= droneCount_;

    // calculate distance to other drones
    float dist_min_gt = FLT_MAX;
    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
    {
      distances_gt_[i] = sqrt(
        pow(odometry_gt_.position[0] - dronestate_[i].odometry_gt_.position[0], 2) +
        pow(odometry_gt_.position[1] - dronestate_[i].odometry_gt_.position[1], 2) +
        pow(odometry_gt_.position[2] - dronestate_[i].odometry_gt_.position[2], 2));
      float rand_value = std::max(-5*distance_noise_, std::min((float)dist(generator_), 5*distance_noise_));
      distances_[i] = distances_gt_[i] + rand_value;
      if(i != droneNumber_)
          dist_min_gt = std::min(distances_gt_[i], dist_min_gt);
      if(dataStoring_active_) // save data for log files
          tempDistance << distances_gt_[i] << ",";

      ROS_INFO_ONCE("DroneStateWithTime: distance_gt=%f distance=%f (droneNumber:%d, i:%d)", distances_gt_[i], distances_[i], droneNumber_, (int)i);
    }

    // calculate distance to beacons
    for (size_t i = 0; i < beaconCount_; i++) // iterate over beacons
    {
        ROS_INFO_ONCE("DroneStateWithTime beacon %d at location %s", (int)i, VectorToString(beacon_gt_[i]).c_str());
        beacon_distances_gt_[i] = sqrt(
          pow(odometry_gt_.position[0] - beacon_gt_[i][0], 2) +
          pow(odometry_gt_.position[1] - beacon_gt_[i][1], 2) +
          pow(odometry_gt_.position[2] - beacon_gt_[i][2], 2));
        float rand_value = std::max(-5*distance_noise_, std::min((float)dist(generator_), 5*distance_noise_));
        beacon_distances_[i] = beacon_distances_gt_[i] + rand_value;
        ROS_INFO_ONCE("DroneStateWithTime %d beacon %d at location %s distance %f", droneNumber_, (int)i, VectorToString(beacon_gt_[i]).c_str(), beacon_distances_gt_[i]);
    }

    Vector3f vector_to_center_gt = {(float)(swarm_center_gt[0] - odometry_gt_.position[0]), (float)(swarm_center_gt[1] - odometry_gt_.position[1]), (float)(swarm_center_gt[2] - odometry_gt_.position[2])};

    // save data for log files
    if(dataStoring_active_)
    {
        ROS_INFO_ONCE("DroneStateWithTime (%d) swarm_center_gt:     %s", droneNumber_, VectorToString(swarm_center_gt).c_str());
        ROS_INFO_ONCE("DroneStateWithTime (%d) vector_to_center_gt: %s", droneNumber_, VectorToString(vector_to_center_gt).c_str());
        ROS_INFO_ONCE("DroneStateWithTime (%d) dist_min_gt=%f r=%f ", droneNumber_, dist_min_gt, vector_to_center_gt.norm());

        tempMetrics << droneCount_ << ","; // number of drones
        tempMetrics << beaconCount_ << ","; // number of beacons
        tempMetrics << dist_min_gt << ","; // minimum distance to other drones
        tempMetrics << vector_to_center_gt.norm() << ","; // length of vector, distance from the center (radius of whole flock = compactness)
        tempMetrics << beacon_distances_gt_[0] << ","; // distance to beacon 0

        tempState << odometry_gt_.position[0] << "," << odometry_gt_.position[1] << "," << odometry_gt_.position[2] << ",";

        for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
          tempDistance << distances_gt_[i] << "," << distances_[i] << ","; // distance to quadcopter i (first ground truth, then with measurement noise)
    }

    // publish distances message
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
    for (size_t i = 0; i < droneCount_; i++)
      for (size_t j = 0; j < droneCount_; j++)
        vec[i*droneCount_ + j] = dronestate_[i].distances_gt_[j];
    dat.data = vec;
    distances_pub_->publish(dat);

    // publish positions message
    std_msgs::Float32MultiArray pos;
    pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
    pos.layout.dim.push_back(std_msgs::MultiArrayDimension());
    pos.layout.dim[0].label = "drone";
    pos.layout.dim[1].label = "dimension";
    pos.layout.dim[0].size = droneCount_;
    pos.layout.dim[1].size = 3;
    pos.layout.dim[0].stride = droneCount_*3;
    pos.layout.dim[1].stride = 3;
    pos.layout.data_offset = 0;
    std::vector<float> posi(droneCount_*droneCount_, 0);
    for (size_t i = 0; i < droneCount_; i++)
    {
      posi[i*3 + 0] = dronestate_[i].odometry_gt_.position[0];
      posi[i*3 + 1] = dronestate_[i].odometry_gt_.position[1];
      posi[i*3 + 2] = dronestate_[i].odometry_gt_.position[2];
    }
    pos.data = posi;
    positions_pub_->publish(pos);

    // publish elevation message
    std_msgs::Float32MultiArray el;
    el.layout.dim.push_back(std_msgs::MultiArrayDimension());
    el.layout.dim[0].label = "drone";
    el.layout.dim[0].size = droneCount_;
    el.layout.dim[0].stride = droneCount_*droneCount_;
    el.layout.data_offset = 0;
    std::vector<float> elev(droneCount_, 0);
    for (size_t i = 0; i < droneCount_; i++)
        elev[i] = dronestate_[i].odometry_gt_.position[2];
    el.data = elev;
    elevation_pub_->publish(el);

    // publish distances to beacons message
    std_msgs::Float32MultiArray be;
    be.layout.dim.push_back(std_msgs::MultiArrayDimension());
    be.layout.dim.push_back(std_msgs::MultiArrayDimension());
    be.layout.dim[0].label = "from_drone";
    be.layout.dim[1].label = "to_beacon";
    be.layout.dim[0].size = droneCount_;
    be.layout.dim[1].size = beaconCount_;
    be.layout.dim[0].stride = droneCount_*beaconCount_;
    be.layout.dim[1].stride = beaconCount_;
    be.layout.data_offset = 0;
    std::vector<float> bec(droneCount_*beaconCount_, 0);
    for (size_t i = 0; i < droneCount_; i++)
      for (size_t j = 0; j < beaconCount_; j++)
        bec[i*beaconCount_ + j] = dronestate_[i].beacon_distances_gt_[j];
    be.data = bec;
    beacons_pub_->publish(be);

    if(dataStoring_active_) // save data for log files
    {
        tempDistance << "\n";
        listDistance_.push_back(tempDistance.str());
        tempMetrics << "\n";
        listMetrics_.push_back(tempMetrics.str());
        tempState << "\n";
        listState_.push_back(tempState.str());
    }

}

void DroneStateWithTime::EnableCallback(const std_msgs::Int32ConstPtr& enable_msg) {
  ROS_INFO("DroneStateWithTime got enable message: %d", enable_msg->data);

  enable_swarm_ = enable_msg->data;
}

//The callback saves data into csv files
void DistanceMeasurementSim::CallbackSaveData(const ros::TimerEvent& event){
  ROS_INFO("DistanceMeasurementSim CallbackSavaData.");
  for (size_t i = 0; i < droneCount_; i++)
    dronestate[i].FileSaveData();
}

void DroneStateWithTime::FileSaveData(void){

      if(!dataStoring_active_){
         return;
      }

      std::ofstream fileDistance;
      std::ofstream fileMetrics;
      std::ofstream fileState;

      ROS_INFO("DroneStateWithTime FileSaveData. droneNumber: %d", droneNumber_);

      if(mkdir("/tmp/log_output/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
          if(errno != EEXIST)
             ROS_ERROR("Cannot create directory /tmp/log_output/");

      fileDistance.open(std::string("/tmp/log_output/DistSimDistance") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      fileMetrics.open(std::string("/tmp/log_output/DistSimMetrics") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      fileState.open(std::string("/tmp/log_output/DistSimState") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);

      // Saving distances from every to every drone in a file
      for (unsigned n=0; n < listDistance_.size(); ++n) {
          fileDistance << listDistance_.at( n );
      }
      // Saving quality metrics in a file
      for (unsigned n=0; n < listMetrics_.size(); ++n) {
          fileMetrics << listMetrics_.at( n );
      }
      // Saving states in a file
      for (unsigned n=0; n < listState_.size(); ++n) {
          fileState << listState_.at( n );
      }

      // Closing all opened files
      fileDistance.close();
      fileMetrics.close();
      fileState.close();

      // To have a one shot storing
      // dataStoring_active_ = false;
}

void DroneStateWithTime::SetId(int droneNumber, int droneCount, int beaconCount, float position_noise, DroneStateWithTime* dronestate, ros::Publisher* distances_pub, ros::Publisher* positions_pub, ros::Publisher* elevation_pub, ros::Publisher* beacons_pub, bool dataStoring_active, Vector3f* beacon_gt)
{
    droneNumber_ = droneNumber;
    droneCount_ = droneCount;
    beaconCount_ = beaconCount;
    distance_noise_ = position_noise;
    dataStoring_active_ = dataStoring_active;

    dronestate_ = dronestate;
    distances_pub_ = distances_pub;
    positions_pub_ = positions_pub;
    elevation_pub_ = elevation_pub;
    beacons_pub_ = beacons_pub;

    beacon_gt_ = beacon_gt;
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "distance_measurement_sim_node");

    ros::NodeHandle nh2;

    rotors_control::DistanceMeasurementSim distance_measurement_sim_node;

    ros::spin();

    return 0;
}
