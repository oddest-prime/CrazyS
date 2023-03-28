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

    old_timeStamp_ = 0;

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    positions_pub_ = nh.advertise<std_msgs::Float32MultiArray>("drone_positions", 1);
    elevation_pub_ = nh.advertise<std_msgs::Float32MultiArray>("drone_elevation", 1);

    InitializeParams();

    modelstate_sub_ = nh.subscribe("/gazebo/model_states", 1, &DistanceMeasurementSim::ModelstateCallback, this);
    logsave_sub_ = nh.subscribe("logsave", 1, &DistanceMeasurementSim::SaveLogCallback, this);

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
      // mav_msgs::default_topics::ODOMETRY
      ROS_INFO("DistanceMeasurementSim: Setup subscriber %s/%s.", nhq[i].getNamespace().c_str(), "odometry");
      odometry_sub_[i] = nhq[i].subscribe("odometry", 1, &DroneStateWithTime::OdometryCallback, &dronestate[i]);
      enable_sub_[i] = nhq[i].subscribe("enable", 1, &DroneStateWithTime::EnableCallback, &dronestate[i]);
      distances_pub_[i] = nhq[i].advertise<std_msgs::Float32MultiArray>("drone_distances", 1);
      beacons_pub_[i] = nhq[i].advertise<std_msgs::Float32MultiArray>("beacon_distances", 1);

      dronestate[i].SetId(this, (int)i, droneCount_, beaconCount_,
                          distance_noise_, elevation_noise_, noise_color_,
                          distance_max_rate_, elevation_max_rate_, dronestate,
                          &(distances_pub_[i]), &positions_pub_, &elevation_pub_, &(beacons_pub_[i]),
                          dataStoring_active_, beacon_gt_, beacon_air_, &swarm_center_gt_);
    }
}

DistanceMeasurementSim::~DistanceMeasurementSim(){}

void DistanceMeasurementSim::InitializeParams() {
    ros::NodeHandle pnh("~");

    ROS_INFO_ONCE("[DistanceMeasurementSim] InitializeParams");

    GetRosParameter(pnh, "swarm/distance_noise", (float)0.1, &distance_noise_);
    GetRosParameter(pnh, "swarm/elevation_noise", (float)0.1, &elevation_noise_);
    GetRosParameter(pnh, "swarm/noise_color", (int)NOISE_COLOR_WHITE, &noise_color_);
    GetRosParameter(pnh, "swarm/distance_max_rate", (float)100, &distance_max_rate_);
    GetRosParameter(pnh, "swarm/elevation_max_rate", (float)100, &elevation_max_rate_);

    ROS_INFO_ONCE("[DistanceMeasurementSim] GetRosParameter values:");
    ROS_INFO_ONCE("  swarm/distance_noise=%f", distance_noise_);
    ROS_INFO_ONCE("  swarm/elevation_noise=%f", elevation_noise_);
    ROS_INFO_ONCE("  swarm/noise_color=%d", noise_color_);
    ROS_INFO_ONCE("  swarm/distance_max_rate=%f", distance_max_rate_);
    ROS_INFO_ONCE("  swarm/elevation_max_rate=%f", elevation_max_rate_);

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

    if(droneCount_ > N_DRONES_MAX)
      ROS_FATAL("droneCount_ (%d) exceeds N_DRONES_MAX (%d)", droneCount_, N_DRONES_MAX);

    ros::NodeHandle nh;
    timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &DistanceMeasurementSim::CallbackSaveData, this, false, true);
}

void DistanceMeasurementSim::ModelstateCallback(const gazebo_msgs::ModelStatesConstPtr& modelstates_msg)
{
    ROS_INFO_ONCE("DistanceMeasurementSim got ModelstateCallback.");

    int b = 0;
    for (size_t i = 0; i < modelstates_msg->name.size(); i++) // iterate over models
    {
        for (size_t j = 0; j < beaconCount_; j++) // iterate over all beacons
        {
            if(modelstates_msg->name[i] == std::string("marker_green_beacon_").append(std::to_string(j)))
            {
                char beacon0_moved = 0;
                // extra for beacon0, if its position changed
                if( fabs(beacon_gt_[b][0] - modelstates_msg->pose[i].position.x) > BEACON_MOVED_SMALL_EPS ||
                    fabs(beacon_gt_[b][1] - modelstates_msg->pose[i].position.y) > BEACON_MOVED_SMALL_EPS ||
                    fabs(beacon_gt_[b][2] - modelstates_msg->pose[i].position.z) > BEACON_MOVED_SMALL_EPS)
                    beacon0_moved = 1;

                // save beacon positions
                beacon_gt_[b][0] = modelstates_msg->pose[i].position.x;
                beacon_gt_[b][1] = modelstates_msg->pose[i].position.y;
                beacon_gt_[b][2] = modelstates_msg->pose[i].position.z;
                beacon_air_[b][0] = modelstates_msg->pose[i].position.x;
                beacon_air_[b][1] = modelstates_msg->pose[i].position.y;
                beacon_air_[b][2] = 5.0; // TODO: take from config.
                ROS_INFO_ONCE("model %d: %s (%d) at location %s", (int)i, modelstates_msg->name[i].c_str(), b, VectorToString(beacon_gt_[b]).c_str());
                b ++;

                if(beacon0_moved)
                  this->RecalcTargetSpeed(ros::Time::now());
            }
        }
/*
        if(modelstates_msg->name[i] == "jackal")
        {
            // save jackal position as beacon 1
            beacon_gt_[1][0] = modelstates_msg->pose[i].position.x;
            beacon_gt_[1][1] = modelstates_msg->pose[i].position.y;
            beacon_gt_[1][2] = modelstates_msg->pose[i].position.z;
            beacon_gt_[1][2] = 3.5; // TODO: this is just for testing!
            ROS_INFO_ONCE("model %d: %s (%d) at location %s", (int)i, modelstates_msg->name[i].c_str(), 1, VectorToString(beacon_gt_[b]).c_str());
        }
*/
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

    ros::Time odometry_msg_time(odometry_msg->header.stamp.sec, odometry_msg->header.stamp.nsec);

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
    std::stringstream tempCentroid;
    tempCentroid.precision(24);
    tempCentroid << odometry_gt_.timeStampSec << "," << odometry_gt_.timeStampNsec << "," << enable_swarm_ << ",";

    ROS_INFO_ONCE("DroneStateWithTime got odometry message: x=%f y=%f z=%f (droneNumber:%d)", odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2], droneNumber_);

    // gaussian random number generator
    std::normal_distribution<float> dist_distribution(0.0, distance_noise_);
    std::normal_distribution<float> elev_distribution(0.0, elevation_noise_);
    //rand_cnt_++;
    //if(rand_cnt_ > 10) // reduce frequency of noise

    // Vector3f swarm_center_gt_ = {0,0,0};
    (*swarm_center_gt_)[0] = 0;
    (*swarm_center_gt_)[1] = 0;
    (*swarm_center_gt_)[2] = 0;
    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
    {
        (*swarm_center_gt_)[0] += dronestate_[i].odometry_gt_.position[0];
        (*swarm_center_gt_)[1] += dronestate_[i].odometry_gt_.position[1];
        (*swarm_center_gt_)[2] += dronestate_[i].odometry_gt_.position[2];
    }
    (*swarm_center_gt_) /= droneCount_;

    // add elevation noise
    float rand_value = std::max(-5*elevation_noise_, std::min((float)elev_distribution(generator_), 5*elevation_noise_));
    elevation_ = odometry_gt_.position[2] + rand_value;

    // calculate distance to other drones
    float dist_min_gt = FLT_MAX;
    float dist_max_gt = 0;
    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
    {
      distances_gt_[i] = sqrt(
        pow(odometry_gt_.position[0] - dronestate_[i].odometry_gt_.position[0], 2) +
        pow(odometry_gt_.position[1] - dronestate_[i].odometry_gt_.position[1], 2) +
        pow(odometry_gt_.position[2] - dronestate_[i].odometry_gt_.position[2], 2));

      if(noise_color_ == NOISE_COLOR_WHITE) // white noise
      {
        float rand_value = std::max(-5*distance_noise_, std::min((float)dist_distribution(generator_), 5*distance_noise_));
        distances_[i] = distances_gt_[i] + rand_value; // add distance noise
      }
      else if(noise_color_ == NOISE_COLOR_PINK_A) // pink noise
      {
        float rand_value = (float)dist_distribution(generator_);
        distances_iir_[i] = 0.99 * distances_iir_[i] + 0.01 * rand_value;
        rand_value = (float)dist_distribution(generator_);
        distances_[i] = distances_gt_[i] + (distances_iir_[i] + rand_value / 30) * 12.0; // add distance noise
      }
      else if(noise_color_ == NOISE_COLOR_PINK_B) // pink noise
      {
        float rand_value = (float)dist_distribution(generator_);
        distances_iir_[i] = 0.995 * distances_iir_[i] + 0.005 * rand_value;
        rand_value = (float)dist_distribution(generator_);
        distances_[i] = distances_gt_[i] + (distances_iir_[i] + rand_value / 50) * 19.7; // add distance noise
      }
      else
        ROS_FATAL("DroneStateWithTime (%d) invalid noise color: %d", droneNumber_, noise_color_);

      if(i != droneNumber_)
          dist_min_gt = std::min(distances_gt_[i], dist_min_gt);
      if(i != droneNumber_)
          dist_max_gt = std::max(distances_gt_[i], dist_max_gt);
      if(dataStoring_active_) // save data for log files
          tempDistance << distances_gt_[i] << ",";

      ROS_INFO_ONCE("DroneStateWithTime: distance_gt=%f distance=%f (droneNumber:%d, i:%d)", distances_gt_[i], distances_[i], droneNumber_, (int)i);
    }

    // calculate distance to beacons
    for (size_t i = 0; i < beaconCount_; i++) // iterate over beacons
    {
        ROS_INFO_ONCE("DroneStateWithTime (%d) beacon %d at location %s", droneNumber_, (int)i, VectorToString(beacon_gt_[i]).c_str());
        beacon_distances_gt_[i] = sqrt(
          pow(odometry_gt_.position[0] - beacon_gt_[i][0], 2) +
          pow(odometry_gt_.position[1] - beacon_gt_[i][1], 2) +
          pow(odometry_gt_.position[2] - beacon_gt_[i][2], 2));
        beacon_distances_air_[i] = sqrt(
          pow(odometry_gt_.position[0] - beacon_air_[i][0], 2) +
          pow(odometry_gt_.position[1] - beacon_air_[i][1], 2) +
          pow(odometry_gt_.position[2] - beacon_air_[i][2], 2));

        if(noise_color_ == NOISE_COLOR_WHITE) // white noise
        {
          float rand_value = std::max(-5*distance_noise_, std::min((float)dist_distribution(generator_), 5*distance_noise_));
          beacon_distances_[i] = beacon_distances_gt_[i] + rand_value;
        }
        else if(noise_color_ == NOISE_COLOR_PINK_A) // pink noise
        {
          float rand_value = (float)dist_distribution(generator_);
          //beacon_distances_iir_[i] = 0.995 * beacon_distances_iir_[i] + 0.005 * rand_value;
          beacon_distances_iir_[i] = 0.99 * beacon_distances_iir_[i] + 0.01 * rand_value;
          rand_value = (float)dist_distribution(generator_);
          //beacon_distances_[i] = beacon_distances_gt_[i] + (beacon_distances_iir_[i] + rand_value / 50) * 19.7; // add distance noise
          beacon_distances_[i] = beacon_distances_gt_[i] + (beacon_distances_iir_[i] + rand_value / 30) * 12.0; // add distance noise
        }
        else if(noise_color_ == NOISE_COLOR_PINK_B) // pink noise
        {
          float rand_value = (float)dist_distribution(generator_);
          beacon_distances_iir_[i] = 0.995 * beacon_distances_iir_[i] + 0.005 * rand_value;
          rand_value = (float)dist_distribution(generator_);
          beacon_distances_[i] = beacon_distances_gt_[i] + (beacon_distances_iir_[i] + rand_value / 50) * 19.7; // add distance noise
        }
        else
          ROS_FATAL("DroneStateWithTime (%d) invalid noise color: %d", droneNumber_, noise_color_);

        ROS_INFO_ONCE("DroneStateWithTime (%d) beacon %d at location %s distance %f", droneNumber_, (int)i, VectorToString(beacon_gt_[i]).c_str(), beacon_distances_gt_[i]);
    }

    Vector3f vector_to_center_gt = {(float)((*swarm_center_gt_)[0] - odometry_gt_.position[0]), (float)((*swarm_center_gt_)[1] - odometry_gt_.position[1]), (float)((*swarm_center_gt_)[2] - odometry_gt_.position[2])};
    Vector3f beacon0_to_center_gt = {(float)((*swarm_center_gt_)[0] - beacon_gt_[0][0]), (float)((*swarm_center_gt_)[1] - beacon_gt_[0][1]), (float)((*swarm_center_gt_)[2] - beacon_gt_[0][2])};

    // save data for log files
    if(dataStoring_active_)
    {
        ROS_INFO_ONCE("DroneStateWithTime (%d) swarm_center_gt_:     %s", droneNumber_, VectorToString((*swarm_center_gt_)).c_str());
        ROS_INFO_ONCE("DroneStateWithTime (%d) vector_to_center_gt: %s", droneNumber_, VectorToString(vector_to_center_gt).c_str());
        ROS_INFO_ONCE("DroneStateWithTime (%d) dist_min_gt=%f r=%f ", droneNumber_, dist_min_gt, vector_to_center_gt.norm());

        tempMetrics << droneCount_ << ","; // number of drones
        tempMetrics << beaconCount_ << ","; // number of beacons
        tempMetrics << dist_min_gt << ","; // minimum distance to other drones
        tempMetrics << vector_to_center_gt.norm() << ","; // length of vector, distance from the center (radius of whole flock = compactness)
        tempMetrics << beacon_distances_air_[0] << ","; // distance to beacon 0 (moved to air)
        tempMetrics << dist_max_gt << ","; // maximum distance to other drones
        tempMetrics << beacon_distances_gt_[0] << ","; // distance to beacon 0

        tempState << odometry_gt_.position[0] << "," << odometry_gt_.position[1] << "," << odometry_gt_.position[2] << ",";

        tempCentroid << (*swarm_center_gt_)[0] << "," << (*swarm_center_gt_)[1] << "," << (*swarm_center_gt_)[2] << ","; // position of centroid
        tempCentroid << beacon_gt_[0][0] << "," << beacon_gt_[0][1] << "," << beacon_gt_[0][2] << ","; // position of beacon0 = target
        tempCentroid << beacon0_to_center_gt.norm() << ","; // length of vector, distance from the centroid to beacon0

        //for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
        //  tempDistance << distances_gt_[i] << "," << distances_[i] << ","; // distance to quadcopter i (first ground truth, then with measurement noise)
    }

    // rate limitation based on time since last published message on that topic
    int distances_pub_now = false;
    ros::Duration distances_pub_last_ago = odometry_msg_time - distances_pub_last_time_;
    if(distances_pub_last_ago.toSec() > 1.0/distance_max_rate_)
    {
        ROS_INFO_ONCE("DroneStateWithTime (%d) distances_pub_now %lf sec since last; approx. rate: %.1f; max. rate: %.1f", droneNumber_, distances_pub_last_ago.toSec(), 1.0/distances_pub_last_ago.toSec(), distance_max_rate_);
        distances_pub_now = true;
        distances_pub_last_time_ = odometry_msg_time;
    }
    int elevation_pub_now = false;
    ros::Duration elevation_pub_last_ago = odometry_msg_time - elevation_pub_last_time_;
    if(elevation_pub_last_ago.toSec() > 1.0/elevation_max_rate_)
    {
        ROS_INFO_ONCE("DroneStateWithTime (%d) elevation_pub_now %lf sec since last; approx. rate: %.1f; max. rate: %.1f", droneNumber_, elevation_pub_last_ago.toSec(), 1.0/elevation_pub_last_ago.toSec(), elevation_max_rate_);
        elevation_pub_now = true;
        elevation_pub_last_time_ = odometry_msg_time;
    }

/*
    // publish distances message (old version, this contains all distances for every drone to every)
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
        vec[i*droneCount_ + j] = dronestate_[i].distances_[j]; // distances with simulated sensor noise
      //vec[i*droneCount_ + j] = dronestate_[i].distances_gt_[j]; // ground-truth only for debugging
    dat.data = vec;
    if(distances_pub_now)
        distances_pub_->publish(dat);
*/

    // publish distances message (new version, this only contains the distance from the own drone to all others)
    std_msgs::Float32MultiArray dat;
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "to_drone";
    dat.layout.dim[0].size = droneCount_;
    dat.layout.dim[0].stride = droneCount_;
    dat.layout.data_offset = 0;
    std::vector<float> vec(droneCount_, 0);
    for (size_t j = 0; j < droneCount_; j++)
      vec[j] = distances_[j]; // distances with simulated sensor noise
      //vec[j] = distances_gt_[j]; // ground-truth only for debugging
    dat.data = vec;
    if(distances_pub_now)
        distances_pub_->publish(dat);

    // publish positions message (ground-truth only for debugging)
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
        elev[i] = dronestate_[i].elevation_;
    el.data = elev;
    if(elevation_pub_now)
        elevation_pub_->publish(el);

/*
    // publish distances to beacons message (old version, this contains all distances for every drone to every beacon)
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
        bec[i*beaconCount_ + j] = dronestate_[i].beacon_distances_[j]; // distances with simulated sensor noise
      //bec[i*beaconCount_ + j] = dronestate_[i].beacon_distances_gt_[j]; // ground-truth only for debugging
    be.data = bec;
    if(distances_pub_now)
        beacons_pub_->publish(be);
*/
    // publish distances to beacons message (new version, this only contains the distance from the own drone to all beacons)
    std_msgs::Float32MultiArray be;
    be.layout.dim.push_back(std_msgs::MultiArrayDimension());
    be.layout.dim[0].label = "to_beacon";
    be.layout.dim[0].size = beaconCount_;
    be.layout.dim[0].stride = beaconCount_;
    be.layout.data_offset = 0;
    std::vector<float> bec(beaconCount_, 0);
      for (size_t j = 0; j < beaconCount_; j++)
        bec[j] = beacon_distances_[j]; // distances with simulated sensor noise
      //bec[j] = beacon_distances_gt_[j]; // ground-truth only for debugging
    be.data = bec;
    if(distances_pub_now)
        beacons_pub_->publish(be);

    if(dataStoring_active_) // save data for log files
    {
        tempDistance << "\n";
        listDistance_.push_back(tempDistance.str());
        tempMetrics << "\n";
        listMetrics_.push_back(tempMetrics.str());
        tempState << "\n";
        listState_.push_back(tempState.str());
        tempCentroid << "\n";
        listCentroid_.push_back(tempCentroid.str());
    }
}

void DroneStateWithTime::EnableCallback(const std_msgs::Int32ConstPtr& enable_msg) {
  ROS_INFO("DroneStateWithTime (%d) got enable message: %d", droneNumber_, enable_msg->data);

  enable_swarm_ = enable_msg->data;
  parentPtr_->enable_swarm_ = enable_msg->data;

  if(droneNumber_ == 0 && enable_swarm_ == 0) // call this function only once, per callback (only for drone 0)
    parentPtr_->RecalcTargetSpeed(ros::Time::now());
}

void DistanceMeasurementSim::RecalcTargetSpeed(ros::Time timeStamp){
  Vector3f diff_a = old_swarm_center_gt_ - old_beacon0_gt_;
  Vector3f diff_b = swarm_center_gt_ - old_beacon0_gt_;
  float diff_a_norm = diff_a.norm();
  float diff_b_norm = diff_b.norm();
  float duration = timeStamp.toSec() - old_timeStamp_;

  float avg_speed = (diff_a_norm - diff_b_norm) / duration;

//  if(old_beacon0_gt_.norm() < 100 && beacon_gt_[0].norm() < 100 && timeStamp.sec > 3)
  if(enable_swarm_ & SWARM_PHASE_ESTABLISHED && timeStamp.sec > 30)
  {
    ROS_INFO("----------------------------------------------------");
    ROS_INFO("DroneStateWithTime RecalcTargetSpeed.");
    ROS_INFO("old_beacon0_gt_        %s", VectorToString(old_beacon0_gt_).c_str());
    ROS_INFO("beacon_gt_[0]          %s", VectorToString(beacon_gt_[0]).c_str());
    ROS_INFO("old_swarm_center_gt_   %s", VectorToString(old_swarm_center_gt_).c_str());
    ROS_INFO("swarm_center_gt_       %s", VectorToString(swarm_center_gt_).c_str());
    ROS_INFO("diff_a_norm:           %f", diff_a_norm);
    ROS_INFO("diff_b_norm:           %f", diff_b_norm);
    ROS_INFO("duration:              %f", duration);
    ROS_INFO("avg_speed:             %f", avg_speed);
/*
    ROS_INFO("swarm_center_gt_:     %s", VectorToString(swarm_center_gt_).c_str());
    ROS_INFO("beacon_gt_[0]:        %s", VectorToString(beacon_gt_[0]).c_str());
    ROS_INFO("beacon0_to_center_gt: %f", beacon0_to_center_gt.norm());
    */
    ROS_INFO("----------------------------------------------------");

    std::stringstream tempTarget;
    tempTarget.precision(24);
    tempTarget << timeStamp.sec << "," << timeStamp.nsec << ",";

    tempTarget << diff_a_norm << "," << diff_b_norm << "," << duration << "," << avg_speed << ",";

    tempTarget << "\n";
    listTarget_.push_back(tempTarget.str());

  }

  old_timeStamp_ = timeStamp.toSec();
  old_swarm_center_gt_ = swarm_center_gt_;
  old_beacon0_gt_ = beacon_gt_[0];
}

//The callback saves data into csv files
void DistanceMeasurementSim::CallbackSaveData(const ros::TimerEvent& event){
  ROS_INFO("DistanceMeasurementSim CallbackSavaData.");
  for (size_t i = 0; i < droneCount_; i++)
    dronestate[i].FileSaveData();
    this->FileSaveData();
}

void DistanceMeasurementSim::SaveLogCallback(const std_msgs::Int32ConstPtr& enable_msg){
  ROS_INFO("DistanceMeasurementSim SaveLogCallback.");
  for (size_t i = 0; i < droneCount_; i++)
    dronestate[i].FileSaveData();
  this->FileSaveData();
}

void DistanceMeasurementSim::FileSaveData(void){
      std::ofstream fileTarget;

      ROS_INFO("DistanceMeasurementSim FileSaveData.");

      if(mkdir("/tmp/log_output/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
          if(errno != EEXIST)
             ROS_ERROR("Cannot create directory /tmp/log_output/");

      fileTarget.open(std::string("/tmp/log_output/DistSimTarget.csv"), std::ios_base::trunc);

      // Saving centroid log data in a file
      for (unsigned n=0; n < listTarget_.size(); ++n) {
          fileTarget << listTarget_.at( n );
      }

      // Closing all opened files
      fileTarget.close();
}

void DroneStateWithTime::FileSaveData(void){

      if(!dataStoring_active_){
         return;
      }

      std::ofstream fileDistance;
      std::ofstream fileMetrics;
      std::ofstream fileState;
      std::ofstream fileCentroid;

      ROS_INFO("DroneStateWithTime FileSaveData. droneNumber: %d", droneNumber_);

      if(mkdir("/tmp/log_output/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
          if(errno != EEXIST)
             ROS_ERROR("Cannot create directory /tmp/log_output/");

      if(droneNumber_ < 10)
      {
          fileDistance.open(std::string("/tmp/log_output/DistSimDistance0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
          fileMetrics.open(std::string("/tmp/log_output/DistSimMetrics0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
          fileState.open(std::string("/tmp/log_output/DistSimState0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
          fileCentroid.open(std::string("/tmp/log_output/DistSimCentroid0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      }
      else
      {
          fileDistance.open(std::string("/tmp/log_output/DistSimDistance") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
          fileMetrics.open(std::string("/tmp/log_output/DistSimMetrics") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
          fileState.open(std::string("/tmp/log_output/DistSimState") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
          fileCentroid.open(std::string("/tmp/log_output/DistSimCentroid") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      }

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
      // Saving centroid log data in a file
      for (unsigned n=0; n < listCentroid_.size(); ++n) {
          fileCentroid << listCentroid_.at( n );
      }

      // Closing all opened files
      fileDistance.close();
      fileMetrics.close();
      fileState.close();
      fileCentroid.close();

      // To have a one shot storing
      // dataStoring_active_ = false;
}

void DroneStateWithTime::SetId(DistanceMeasurementSim* parentPtr, int droneNumber, int droneCount, int beaconCount, float position_noise, float elevation_noise, int noise_color, float distance_max_rate, float elevation_max_rate, DroneStateWithTime* dronestate, ros::Publisher* distances_pub, ros::Publisher* positions_pub, ros::Publisher* elevation_pub, ros::Publisher* beacons_pub, bool dataStoring_active, Vector3f* beacon_gt, Vector3f* beacon_air, Vector3f* swarm_center_gt)
{
    parentPtr_ = parentPtr;

    droneNumber_ = droneNumber;
    droneCount_ = droneCount;
    beaconCount_ = beaconCount;

    distance_noise_ = position_noise;
    elevation_noise_ = elevation_noise;
    noise_color_ = noise_color;
    distance_max_rate_ = distance_max_rate;
    elevation_max_rate_ = elevation_max_rate;

    dataStoring_active_ = dataStoring_active;

    dronestate_ = dronestate;
    distances_pub_ = distances_pub;
    positions_pub_ = positions_pub;
    elevation_pub_ = elevation_pub;
    beacons_pub_ = beacons_pub;

    beacon_gt_ = beacon_gt;
    beacon_air_ = beacon_air;
    swarm_center_gt_ = swarm_center_gt;

    enable_swarm_ = 0; // initial value

    for (size_t i = 0; i < N_DRONES_MAX; i++) // iterate over all quadcopters
      distances_iir_[i] = 0;
    for (size_t i = 0; i < N_BEACONS_MAX; i++) // iterate over all beacons
      beacon_distances_iir_[i] = 0;
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "distance_measurement_sim_node");

    ros::NodeHandle nh2;

    rotors_control::DistanceMeasurementSim distance_measurement_sim_node;

    ros::spin();

    return 0;
}
