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

    ROS_INFO_ONCE("DistancesCallback got distances message.");

    for (size_t i = 0; i < droneCount_; i++)
    {
        odometry_estimate_[i].position[0] = 0;
        odometry_estimate_[i].position[1] = 0;
        odometry_estimate_[i].position[2] = 0;
    }

    for (size_t i = 0; i < droneCount_; i++)
    {
        for (size_t j = 0; j < droneCount_; j++)
        {
            distances_[i][j] = distances_msg.data[i*droneCount_ + j];
            ROS_INFO_ONCE("DistancesCallback drone#%d -> drone#%d: distance=%f.", i, j, distances_[i][j]);
        }
    }

    if(droneNumber_ == 2 || true)
    {
        FindBestTriangle(distances_, best_triangle_);
        ROS_INFO_ONCE("DistancesCallback (%d) best_triangle_ = %d / %d / %d / %d", droneNumber_, best_triangle_[0], best_triangle_[1], best_triangle_[2], best_triangle_[3]);
        InferPositions(distances_, best_triangle_, odometry_estimate_);
        CheckDistances(distances_, odometry_estimate_);
    }

    if(droneNumber_ == 0) // move red indicator sphere for testing purposes
    {
        for (size_t i = 0; i < droneCount_; i++)
        {
            geometry_msgs::Point pr2_position;
            //            pr2_position.x = odometry_gt_.position[0] + sin(visual_cnt_)* 0.1;
            //            pr2_position.y = odometry_gt_.position[1] + cos(visual_cnt_)* 0.1;
            //            pr2_position.z = odometry_gt_.position[2];
            pr2_position.x = odometry_gt_.position[0] + odometry_estimate_[i].position[0];
            pr2_position.y = odometry_gt_.position[1] + odometry_estimate_[i].position[1];
            pr2_position.z = odometry_gt_.position[2] + odometry_estimate_[i].position[2];
            geometry_msgs::Quaternion pr2_orientation;
            pr2_orientation.x = 0.0;
            pr2_orientation.y = 0.0;
            pr2_orientation.z = 0.0;
            pr2_orientation.w = 1.0;
            geometry_msgs::Pose pr2_pose;
            pr2_pose.position = pr2_position;
            pr2_pose.orientation = pr2_orientation;
            gazebo_msgs::ModelState pr2_modelstate;
            pr2_modelstate.model_name = (std::string) "marker_crazyflie2_" + std::to_string(i);
            pr2_modelstate.pose = pr2_pose;
            gazebo_msgs::SetModelState srv;
            srv.request.model_state = pr2_modelstate;
            if(!gazebo_client_.call(srv))
                ROS_ERROR("Failed to move marker! Error msg:%s",srv.response.status_message.c_str());
            visual_cnt_ += 2*3.14159265358979323846 / 200;
        }
    }
}

void SwarmStateEstimator::FindBestTriangle(float (*distances)[N_DRONES_MAX], int* triangle) {
    float diff_min = FLT_MAX;
    int diff_min_ind_i = droneNumber_;
    int diff_min_ind_a = -1;
    int diff_min_ind_b = -1;
    int z_max_ind = -1;

    for (size_t a = 0; a < droneCount_; a++)
    {
        if(a == droneNumber_)
            continue;
        for (size_t b = 0; b < a; b++)
        {
            if(b == droneNumber_)
                continue;
            float diff = (
                            pow(distances[droneNumber_][a] - distances[droneNumber_][b], 2) +
                            pow(distances[droneNumber_][a] - distances[a][b], 2) +
                            pow(distances[droneNumber_][b] - distances[a][b], 2)
                          ) / pow(distances[droneNumber_][a] + distances[droneNumber_][b] + distances[a][b], 2);
            if (diff < diff_min)
            {
                diff_min = diff;
                diff_min_ind_i = droneNumber_;
                diff_min_ind_a = a;
                diff_min_ind_b = b;
            }
            //_, _, z_max_ind = infer_positions_helper(nquads, distances, [i, a, b])
        }
    }
    triangle[0] = diff_min_ind_i;
    triangle[1] = diff_min_ind_a;
    triangle[2] = diff_min_ind_b;
    triangle[3] = z_max_ind; // TODO: determine actual best z-index drone
}

void SwarmStateEstimator::InferPositions(float (*distances)[N_DRONES_MAX], int* triangle, EigenOdometry* positions) {
    int a = triangle[1];
    int b = triangle[2];
    int c = triangle[3];
    // int c = -1 # this saves the index of the fourth quadcopter to tie-break the roots (it is optional, if not given, the first quadcopter not in the triangle is used)

    float positions_quality[N_DRONES_MAX];

    positions[triangle[0]].position[0] = 0; // relative x-coordinate
    positions[triangle[0]].position[1] = 0; // relative y-coordinate
    positions[triangle[0]].position[2] = 0; // relative z-coordinate
    positions_quality[triangle[0]] = 3; // high quality of point (part of triangle)

    ROS_INFO_ONCE("InferPositions (%d) tr0_:%d at x:%f y:%f z:%f", droneNumber_, triangle[0], positions[triangle[0]].position[0], positions[triangle[0]].position[1], positions[triangle[0]].position[2]);

    positions[triangle[1]].position[0] = distances[triangle[0]][triangle[1]]; // relative x-coordinate
    positions[triangle[1]].position[1] = 0; // relative y-coordinate
    positions[triangle[1]].position[2] = 0; // relative z-coordinate
    positions_quality[triangle[1]] = 3; // high quality of point (part of triangle)

    ROS_INFO_ONCE("InferPositions (%d) tr1_:%d at x:%f y:%f z:%f", droneNumber_, triangle[1], positions[triangle[1]].position[0], positions[triangle[1]].position[1], positions[triangle[1]].position[2]);

    // https://en.wikipedia.org/wiki/True-range_multilateration#Two_Cartesian_dimensions,_two_measured_slant_ranges_(Trilateration)
    positions[triangle[2]].position[0] = ( // relative x-coordinate
                                            pow(distances[triangle[0]][triangle[2]], 2) -
                                            pow(distances[triangle[1]][triangle[2]], 2) +
                                            pow(distances[triangle[0]][triangle[1]], 2)
                                          ) / (2*distances[triangle[0]][triangle[1]]);
    float t1 = (pow(distances[triangle[0]][triangle[2]], 2) - pow(positions[triangle[2]].position[0], 2));
    if (t1 < 0) // should not be the case, but it can happen because noisy measurements: sum of two edges of the triangle is shorter than the third edge
        positions[triangle[2]].position[1] = 0.001; // relative y-coordinate
    else
        positions[triangle[2]].position[1] = sqrt(pow(distances[triangle[0]][triangle[2]], 2) - pow(positions[triangle[2]].position[0], 2)); // relative y-coordinate
    positions[triangle[2]].position[2] = 0; // relative z-coordinate
    positions_quality[triangle[2]] = 3; // high quality of point (part of triangle)
    ROS_INFO_ONCE("InferPositions (%d) tr2_:%d at x:%f y:%f z:%f", droneNumber_, triangle[2], positions[triangle[2]].position[0], positions[triangle[2]].position[1], positions[triangle[2]].position[2]);

    float z_max = 0;
    int z_max_ind = -1;

    std::list<int> quadrange;
    if(triangle[3] > 0)
        quadrange.push_back(triangle[3]);
    for (size_t i = 0; i < droneCount_; i++)
        if(i != triangle[0] && i != triangle[1] && i != triangle[2])
            quadrange.push_back(i);

    // https://en.wikipedia.org/wiki/True-range_multilateration#Three_Cartesian_dimensions,_three_measured_slant_ranges
    for(auto num = quadrange.begin(); num != quadrange.end(); num++)
    {
        int i = *num;

        positions[i].position[0] = ( // relative x-coordinate
                                      pow(distances[triangle[0]][i], 2) -
                                      pow(distances[triangle[1]][i], 2) +
                                      pow(distances[triangle[0]][triangle[1]], 2)
                                    ) / (2 * distances[triangle[0]][triangle[1]]);
        positions[i].position[1] = (
                                      pow(distances[triangle[0]][i], 2) -
                                      pow(distances[triangle[2]][i], 2) +
                                      pow(distances[triangle[0]][triangle[2]], 2) -
                                      2*positions[triangle[2]].position[0]*positions[i].position[0]
                                    ) / (2*positions[triangle[2]].position[1]);
        float t1 = (pow(distances[triangle[0]][i], 2) - pow(positions[i].position[0], 2) - pow(positions[i].position[1], 2));
        if(t1 > 0)
            positions[i].position[2] = sqrt(t1);
        else
            positions[i].position[2] = 0;

        if(abs(positions[i].position[2]) > z_max)
        {
            z_max = abs(positions[i].position[2]);
            z_max_ind = i;
        }
        positions_quality[i] = positions[i].position[2]; // quality of point proportional to z elevation
        ROS_INFO_ONCE("InferPositions (%d) loop:%d at x:%f y:%f z:%f", droneNumber_, i, positions[i].position[0], positions[i].position[1], positions[i].position[2]);

        // use fourth drone (tie-breaker) to check if relative z-coordinate may be flipped
        if(c >= 0)
        {
            EigenOdometry pos_neg = positions[i];
            pos_neg.position[2] = 0 - positions[i].position[2];
            float dist_pos = Distance(&positions[c], &positions[i]);
            float dist_neg = Distance(&positions[c], &pos_neg);
            if(abs(dist_neg - distances[c][i]) < abs(dist_pos - distances[c][i]))
                positions[i].position[2] = pos_neg.position[2];
        }
        if(c < 0)
            c = i;
    }

    for (size_t i = 0; i < droneCount_; i++)
        ROS_INFO_ONCE("InferPositions (%d) drone:%d at x:%f y:%f z:%f", droneNumber_, i, positions[i].position[0], positions[i].position[1], positions[i].position[2]);

  //  return positions, positions_quality, z_max_ind
}

void SwarmStateEstimator::CheckDistances(float (*distances)[N_DRONES_MAX], EigenOdometry* positions) {
    float eps = 0.01;

    for (size_t i = 0; i < droneCount_; i++)
        for (size_t j = 0; j < droneCount_; j++)
        {
            float dist = Distance(&positions[i], &positions[j]);
            if(abs(distances[i][j] - dist) > eps)
                ROS_INFO("CheckDistances (%d) distance missmatch at i:%d j:%d dist:%f positions:%f", droneNumber_, i, j, distances[i][j], dist);
            else
                ROS_INFO_ONCE("CheckDistances (%d) distance ok at        i:%d j:%d dist:%f positions:%f", droneNumber_, i, j, distances[i][j], dist);
        }
}

float Distance(EigenOdometry* a, EigenOdometry* b) {
    return sqrt(pow(a->position[0] - b->position[0], 2) +
                pow(a->position[1] - b->position[1], 2) +
                pow(a->position[2] - b->position[2], 2));
}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "swarm_state_estimator_node");

    ros::NodeHandle nh2;

    rotors_control::SwarmStateEstimator swarm_state_estimator_node;

    ros::spin();

    return 0;
}
