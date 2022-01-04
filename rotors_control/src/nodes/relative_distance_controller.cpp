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

#include "relative_distance_controller.h"

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

RelativeDistanceController::RelativeDistanceController() {

    ROS_INFO_ONCE("Started RelativeDistanceController");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

    history_cnt_ = 0;
    odometry_gt_history1_.position[0] = 0;
    odometry_gt_history1_.position[1] = 0;
    odometry_gt_history1_.position[2] = 0;
    odometry_gt_history2_.position[0] = 0;
    odometry_gt_history2_.position[1] = 0;
    odometry_gt_history2_.position[2] = 0;
    for (size_t i = 0; i < N_VECTORS_MAX; i++)
    {
        unit_vectors_[i][0] = 0;
        unit_vectors_[i][1] = 0;
        unit_vectors_[i][2] = 0;
    }
    transform_ok_ = 0;

    // Topics subscribe
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &RelativeDistanceController::OdometryCallback, this);
    distances_sub_ = nh.subscribe("/drone_distances", 1, &RelativeDistanceController::DistancesCallback, this);

    // To publish the current pose for gazebo visual
    visual_cnt_ = 0;
    gazebo_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

RelativeDistanceController::~RelativeDistanceController(){}

void RelativeDistanceController::InitializeParams() {
    ros::NodeHandle pnh("~");

    ROS_INFO_ONCE("[RelativeDistanceController] InitializeParams");

    n_move_max_ = 1;
    spc_cohesion_weight_ = 1.0;
    spc_separation_weight_ = 1.0;
    eps_move_ = 0.1;
    neighbourhood_distance_ = 999; // global neighbourhood

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

    ros::NodeHandle nh;
    //timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &RelativeDistanceController::CallbackSaveData, this, false, true);
}

void RelativeDistanceController::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("RelativeDistanceController got first odometry message.");

    // received message drone position information, store ground-truth pose
    odometry_gt_.timeStampSec = odometry_msg->header.stamp.sec;
    odometry_gt_.timeStampNsec = odometry_msg->header.stamp.nsec;
    odometry_gt_.position[0] = odometry_msg->pose.pose.position.x;
    odometry_gt_.position[1] = odometry_msg->pose.pose.position.y;
    odometry_gt_.position[2] = odometry_msg->pose.pose.position.z;
    ROS_INFO_ONCE("RelativeDistanceController got odometry message: x=%f y=%f z=%f (%d)", odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2], droneNumber_);

    Vector3f movement;
    movement[0] = odometry_gt_.position[0] - odometry_gt_history1_.position[0];
    movement[1] = odometry_gt_.position[1] - odometry_gt_history1_.position[1];
    movement[2] = odometry_gt_.position[2] - odometry_gt_history1_.position[2];
    float movement_norm = movement.norm();

    history_cnt_ ++;
//    if(history_cnt_ > 10 || true)
    if(movement_norm > 0.15) // last move was at least 15cm, TODO: find better threshold?
    {
        ROS_INFO_ONCE("OdometryCallback (%d) movement over threshold: %s", droneNumber_, VectorToString(movement).c_str());

        movement = movement / movement_norm; // divide by norm to get unit vector
        // find unit vector closest to the current movement vector: having the largest dot product
        float max_dot_product = 0;
        size_t index_dot_product = 0;
        float same_dot[N_VECTORS_MAX]; // calculate similarity of unit_vectors to each other
        same_dot[0] = abs(unit_vectors_[0].dot(unit_vectors_[1])) + abs(unit_vectors_[0].dot(unit_vectors_[2]));
        same_dot[1] = abs(unit_vectors_[1].dot(unit_vectors_[0])) + abs(unit_vectors_[1].dot(unit_vectors_[2]));
        same_dot[2] = abs(unit_vectors_[2].dot(unit_vectors_[1])) + abs(unit_vectors_[2].dot(unit_vectors_[0]));
        for (size_t i = 0; i < N_VECTORS_MAX; i++)
        {
            if(unit_vectors_[i].norm() < 0.1) // this unit vector is not yet initialized, use it
            {
                ROS_INFO_ONCE("OdometryCallback (%d) this unit vector is not yet initialized, use it: %d", droneNumber_, (int)i);
                index_dot_product = i;
                break;
            }
            float dot_product = abs(unit_vectors_[i].dot(movement)); // TODO: find factor for penalizing same similarity
            //ROS_INFO("%s * %s (%d) = %f (%f)", VectorToString(unit_vectors_[i]).c_str(), VectorToString(movement).c_str(), (int)i, dot_product, same_dot[i]);

            if(dot_product*same_dot[i] > max_dot_product)
            {
                index_dot_product = i;
                max_dot_product = dot_product*same_dot[i];
            }
        }

        ROS_INFO_ONCE("OdometryCallback (%d) replace unit vector: %d", droneNumber_, (int)index_dot_product);
        unit_vectors_[index_dot_product] = movement;
        for (size_t i = 0; i < droneCount_; i++)
            distances_differences_[index_dot_product][i] = (distances_[droneNumber_][i] - distances_history1_[i]) / movement_norm;

        ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_0: %s", droneNumber_, VectorToString(unit_vectors_[0]).c_str());
        ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_1: %s", droneNumber_, VectorToString(unit_vectors_[1]).c_str());
        ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_2: %s\n", droneNumber_, VectorToString(unit_vectors_[2]).c_str());

        Vector3f span_vectors;
        span_vectors[0] = fmax(unit_vectors_[0][0], fmax(unit_vectors_[1][0], unit_vectors_[2][0])) - fmin(unit_vectors_[0][0], fmin(unit_vectors_[1][0], unit_vectors_[2][0]));
        span_vectors[1] = fmax(unit_vectors_[0][1], fmax(unit_vectors_[1][1], unit_vectors_[2][1])) - fmin(unit_vectors_[0][1], fmin(unit_vectors_[1][1], unit_vectors_[2][1]));
        span_vectors[2] = fmax(unit_vectors_[0][2], fmax(unit_vectors_[1][2], unit_vectors_[2][2])) - fmin(unit_vectors_[0][2], fmin(unit_vectors_[1][2], unit_vectors_[2][2]));
        ROS_INFO("OdometryCallback (%d) span_vectors: %s", droneNumber_, VectorToString(span_vectors).c_str());

        same_dot[0] = abs(unit_vectors_[0].dot(unit_vectors_[1])) + abs(unit_vectors_[0].dot(unit_vectors_[2]));
        same_dot[1] = abs(unit_vectors_[1].dot(unit_vectors_[0])) + abs(unit_vectors_[1].dot(unit_vectors_[2]));
        same_dot[2] = abs(unit_vectors_[2].dot(unit_vectors_[1])) + abs(unit_vectors_[2].dot(unit_vectors_[0]));
        if(span_vectors[0] > 0.01 && span_vectors[1] > 0.01 && span_vectors[2] > 0.01 &&
           same_dot[0] < (2-0.02) && same_dot[1] < (2-0.02) && same_dot[2] < (2-0.02))
        {
            // calculate basis transform Matrix
            Matrix3f transform_tmp;
            transform_tmp(0,0) = unit_vectors_[0][0];
            transform_tmp(1,0) = unit_vectors_[0][1];
            transform_tmp(2,0) = unit_vectors_[0][2];
            transform_tmp(0,1) = unit_vectors_[1][0];
            transform_tmp(1,1) = unit_vectors_[1][1];
            transform_tmp(2,1) = unit_vectors_[1][2];
            transform_tmp(0,2) = unit_vectors_[2][0];
            transform_tmp(1,2) = unit_vectors_[2][1];
            transform_tmp(2,2) = unit_vectors_[2][2];
            transform_vectors_ = transform_tmp.inverse();
            ROS_INFO_ONCE("OdometryCallback (%d) transform_vectors_: %s", droneNumber_, MatrixToString(transform_vectors_).c_str());
            transform_ok_ = 1;
        }
        else
        {
            ROS_INFO("OdometryCallback (%d) Bad unit vectors, cannot calculate transform_vectors.", droneNumber_);
            transform_ok_ = 0;
        }

        // save new history point
        for (size_t i = 0; i < droneCount_; i++)
        {
            distances_history2_[i] = distances_history1_[i];
            distances_history1_[i] = distances_[droneNumber_][i];
        }
        odometry_gt_history2_ = odometry_gt_history1_;
        odometry_gt_history1_ = odometry_gt_;
        history_cnt_ = 0;
    }



    // ################################################################################
    if(enable_swarm_ == SWARM_DISABLED) // set target point if not in swarm mode
    {
/*        ROS_INFO_ONCE("SwarmController %d swarm disabled x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
        set_point.pose.position.x = target_swarm_.position_W[0];
        set_point.pose.position.y = target_swarm_.position_W[1];
        set_point.pose.position.z = target_swarm_.position_W[2];*/
    }
    else if(enable_swarm_ == SWARM_LANDING) // set keep target point and set small z if in landing mode
    {
/*        ROS_INFO_ONCE("SwarmController %d landing x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
        set_point.pose.position.x = odometry_.position[0];
        set_point.pose.position.y = odometry_.position[1];
        set_point.pose.position.z = max(0.0, min(odometry_.position[2] - 0.05, 0.1));*/
    }
    else if(enable_swarm_ & SWARM_DECLARATIVE_DISTANCES && transform_ok_ && droneNumber_ == 0)
    {
        ROS_INFO_ONCE("RelativeDistanceController starting swarm mode (SWARM_DECLARATIVE_DISTANCES)");

        int min_xi = 0;
        int min_yi = 0;
        int min_zi = 0;

        // calculate neighbourhood independently from potential position
        int neighbourhood_cnt = 0;
        bool neighbourhood_bool[N_DRONES_MAX];
        for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
        {
            if(distances_[droneNumber_][i] < neighbourhood_distance_ && i != droneNumber_)
            {
                neighbourhood_cnt ++;
                neighbourhood_bool[i] = true;
            }
            else
                neighbourhood_bool[i] = false;
        }

        float min_sum = FLT_MAX;
        for(int xi = 0-n_move_max_; xi <= n_move_max_; xi ++) // iterate over all possible next actions in x-, y- and z-dimension
        {
            for(int yi = 0-n_move_max_; yi <= n_move_max_; yi ++)
            {
                for(int zi = 0-n_move_max_; zi <= n_move_max_; zi ++)
                {
                    Vector3f potential_movement = {(float)xi * eps_move_, (float)yi * eps_move_, (float)zi * eps_move_};
                    Vector3f potential_movement_transformed = transform_vectors_ * potential_movement;

                    float cohesion_sum = 0;
                    float separation_sum = 0;
                    float total_sum = 0;

                    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
                    {
                        float dist = distances_[droneNumber_][i] +
                                     distances_differences_[0][i] * potential_movement_transformed[0] +
                                     distances_differences_[1][i] * potential_movement_transformed[1] +
                                     distances_differences_[2][i] * potential_movement_transformed[2];

                        if(i == droneNumber_) // skip for own quadcopter
                          continue;

                        cohesion_sum += dist*dist;
                        separation_sum += 1.0/(dist*dist);
                    }

                    // coehesion term
                    total_sum = spc_cohesion_weight_ * cohesion_sum / ((float)neighbourhood_cnt);
                    // separation term
                    total_sum += spc_separation_weight_ * separation_sum / ((float)neighbourhood_cnt);

                    if(total_sum < min_sum)
                    {
                        min_sum = total_sum;
                        min_xi = xi;
                        min_yi = yi;
                        min_zi = zi;
                    }
                }
            }
        }
        ROS_INFO("RelativeDistanceController %d swarm direction xi=%d yi=%d zi=%d tsum=%f", droneNumber_, min_xi, min_yi, min_zi, min_sum);
/*        set_point.pose.position.x = odometry_.position[0] + (float)min_xi * eps_move_;
        set_point.pose.position.y = odometry_.position[1] + (float)min_xi * eps_move_;
        set_point.pose.position.z = odometry_.position[2] + (float)min_xi * eps_move_;*/

  }

}

void RelativeDistanceController::DistancesCallback(const std_msgs::Float32MultiArray& distances_msg) {
    ROS_INFO_ONCE("DistancesCallback got distances message.");

    for (size_t i = 0; i < droneCount_; i++)
    {
        for (size_t j = 0; j < droneCount_; j++)
        {
            distances_[i][j] = distances_msg.data[i*droneCount_ + j];
            ROS_INFO_ONCE("DistancesCallback drone#%d -> drone#%d: distance=%f.", (int)i, (int)j, distances_[i][j]);
        }
    }
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "relative_distance_controller_node");

    ros::NodeHandle nh2;

    rotors_control::RelativeDistanceController relative_distance_controller_node;

    ros::spin();

    return 0;
}
