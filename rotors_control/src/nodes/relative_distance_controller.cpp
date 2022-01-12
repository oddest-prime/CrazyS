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

    generator_.seed(droneNumber_); // make sure random numbers are different for each drone

    history_cnt_ = 0;
    random_direction_[0] = 0;
    random_direction_[1] = 0;
    random_direction_[2] = 0;
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
        unit_vectors_age_[i] = 0;
    }
    transform_ok_ = 0;

    // Topics subscribe
    // Topics subscribe
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, &RelativeDistanceController::MultiDofJointTrajectoryCallback, this);
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &RelativeDistanceController::OdometryCallback, this);
    enable_sub_ = nh.subscribe("enable", 1, &RelativeDistanceController::EnableCallback, this);
    distances_sub_ = nh.subscribe("/drone_distances", 1, &RelativeDistanceController::DistancesCallback, this);

    // To publish the set-point
    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("set_point", 1);

    // To publish the current pose for gazebo visual
    visual_cnt_ = 0;
    gazebo_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

RelativeDistanceController::~RelativeDistanceController(){}

void RelativeDistanceController::InitializeParams() {
    ros::NodeHandle pnh("~");

    ROS_INFO_ONCE("[RelativeDistanceController] InitializeParams");

    GetRosParameter(pnh, "swarm/neighbourhood_distance", (float)99, &neighbourhood_distance_);
    GetRosParameter(pnh, "dist/eps_move", (float)0.2, &eps_move_);
    GetRosParameter(pnh, "dist/n_move_max", (int)2, &n_move_max_);
    GetRosParameter(pnh, "dist/spc_cohesion_weight", (float)1.0, &spc_cohesion_weight_);
    GetRosParameter(pnh, "dist/spc_separation_weight", (float)1.0, &spc_separation_weight_);

    ROS_INFO_ONCE("[RelativeDistanceController] GetRosParameter values:");
    ROS_INFO_ONCE("  swarm/neighbourhood_distance=%f", neighbourhood_distance_);
    ROS_INFO_ONCE("  dist/eps_move=%f", eps_move_);
    ROS_INFO_ONCE("  dist/n_move_max=%d", n_move_max_);
    ROS_INFO_ONCE("  dist/spc_cohesion_weight=%f", spc_cohesion_weight_);
    ROS_INFO_ONCE("  dist/spc_separation_weight=%f", spc_separation_weight_);

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

void RelativeDistanceController::EnableCallback(const std_msgs::Int32ConstPtr& enable_msg) {
    ROS_INFO("RelativeDistanceController got enable message: %d", enable_msg->data);

    enable_swarm_ = enable_msg->data;
}

void RelativeDistanceController::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  ROS_INFO_ONCE("SwarmController got MultiDOFJointTrajectory message: x=%f, y=%f, z=%f, yaw=%f", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2], eigen_reference.getYaw());

  // set target point for hovering or swarm target if in swarm mode
  target_swarm_ = eigen_reference;
}

void RelativeDistanceController::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    ROS_INFO_ONCE("RelativeDistanceController got first odometry message.");

    // received message drone position information, store ground-truth pose
    odometry_gt_.timeStampSec = odometry_msg->header.stamp.sec;
    odometry_gt_.timeStampNsec = odometry_msg->header.stamp.nsec;
    odometry_gt_.position[0] = odometry_msg->pose.pose.position.x;
    odometry_gt_.position[1] = odometry_msg->pose.pose.position.y;
    odometry_gt_.position[2] = odometry_msg->pose.pose.position.z;
    ROS_INFO_ONCE("RelativeDistanceController (%d) odometry: x=%f y=%f z=%f", droneNumber_, odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2]);

    // calculate movement vector based on current ground-truth position and history (TODO: use accelerometer data)
    Vector3f movement;
    movement[0] = odometry_gt_.position[0] - odometry_gt_history1_.position[0];
    movement[1] = odometry_gt_.position[1] - odometry_gt_history1_.position[1];
    movement[2] = odometry_gt_.position[2] - odometry_gt_history1_.position[2];
    float movement_norm = movement.norm();

    // setpoint for message to be sent to low-level controller
    geometry_msgs::PoseStamped set_point;
    set_point.header.stamp = odometry_msg->header.stamp;

    history_cnt_ ++;
    unit_vectors_age_[0] ++;
    unit_vectors_age_[1] ++;
    unit_vectors_age_[2] ++;

    if(unit_vectors_age_[0] > 1500) // delete old unit vectors
    {
        unit_vectors_[0] = unit_vectors_[2];
        unit_vectors_age_[0] = unit_vectors_age_[2];
        unit_vectors_[2][0] = 0;
        unit_vectors_[2][1] = 0;
        unit_vectors_[2][2] = 0;
        unit_vectors_age_[2] = 0;
        ROS_INFO("OdometryCallback (%d) deleted old unit_vectors_0", droneNumber_);
    }
    else if(unit_vectors_age_[1] > 1500) // delete old unit vectors
    {
        unit_vectors_[1] = unit_vectors_[2];
        unit_vectors_age_[1] = unit_vectors_age_[2];
        unit_vectors_[2][0] = 0;
        unit_vectors_[2][1] = 0;
        unit_vectors_[2][2] = 0;
        unit_vectors_age_[2] = 0;
        ROS_INFO("OdometryCallback (%d) deleted old unit_vectors_1", droneNumber_);
    }
    else if(unit_vectors_age_[2] > 1500) // delete old unit vectors
    {
        unit_vectors_[2][0] = 0;
        unit_vectors_[2][1] = 0;
        unit_vectors_[2][2] = 0;
        unit_vectors_age_[2] = 0;
        ROS_INFO("OdometryCallback (%d) deleted old unit_vectors_2", droneNumber_);
    }

//    if(history_cnt_ > 10 || true)
    if(movement_norm > 0.15) // last move was at least 15cm, TODO: find better threshold?
    {
        ROS_INFO_ONCE("OdometryCallback (%d) movement over threshold: %s", droneNumber_, VectorToString(movement).c_str());

        if(enable_swarm_ != SWARM_DISABLED)
        {
            movement = movement / movement_norm; // divide by norm to get unit vector

            // calculate similarity of unit_vectors to each other
            float same_dot[3];
            same_dot[0] = abs(unit_vectors_[0].dot(unit_vectors_[1])) + abs(unit_vectors_[0].dot(unit_vectors_[2]));
            same_dot[1] = abs(unit_vectors_[1].dot(unit_vectors_[0])) + abs(unit_vectors_[1].dot(unit_vectors_[2]));
            same_dot[2] = abs(unit_vectors_[2].dot(unit_vectors_[1])) + abs(unit_vectors_[2].dot(unit_vectors_[0]));

            // find unit vector closest to the current movement vector: having the largest dot product
            float max_dot_product = 0;
            size_t index_dot_product = 0;
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
            unit_vectors_age_[index_dot_product] = 0;
            for (size_t i = 0; i < droneCount_; i++)
                distances_differences_[index_dot_product][i] = (distances_[droneNumber_][i] - distances_history1_[i]) / movement_norm;

            ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_0 (age:%d): %s", droneNumber_, unit_vectors_age_[0], VectorToString(unit_vectors_[0]).c_str());
            ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_1 (age:%d): %s", droneNumber_, unit_vectors_age_[1], VectorToString(unit_vectors_[1]).c_str());
            ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_2 (age:%d): %s\n", droneNumber_, unit_vectors_age_[2], VectorToString(unit_vectors_[2]).c_str());

            Vector3f span_vectors;
            span_vectors[0] = fmax(unit_vectors_[0][0], fmax(unit_vectors_[1][0], unit_vectors_[2][0])) - fmin(unit_vectors_[0][0], fmin(unit_vectors_[1][0], unit_vectors_[2][0]));
            span_vectors[1] = fmax(unit_vectors_[0][1], fmax(unit_vectors_[1][1], unit_vectors_[2][1])) - fmin(unit_vectors_[0][1], fmin(unit_vectors_[1][1], unit_vectors_[2][1]));
            span_vectors[2] = fmax(unit_vectors_[0][2], fmax(unit_vectors_[1][2], unit_vectors_[2][2])) - fmin(unit_vectors_[0][2], fmin(unit_vectors_[1][2], unit_vectors_[2][2]));
            ROS_INFO_ONCE("OdometryCallback (%d) span_vectors: %s", droneNumber_, VectorToString(span_vectors).c_str());

            if(unit_vectors_[0].norm() > 0.01 &&
               unit_vectors_[1].norm() > 0.01 &&
               unit_vectors_[2].norm() > 0.01 &&
               span_vectors[0] > 0.01 && span_vectors[1] > 0.01 && span_vectors[2] > 0.01 &&
               abs(unit_vectors_[0].dot(unit_vectors_[1])) < 0.9 &&
               abs(unit_vectors_[1].dot(unit_vectors_[2])) < 0.9 &&
               abs(unit_vectors_[2].dot(unit_vectors_[0])) < 0.9)
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
                ROS_INFO_ONCE("OdometryCallback (%d) Bad unit vectors, cannot calculate transform_vectors.", droneNumber_);
                transform_ok_ = 0;
            }
        }

        // save current point as history point
        for (size_t i = 0; i < droneCount_; i++)
        {
            distances_history2_[i] = distances_history1_[i];
            distances_history1_[i] = distances_[droneNumber_][i];
        }
        odometry_gt_history2_ = odometry_gt_history1_;
        odometry_gt_history1_ = odometry_gt_;
        history_cnt_ = 0;
        random_direction_[0] = 0;
        random_direction_[1] = 0;
        random_direction_[2] = 0;
    }


    int exploration_info = 0;

    // ################################################################################
    if(enable_swarm_ == SWARM_DISABLED) // set target point if not in swarm mode
    {
        ROS_INFO_ONCE("RelativeDistanceController %d swarm disabled x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
        set_point.pose.position.x = target_swarm_.position_W[0];
        set_point.pose.position.y = target_swarm_.position_W[1];
        set_point.pose.position.z = target_swarm_.position_W[2];
    }
    else if(enable_swarm_ == SWARM_LANDING) // set keep target point and set small z if in landing mode
    {
/*        ROS_INFO_ONCE("SwarmController %d landing x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
        set_point.pose.position.x = odometry_.position[0];
        set_point.pose.position.y = odometry_.position[1];
        set_point.pose.position.z = max(0.0, min(odometry_.position[2] - 0.05, 0.1));*/
    }
    else if(enable_swarm_ & SWARM_DECLARATIVE_DISTANCES)
    {
        ROS_INFO_ONCE("RelativeDistanceController starting swarm mode (SWARM_DECLARATIVE_DISTANCES)");

        if(!transform_ok_) // need to do exploration
        {
            if(random_direction_.norm() <= 0.02) // need new random value, if value was cleared
            {
                std::normal_distribution<float> dist(0.0, 1); // gaussian random number generator
                random_direction_[0] = dist(generator_);
                random_direction_[1] = dist(generator_);
                random_direction_[2] = dist(generator_);
            }
            exploration_info = 80;
            Vector3f direction = {1, 1, 1};
            if(unit_vectors_[0].norm() <= 0.02) // all unit vectors empty, since they are populated sequentially
            {
                direction = random_direction_; // go to random direction
                exploration_info = 10;
            }
            else if(unit_vectors_[1].norm() <= 0.02) // only vector 0 is not empty
            {
                Vector3f tmp = unit_vectors_[0] + random_direction_;
                direction = unit_vectors_[0].cross(tmp); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                exploration_info = 11;
            }
            else if(unit_vectors_[2].norm() <= 0.02) // vector 0 and 1 are not empty
            {
                if(abs(unit_vectors_[0].dot(unit_vectors_[1])) > 0.99) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors go into the same direction, need some orthogonal move
                    Vector3f tmp = unit_vectors_[1] + random_direction_;
                    direction = unit_vectors_[0].cross(tmp); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 12;
                }
                else
                { // vectors go into different directions, use them to calculate orthogonal move
                    direction = unit_vectors_[0].cross(unit_vectors_[1]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 13;
                }
            }
            else // vector 0, 1 and 2 are not empty
            {
                if(abs(unit_vectors_[0].dot(unit_vectors_[1])) < 0.5) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 0 and 1 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[0].cross(unit_vectors_[1]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 41;
                }
                else if(abs(unit_vectors_[1].dot(unit_vectors_[2])) < 0.5) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 1 and 2 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[1].cross(unit_vectors_[2]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 42;
                }
                else if(abs(unit_vectors_[2].dot(unit_vectors_[0])) < 0.5) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 2 and 0 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[2].cross(unit_vectors_[0]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 43;
                }
                else if(abs(unit_vectors_[0].dot(unit_vectors_[1])) < 0.9) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 0 and 1 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[0].cross(unit_vectors_[1]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 51;
                }
                else if(abs(unit_vectors_[1].dot(unit_vectors_[2])) < 0.9) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 1 and 2 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[1].cross(unit_vectors_[2]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 52;
                }
                else if(abs(unit_vectors_[2].dot(unit_vectors_[0])) < 0.9) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 2 and 0 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[2].cross(unit_vectors_[0]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 53;
                }
                else
                {
                    ROS_INFO("RelativeDistanceController %d unit_vectors_[0].dot(unit_vectors_[1]):%f", droneNumber_, unit_vectors_[0].dot(unit_vectors_[1]));
                    ROS_INFO("RelativeDistanceController %d unit_vectors_[1].dot(unit_vectors_[2]):%f", droneNumber_, unit_vectors_[1].dot(unit_vectors_[2]));
                    ROS_INFO("RelativeDistanceController %d unit_vectors_[2].dot(unit_vectors_[0]):%f", droneNumber_, unit_vectors_[2].dot(unit_vectors_[0]));
                    exploration_info = 91;
                }
            }
            direction = direction / direction.norm(); // calculate unit vector of length 1
            set_point.pose.position.x = odometry_gt_.position[0] + direction[0];
            set_point.pose.position.y = odometry_gt_.position[1] + direction[1];
            set_point.pose.position.z = odometry_gt_.position[2] + direction[2];
            ROS_INFO("RelativeDistanceController %d explore:%d direction:%s", droneNumber_, exploration_info, VectorToString(direction).c_str());
        }
        else // possible to do exploitation
        {
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
            ROS_INFO_ONCE("RelativeDistanceController %d exploitation xi=%d yi=%d zi=%d tsum=%f", droneNumber_, min_xi, min_yi, min_zi, min_sum);
            set_point.pose.position.x = odometry_gt_.position[0] + (float)min_xi * eps_move_;
            set_point.pose.position.y = odometry_gt_.position[1] + (float)min_yi * eps_move_;
            set_point.pose.position.z = odometry_gt_.position[2] + (float)min_zi * eps_move_*1.5; // TODO: proper scaling
        }
    }

    if(enable_swarm_ != SWARM_DISABLED || set_point.pose.position.z > 0.01) // do not enable drone until proper target point received
    {
        setpoint_pub_.publish(set_point);
        ROS_INFO_ONCE("RelativeDistanceController %d set_point x=%f y=%f z=%f", droneNumber_, set_point.pose.position.x, set_point.pose.position.y, set_point.pose.position.z);
    }

    // move gazebo markers
    geometry_msgs::Point pr2_position_red;
    geometry_msgs::Point pr2_position_green;
    pr2_position_red.x = set_point.pose.position.x;
    pr2_position_red.y = set_point.pose.position.y;
    pr2_position_red.z = set_point.pose.position.z;
    if(enable_swarm_ == SWARM_DISABLED) // hovering at fixed position, use both markers
    {
        pr2_position_green = pr2_position_red;
    }
    else if(exploration_info == 0) // exploitation phase, use blue marker
    {
        pr2_position_green = pr2_position_red;
        pr2_position_red.z -= 1000;
    }
    else // exploration phase, use green marker
    {
        pr2_position_green = pr2_position_red;
        pr2_position_green.z -= 1000;
    }

    geometry_msgs::Quaternion pr2_orientation;
    geometry_msgs::Pose pr2_pose;
    gazebo_msgs::ModelState pr2_modelstate;
    gazebo_msgs::SetModelState srv;
    pr2_orientation.x = 0.0;
    pr2_orientation.y = 0.0;
    pr2_orientation.z = 0.0;
    pr2_orientation.w = 1.0;
    pr2_pose.position = pr2_position_red;
    pr2_pose.orientation = pr2_orientation;
    pr2_modelstate.model_name = (std::string) "marker_red_crazyflie2_" + std::to_string(droneNumber_);
    pr2_modelstate.pose = pr2_pose;
    srv.request.model_state = pr2_modelstate;
    if(!gazebo_client_.call(srv))
        ROS_ERROR("Failed to move red marker! Error msg:%s",srv.response.status_message.c_str());
    pr2_pose.position = pr2_position_green;
    pr2_modelstate.model_name = (std::string) "marker_green_crazyflie2_" + std::to_string(droneNumber_);
    pr2_modelstate.pose = pr2_pose;
    srv.request.model_state = pr2_modelstate;
    if(!gazebo_client_.call(srv))
        ROS_ERROR("Failed to move green marker! Error msg:%s",srv.response.status_message.c_str());
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
