/*
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2020 Ria Sonecha, MIT, USA
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
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

#ifndef CRAYZFLIE_2_RELATIVE_DISTANCE_CONTROLLER_H
#define CRAYZFLIE_2_RELATIVE_DISTANCE_CONTROLLER_H

#include <boost/bind.hpp>
#include <stdio.h>
#include <random>

#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>

#include "rotors_control/common.h"
#include "rotors_control/Eigen.h"

#define N_DRONES_MAX  20          /* maximum number of drones */
#define N_VECTORS_MAX  3          /* number of saved unit vectors */

namespace rotors_control {
    using namespace Eigen;

    class RelativeDistanceController{
        public:
            RelativeDistanceController();
            ~RelativeDistanceController();

            void InitializeParams();

        private:
            int droneNumber_;
            int droneCount_;

            int history_cnt_;

            EigenOdometry odometry_gt_; // ground-truth
            EigenOdometry odometry_gt_history1_; // ground-truth at history point
            EigenOdometry odometry_gt_history2_; // ground-truth at history point
            float distances_[N_DRONES_MAX][N_DRONES_MAX]; // received distance measurements
            float distances_history1_[N_DRONES_MAX]; // distance measurements to own drone at history point
            float distances_history2_[N_DRONES_MAX]; // distance measurements to own drone at history point
            float elevation_[N_DRONES_MAX]; // received elevation measurements

            Vector3f unit_vectors_[N_VECTORS_MAX]; // directions of unit vectors
            float distances_differences_[N_VECTORS_MAX][N_DRONES_MAX]; // change in distances for unit vectors

            std::string namespace_;

            ros::NodeHandle n_;

            ros::ServiceClient gazebo_client_;
            float visual_cnt_;

            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void DistancesCallback(const std_msgs::Float32MultiArray& distances_msg);
            void ElevationCallback(const std_msgs::Float32MultiArray& elevation_msg);

            //subscribers
            ros::Subscriber odometry_sub_;
            ros::Subscriber distances_sub_;
            ros::Subscriber elevation_sub_;

            //publisher
            ros::Publisher distances_pub_;
    };
}

#endif // CRAYZFLIE_2_RELATIVE_DISTANCE_CONTROLLER_H
