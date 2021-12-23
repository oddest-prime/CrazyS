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

#ifndef CRAYZFLIE_2_SWARM_STATE_ESTIMATOR_H
#define CRAYZFLIE_2_SWARM_STATE_ESTIMATOR_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
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

#define N_DRONES_MAX  20          /* maximum number of drones */

namespace rotors_control {
    using namespace Eigen;

    float Distance(EigenOdometry* a, EigenOdometry* b);
    float EuclideanNorm(EigenOdometry* a);
    EigenOdometry CrossProduct(EigenOdometry* a, EigenOdometry* b);
    float DotProduct(EigenOdometry* a, EigenOdometry* b);

    EigenOdometry operator-(const EigenOdometry& a, const EigenOdometry& b);
    EigenOdometry operator+(const EigenOdometry& a, const EigenOdometry& b);
    EigenOdometry operator*(const EigenOdometry& a, const float& b);
    EigenOdometry operator/(const EigenOdometry& a, const float& b);
    EigenOdometry operator*(const Eigen::Matrix3f& b, const EigenOdometry& a);
    EigenOdometry operator*(const EigenOdometry& a, const Eigen::Matrix3f& b);

    Eigen::Matrix3f RotationMatrixFromAxisAngle(const Eigen::Vector3f& axis, const float& angle);
    Eigen::Matrix3f IdentityMatrix();
    std::string MatrixToString(const Eigen::Matrix3f& a);
    std::string VectorToString(const Eigen::Vector3f& a);
    std::string toString(const EigenOdometry& a);

    class SwarmStateEstimator{
        public:
            SwarmStateEstimator();
            ~SwarmStateEstimator();

            void InitializeParams();

        private:
            bool dataStoring_active_;

            int droneNumber_;
            int droneCount_;

            int history_cnt_;

            EigenOdometry odometry_gt_; // ground-truth
            EigenOdometry odometry_gt_history1_; // ground-truth
            EigenOdometry odometry_gt_history2_; // ground-truth
            Vector3f odometry_estimate_[N_DRONES_MAX]; // position estimates for other drones
            Vector3f odometry_estimate_history1_[N_DRONES_MAX]; // position estimate history for other drones
            Vector3f odometry_estimate_history2_[N_DRONES_MAX]; // position estimate history for other drones
            float distances_[N_DRONES_MAX][N_DRONES_MAX]; // received distance measurements
            float elevation_[N_DRONES_MAX]; // received elevation measurements

            int best_triangle_[4];
            int best_zset_[3];
            int best_xydist_[3];

            std::string namespace_;

            ros::NodeHandle n_;
            ros::Timer timer_saveData;

            ros::ServiceClient gazebo_client_;
            float visual_cnt_;

            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void DistancesCallback(const std_msgs::Float32MultiArray& distances_msg);
            void ElevationCallback(const std_msgs::Float32MultiArray& elevation_msg);

            void CallbackSaveData(const ros::TimerEvent& event);
            void FileSaveData(void);

            void FindBestTriangle(float (*distances)[N_DRONES_MAX], int* triangle);
            void FindBestZset(float (*distances)[N_DRONES_MAX], Vector3f* positions, int* zset);
            void FindBestXYdist(Vector3f* positions, int* xydist);
            void InferPositions(float (*distances)[N_DRONES_MAX], int* triangle, Vector3f* positions);
            void InferRotationZ(Vector3f* positions, float* elevation, int* zset);
            void InferRotationMovement(Vector3f* positions, Vector3f* positions_moved, int* xydist, const Vector3f& movement);
            void InferRotationMovementEzy(Vector3f* positions, Vector3f* positions_moved, int* xydist, const Vector3f& movement);
            void CheckDistances(float (*distances)[N_DRONES_MAX], Vector3f* positions);
            void RotatePositions(Vector3f* positions, Eigen::Matrix3f* rotation, Vector3f* result);
            void MirrorHotfix(Vector3f* positions, int gt_index, const Vector3f& fixed_gt, const Vector3f& own_gt);

            //subscribers
            ros::Subscriber odometry_sub_;
            ros::Subscriber distances_sub_;
            ros::Subscriber elevation_sub_;

            //publisher
            ros::Publisher distances_pub_;
    };
}

#endif // CRAYZFLIE_2_SWARM_STATE_ESTIMATOR_H
