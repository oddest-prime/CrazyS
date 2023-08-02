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

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <limits>
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
#include <gazebo_msgs/ModelStates.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <torch/script.h> // for ML

#include "rotors_control/common.h"
#include "rotors_control/Eigen.h"

#define N_DRONES_MAX  20          /* maximum number of drones */
#define N_BEACONS_MAX  5          /* maximum number of beacons */
#define N_VECTORS_MAX  3          /* number of saved unit vectors */

#define HISTORY_CNT_MAX           512

#define N_STEPS                   5

#define CYCLIC_PHASE_REST             0
#define CYCLIC_PHASE_IDENTIFY_A       1
#define CYCLIC_PHASE_IDENTIFY_B       2
#define CYCLIC_PHASE_IDENTIFY_C       3
#define CYCLIC_PHASE_CONTROL          4


namespace rotors_control {
    using namespace Eigen;

    void linearLeastSquaresApproximation(float* dataset, size_t n_points, float* alpha0, float* alpha1);

    class DroneStateWithTime {
     public:
      void SetId(int self, int other);
      void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
      float GetDistance_gt(EigenOdometry* odometry_gt);
      float GetDistance_sim_gt(DroneStateWithTime* own_state, Vector3f potential_movement);

      int self_;
      int other_;
      EigenOdometry odometry_; // ground-truth
    };

    class RelativeDistanceController{
        public:
            RelativeDistanceController();
            ~RelativeDistanceController();

            void InitializeParams();
            void InitializeML();

        private:
            std::mt19937 generator_;

            int droneNumber_;
            int droneCount_;
            int beaconCount_;
            int isSim_;

            int n_move_max_;
            float drone_radius_;
            float spc_cohesion_weight_;
            float spc_separation_weight_;
            float spc_target_weight_;
            float spc_height_weight_;
            float spc_limit_weight_;
            float spc_calm_weight_;
            float eps_move_;
            float neighbourhood_distance_;
            float explore_command_length_;
            float explore_movement_thr_;
            float explore_distance_cap_;
            float velocity_scaling_;
            float distance_iir_filter_;
            float elevation_iir_filter_;
            float distance_min_filter_;
            float distance_max_filter_;
            float extra_separation_distance_;
            float target_elevation_;
            float swarm_elevation_;
            int inner_controller_;
            int noise_suppression_;

            bool dataStoring_active_;
            int enable_swarm_ = SWARM_DISABLED;
            int current_target_ = 0;
            int history_cnt_;
            Vector3f random_direction_; // save random exploration direction
            bool waypointHasBeenPublished_ = false;
            mav_msgs::EigenTrajectoryPoint target_swarm_;

            EigenOdometry odometry_; // ground-truth
            EigenOdometry odometry_history1_; // ground-truth at history point

            float distances_[N_DRONES_MAX][N_DRONES_MAX]; // received distance measurements
            float distances_filtered_[N_DRONES_MAX][N_DRONES_MAX]; // distance measurements filtered
            float distances_history1_[N_DRONES_MAX]; // distance measurements to own drone at history point
            float distances_history_[N_DRONES_MAX][HISTORY_CNT_MAX]; // distance measurements to own drone history

            float beacons_[N_DRONES_MAX][N_BEACONS_MAX]; // received distance measurements
            float beacons_filtered_[N_DRONES_MAX][N_BEACONS_MAX]; // distance measurements filtered
            float beacons_history1_[N_BEACONS_MAX]; // distance measurements to own drone at history point
            //float beacons_last_[N_BEACONS_MAX]; // distance measurements from previous message (to check for large changes, when target is updated)
            float beacons_history_[N_BEACONS_MAX][HISTORY_CNT_MAX]; // distance measurements to own drone history

            float elevation_[N_DRONES_MAX]; // received elevation measurements
            float elevation_filtered_[N_DRONES_MAX]; // received elevation measurements

            Vector3f positions_gt_[N_DRONES_MAX]; // ground-truth positions of all drones, only to be used for verification of estimation

            Vector3f unit_vectors_[N_VECTORS_MAX]; // directions of unit vectors
            int unit_vectors_age_[N_VECTORS_MAX]; // age of unit vectors
            float distances_differences_[N_VECTORS_MAX][N_DRONES_MAX]; // change in distances for unit vectors
            float beacons_differences_[N_VECTORS_MAX][N_BEACONS_MAX]; // change in beacon distances for unit vectors
            Matrix3f transform_vectors_;
            int transform_ok_;
            int transform_available_;

            int cyclic_current_phase_;
            EigenOdometry cyclic_odometry_history0_; // ground-truth at history point
            EigenOdometry cyclic_odometry_history1_; // ground-truth at history point
            EigenOdometry cyclic_odometry_history2_; // ground-truth at history point
            EigenOdometry cyclic_odometry_history3_; // ground-truth at history point
            float cyclic_distances_history0_[N_DRONES_MAX]; // distance measurements to own drone at history point
            float cyclic_distances_history1_[N_DRONES_MAX]; // distance measurements to own drone at history point
            float cyclic_distances_history2_[N_DRONES_MAX]; // distance measurements to own drone at history point
            float cyclic_distances_history3_[N_DRONES_MAX]; // distance measurements to own drone at history point

            torch::jit::script::Module distances_model_[N_DRONES_MAX];
            c10::IValue distances_hx_[N_DRONES_MAX];
            torch::jit::script::Module beacons_model_[N_BEACONS_MAX];
            c10::IValue beacons_hx_[N_BEACONS_MAX];
            EigenOdometry odometry_last_distances_measurement_;
            EigenOdometry odometry_last_beacons_measurement_;

            std::string namespace_;

            ros::NodeHandle n_;
            ros::Timer timer_saveData;

            ros::ServiceClient gazebo_client_;

            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void DistancesCallback(const std_msgs::Float32MultiArray& distances_msg);
            void PositionsCallback(const std_msgs::Float32MultiArray& positions_msg);
            void ElevationCallback(const std_msgs::Float32MultiArray& elevation_msg);
            void BeaconsCallback(const std_msgs::Float32MultiArray& distances_msg);
            void EnableCallback(const std_msgs::Int32ConstPtr& enable_msg);
            void TargetCallback(const std_msgs::Int32ConstPtr& target_msg);
            void UpdateCallback(const std_msgs::Int32ConstPtr& update_msg);
            void SaveLogCallback(const std_msgs::Int32ConstPtr& save_msg);
            void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
            void CallbackSaveData(const ros::TimerEvent& event);
            void ModelstateCallback(const gazebo_msgs::ModelStatesConstPtr& modelstates_msg);

            void FileSaveData(void);

            void TrilaterationCalculation(EigenOdometry* history0,
                                          EigenOdometry* history1,
                                          EigenOdometry* history2,
                                          EigenOdometry* history3,
                                          float* distances_history0,
                                          float* distances_history1,
                                          float* distances_history2,
                                          float* distances_history3,
                                          EigenOdometry* current
                                        );

            //subscribers
            ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
            ros::Subscriber odometry_sub_;
            ros::Subscriber distances_sub_;
            ros::Subscriber positions_sub_;
            ros::Subscriber elevation_sub_;
            ros::Subscriber beacons_sub_;
            ros::Subscriber enable_sub_;
            ros::Subscriber target_sub_;
            ros::Subscriber update_sub_;
            ros::Subscriber logsave_sub_;
            ros::Subscriber pose_other_sub_[N_DRONES_MAX];
            ros::Subscriber modelstate_sub_;

            //publisher
            ros::Publisher setpoint_pub_;

            Vector3f beacon_gt_[N_BEACONS_MAX]; // ground-truth beacon positions
            DroneStateWithTime dronestate_[N_DRONES_MAX];

            // Lists for data saving
            std::vector<std::string> listDistance_;
            std::vector<std::string> listEnv_;
            std::vector<std::string> listState_;
            std::vector<std::string> listCost_;
            std::vector<std::string> listVectors_;
    };
}

#endif // CRAYZFLIE_2_RELATIVE_DISTANCE_CONTROLLER_H
