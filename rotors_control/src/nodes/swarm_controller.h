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

#ifndef CRAYZFLIE_2_SWARM_CONTROLLER_H
#define CRAYZFLIE_2_SWARM_CONTROLLER_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <random>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ros/time.h>


#include "rotors_control/common.h"
#include "rotors_control/mpc_controller.h"
#include "rotors_control/position_controller.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define N_DRONES_MAX  30          /* maximum number of drones */
#define N_OBSTACLE_MAX  20        /* maximum number of obstacles */

#define SWARM_DISABLED            0
#define SWARM_DECLARATIVE_SIMPLE  1
#define SWARM_REYNOLDS            2
#define SWARM_REYNOLDS_LIMITED    4
#define SWARM_REYNOLDS_VELOCITY   8
#define SWARM_GRADIENT            16
#define SWARM_GRADIENT_ENUM       32
#define SWARM_PHASE_ESTABLISHED   64
#define SWARM_LANDING             32768

namespace rotors_control {

    EigenOdometry CrossProduct(EigenOdometry* a, EigenOdometry* b);

    EigenOdometry operator-(const EigenOdometry& a, const EigenOdometry& b);
    EigenOdometry operator+(const EigenOdometry& a, const EigenOdometry& b);
    EigenOdometry operator*(const EigenOdometry& a, const float& b);
    EigenOdometry operator/(const EigenOdometry& a, const float& b);
    float norm_squared(const EigenOdometry& a);
    float norm(const EigenOdometry& a);

    EigenOdometry DifferenceVelocity(EigenOdometry* a, EigenOdometry* b);
    EigenOdometry SumVelocity(EigenOdometry* a, EigenOdometry* b);
    float SquaredScalarVelocity(EigenOdometry* a);

    class DroneStateWithTime {
     public:
      void SetId(int self, int other, float position_noise);
      void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
      void UpdateDistance(EigenOdometry* odometry, EigenOdometry* odometry_gt);
      float GetDistance(EigenOdometry* odometry);
      float GetDistance_gt(EigenOdometry* odometry_gt);

      int rand_cnt_;
      float rand_x_;
      float rand_y_;
      float rand_z_;
      std::default_random_engine generator;
      float position_noise_;

      int self_;
      int other_;
      float distance_; // with simulated sensor noise
      float distance_gt_; // ground-truth
      EigenOdometry odometry_; // with simulated sensor noise
      EigenOdometry odometry_gt_; // ground-truth
    };

    class SwarmController{
        public:
            SwarmController();
            ~SwarmController();

            void InitializeParams();
            void Publish();

        private:

            bool waypointHasBeenPublished_ = false;
            bool enable_state_estimator_ = false;
            bool enable_mellinger_controller_ = false;
            bool enable_internal_model_controller_ = false;
            bool dataStoring_active_;
            int enable_swarm_ = SWARM_DISABLED;

            int obstacleScenario_;
            int droneNumber_;
            int droneCount_;
            int neighbourhood_cnt_;

//            PositionController position_controller_;
            MpcController position_controller_;
            sensorData_t sensors_;
            EigenOdometry odometry_; // with simulated sensor noise
            EigenOdometry odometry_gt_; // ground-truth

            int rand_cnt_;
            float rand_x_;
            float rand_y_;
            float rand_z_;
            std::default_random_engine generator;

            float position_noise_;
            float neighbourhood_distance_;
            float eps_move_;
            int n_move_max_;
            float mpc_cohesion_weight_;
            float mpc_separation_weight_;
            float mpc_target_weight_;
            float mpc_obstacle_weight_;
            float weighted_delta_t_;
            float reynolds_global_factor_;
            float reynolds_velocity_factor_;
            float reynolds_cohesion_factor_;
            float reynolds_separation_factor_;
            float reynolds_target_factor_;
            float reynolds_target_accel_limit_;
            float reynolds_accel_limit_;
            float gradient_scale_factor_;


            mav_msgs::EigenTrajectoryPoint target_swarm_;

            ros::Time imu_msg_head_stamp_;

            std::string namespace_;

            ros::NodeHandle n_;
            ros::Timer timer_Attitude_;
            ros::Timer timer_highLevelControl;
            ros::Timer timer_IMUUpdate;
            ros::Timer timer_saveData;

            //Callback functions to compute the errors among axis and angles
            void CallbackAttitudeEstimation(const ros::TimerEvent& event);
            void CallbackHightLevelControl(const ros::TimerEvent& event);
            void CallbackIMUUpdate(const ros::TimerEvent& event);
            void CallbackSaveData(const ros::TimerEvent& event);

            void FileSaveData(void);

            //subscribers
            ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
            ros::Subscriber cmd_multi_dof_joint_trajectory_spline_sub_;
            ros::Subscriber pose_sub_;
            ros::Subscriber enable_sub_;
            ros::Subscriber imu_sub_;
            ros::Subscriber imu_ideal_sub_;
            ros::Subscriber keyboard_sub_;
            ros::Subscriber pose_other_sub_[N_DRONES_MAX];

            //publisher
            ros::Publisher motor_velocity_reference_pub_;
            ros::Publisher setpoint_pub_;

            mav_msgs::EigenTrajectoryPointDeque commands_;
            std::deque<ros::Duration> command_waiting_times_;
            ros::Timer command_timer_;

            void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
            void MultiDofJointTrajectoryMellingerCallback(const mav_msgs::DroneState& drone_state_msg);

            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void EnableCallback(const std_msgs::Int32ConstPtr& enable_msg);
            void MellingerOdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
            void KeyboardCallback(const std_msgs::Int32Ptr& msg);

            void IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg);
            void IMUMellingerCallback(const sensor_msgs::ImuConstPtr& imu_msg); //When the Mellinger's controller is on

            DroneStateWithTime dronestate[N_DRONES_MAX];

            // Lists for data saving
            std::vector<string> listDistance_;
            std::vector<string> listMetrics_;
            std::vector<string> listState_;
    };
}

#endif // CRAYZFLIE_2_SWARM_CONTROLLER_H
