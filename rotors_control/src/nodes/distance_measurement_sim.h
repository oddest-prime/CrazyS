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

#ifndef CRAYZFLIE_2_DISTANCE_MEASUREMENT_SIM_H
#define CRAYZFLIE_2_DISTANCE_MEASUREMENT_SIM_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <random>


#include <geometry_msgs/PoseStamped.h>
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
    class DroneStateWithTime {
     public:
      void SetId(int droneNumber, int droneCount, float position_noise, DroneStateWithTime* dronestate, ros::Publisher* distances_pub);
      void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

      // int rand_cnt_;
      std::default_random_engine generator_;
      float distance_noise_;

      int droneNumber_;
      int droneCount_;

      DroneStateWithTime* dronestate_;
      ros::Publisher* distances_pub_;

      float distances_[N_DRONES_MAX]; // with simulated sensor noise
      float distances_gt_[N_DRONES_MAX]; // ground-truth
      EigenOdometry odometry_gt_; // ground-truth
    };

    class DistanceMeasurementSim{
        public:
            DistanceMeasurementSim();
            ~DistanceMeasurementSim();

            void InitializeParams();
            void Publish();

        private:
            bool dataStoring_active_;

            int droneNumber_;
            int droneCount_;
            int neighbourhood_cnt_;

//            PositionController position_controller_;
            EigenOdometry odometry_; // with simulated sensor noise
            EigenOdometry odometry_gt_; // ground-truth

            float distance_noise_;

            std::string namespace_;

            ros::NodeHandle n_;
            ros::Timer timer_saveData;

            void CallbackSaveData(const ros::TimerEvent& event);
            void FileSaveData(void);

            //subscribers
            ros::Subscriber odometry_sub_[N_DRONES_MAX];

            //publisher
            ros::Publisher distances_pub_;

            DroneStateWithTime dronestate[N_DRONES_MAX];
    };
}

#endif // CRAYZFLIE_2_DISTANCE_MEASUREMENT_SIM_H
