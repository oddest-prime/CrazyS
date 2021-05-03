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

#include "position_controller_mpc.h"

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

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

PositionControllerMpc::PositionControllerMpc() {

    ROS_INFO_ONCE("Started position controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    // This command diplays which controller is enabled
    if(pnh_node.getParam("enable_mellinger_controller", enable_mellinger_controller_)){
      ROS_INFO("Got param 'enable_mellinger_controller': %d", enable_mellinger_controller_);
    }

    if(pnh_node.getParam("enable_internal_model_controller", enable_internal_model_controller_)){
      ROS_INFO("Got param 'enable_internal_model_controller': %d", enable_internal_model_controller_);
    }

    InitializeParams();

    // Topics subscribe
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &PositionControllerMpc::MultiDofJointTrajectoryCallback, this);
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PositionControllerMpc::OdometryCallback, this);
    enable_sub_ = nh.subscribe("enable", 1, &PositionControllerMpc::EnableCallback, this);

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
      dronestate[i].SetId(droneNumber_, i);
      ROS_INFO("Setup subscriber %s.", nhq[i].getNamespace().c_str());
      odom_sub_[i] = nhq[i].subscribe(mav_msgs::default_topics::ODOMETRY, 1, &DroneStateWithTime::OdometryCallback, &dronestate[i]);
    }

    // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // The subscription to the IMU topic is made only if it is available, when simulating the Crazyflie dynamics considering the also IMU values
    if(pnh_node.getParam("enable_state_estimator", enable_state_estimator_)){
      ROS_INFO("Got param 'enable_state_estimator': %d", enable_state_estimator_);
    }

}

PositionControllerMpc::~PositionControllerMpc(){}

//The callback saves data come from simulation into csv files
void PositionControllerMpc::CallbackSaveData(const ros::TimerEvent& event){

      if(!dataStoring_active_){
         return;
      }

      ofstream fileDistance;

      ROS_INFO("CallbackSavaData PositionControllerMpc. droneNumber: %d", droneNumber_);

      fileDistance.open(std::string("/crazyflie_ws/src/crazys/log_output/Distance") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::app);

      // Saving control signals in a file
      for (unsigned n=0; n < listDistance_.size(); ++n) {
          fileDistance << listDistance_.at( n );
      }

      // Closing all opened files
      fileDistance.close();

      // To have a one shot storing
      dataStoring_active_ = false;

}

void PositionControllerMpc::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  if(enable_swarm_ == SWARM_DISABLED) // set target point if not in swarm mode
  {
    // We can trigger the first command immediately.
    position_controller_.SetTrajectoryPoint(eigen_reference);
    commands_.pop_front();
  }
  else
  {
    target_swarm_ = eigen_reference;
  }

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("MpcController got first MultiDOFJointTrajectory message.");
  }

  ROS_DEBUG("MpcController got MultiDOFJointTrajectory message: x=%f, y=%f, z=%f, yaw=%f", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2], eigen_reference.getYaw());

}

void PositionControllerMpc::InitializeParams() {
  ros::NodeHandle pnh("~");

  ROS_INFO_ONCE("[MPC Position Controller] InitializeParams");

  if(!enable_mellinger_controller_ && !enable_internal_model_controller_){

    // Parameters reading from rosparam.
    GetRosParameter(pnh, "xy_gain_kp/x",
                   position_controller_.controller_parameters_.xy_gain_kp_.x(),
                   &position_controller_.controller_parameters_.xy_gain_kp_.x());
    GetRosParameter(pnh, "xy_gain_kp/y",
                   position_controller_.controller_parameters_.xy_gain_kp_.y(),
                   &position_controller_.controller_parameters_.xy_gain_kp_.y());
    GetRosParameter(pnh, "xy_gain_ki/x",
                   position_controller_.controller_parameters_.xy_gain_ki_.x(),
                   &position_controller_.controller_parameters_.xy_gain_ki_.x());
    GetRosParameter(pnh, "xy_gain_ki/y",
                   position_controller_.controller_parameters_.xy_gain_ki_.y(),
                   &position_controller_.controller_parameters_.xy_gain_ki_.y());

    GetRosParameter(pnh, "attitude_gain_kp/phi",
                   position_controller_.controller_parameters_.attitude_gain_kp_.x(),
                   &position_controller_.controller_parameters_.attitude_gain_kp_.x());
    GetRosParameter(pnh, "attitude_gain_kp/phi",
                   position_controller_.controller_parameters_.attitude_gain_kp_.y(),
                   &position_controller_.controller_parameters_.attitude_gain_kp_.y());
    GetRosParameter(pnh, "attitude_gain_ki/theta",
                   position_controller_.controller_parameters_.attitude_gain_ki_.x(),
                   &position_controller_.controller_parameters_.attitude_gain_ki_.x());
    GetRosParameter(pnh, "attitude_gain_ki/theta",
                   position_controller_.controller_parameters_.attitude_gain_ki_.y(),
                   &position_controller_.controller_parameters_.attitude_gain_ki_.y());

    GetRosParameter(pnh, "rate_gain_kp/p",
                    position_controller_.controller_parameters_.rate_gain_kp_.x(),
                    &position_controller_.controller_parameters_.rate_gain_kp_.x());
    GetRosParameter(pnh, "rate_gain_kp/q",
                    position_controller_.controller_parameters_.rate_gain_kp_.y(),
                    &position_controller_.controller_parameters_.rate_gain_kp_.y());
    GetRosParameter(pnh, "rate_gain_kp/r",
                    position_controller_.controller_parameters_.rate_gain_kp_.z(),
                    &position_controller_.controller_parameters_.rate_gain_kp_.z());
    GetRosParameter(pnh, "rate_gain_ki/p",
                    position_controller_.controller_parameters_.rate_gain_ki_.x(),
                    &position_controller_.controller_parameters_.rate_gain_ki_.x());
    GetRosParameter(pnh, "rate_gain_ki/q",
                    position_controller_.controller_parameters_.rate_gain_ki_.y(),
                    &position_controller_.controller_parameters_.rate_gain_ki_.y());
    GetRosParameter(pnh, "rate_gain_ki/r",
                    position_controller_.controller_parameters_.rate_gain_ki_.z(),
                    &position_controller_.controller_parameters_.rate_gain_ki_.z());

    GetRosParameter(pnh, "yaw_gain_kp/yaw",
                    position_controller_.controller_parameters_.yaw_gain_kp_,
                    &position_controller_.controller_parameters_.yaw_gain_kp_);
    GetRosParameter(pnh, "yaw_gain_ki/yaw",
                    position_controller_.controller_parameters_.yaw_gain_ki_,
                    &position_controller_.controller_parameters_.yaw_gain_ki_);

    GetRosParameter(pnh, "hovering_gain_kp/z",
                    position_controller_.controller_parameters_.hovering_gain_kp_,
                    &position_controller_.controller_parameters_.hovering_gain_kp_);
    GetRosParameter(pnh, "hovering_gain_ki/z",
                    position_controller_.controller_parameters_.hovering_gain_ki_,
                    &position_controller_.controller_parameters_.hovering_gain_ki_);
    GetRosParameter(pnh, "hovering_gain_kd/z",
                    position_controller_.controller_parameters_.hovering_gain_kd_,
                    &position_controller_.controller_parameters_.hovering_gain_kd_);

    position_controller_.SetControllerGains();

    ROS_INFO_ONCE("[Position Controller] Set controller gains and vehicle parameters");

    //Reading the parameters come from the launch file
    std::string dataStoringActive;
    int droneNumber;
    int droneCount;
    double dataStoringTime;
    std::string user;

    if (pnh.getParam("user_account", user)){
    ROS_INFO("Got param 'user_account': %s", user.c_str());
    position_controller_.user_ = user;
    }
    else
       ROS_ERROR("Failed to get param 'user'");

     if (pnh.getParam("droneNumber", droneNumber)){
         ROS_INFO("Got param 'droneNumber': %d", droneNumber);
         position_controller_.droneNumber_ = droneNumber;
         droneNumber_ = droneNumber;
     }
     else
        ROS_ERROR("Failed to get param 'droneNumber'");

     if (pnh.getParam("droneCount", droneCount)){
        ROS_INFO("Got param 'droneCount': %d", droneCount);
        droneCount_ = droneCount;
     }
     else
        ROS_ERROR("Failed to get param 'droneCount'");


    if (pnh.getParam("csvFilesStoring", dataStoringActive)){
        ROS_INFO("Got param 'csvFilesStoring': %s", dataStoringActive.c_str());

        position_controller_.dataStoring_active_ = false;
        if(dataStoringActive == "true")
            position_controller_.dataStoring_active_ = true;

        dataStoring_active_ = true;
        if(dataStoringActive == "true" || dataStoringActive == "distance")
            dataStoring_active_ = true;
    }
    else
       ROS_ERROR("Failed to get param 'csvFilesStoring'");

    if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
        ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
        position_controller_.dataStoringTime_ = dataStoringTime;
    }
    else
       ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

    ros::NodeHandle nh;
    timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &PositionControllerMpc::CallbackSaveData, this, false, true);

    position_controller_.SetLaunchFileParameters();

   }

}

void PositionControllerMpc::Publish(){
}

void PositionControllerMpc::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("MpcController got first imu message.");

    // Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;

    // Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp_ = imu_msg->header.stamp;

}

void PositionControllerMpc::EnableCallback(const std_msgs::Int8ConstPtr& enable_msg) {
  ROS_INFO_ONCE("MpcController got first enable message.");

  enable_swarm_ = enable_msg->data;
}


void PositionControllerMpc::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("MpcController got first odometry message.");

    if (waypointHasBeenPublished_ && enable_state_estimator_){

	    //This functions allows us to put the odometry message into the odometry variable--> _position,
 	    //_orientation,_velocit_body,_angular_velocity
	    eigenOdometryFromMsg(odometry_msg, &odometry_);
	    position_controller_.SetOdometryWithStateEstimator(odometry_);

    }

    if(waypointHasBeenPublished_){

      //This functions allows us to put the odometry message into the odometry variable--> _position,
      //_orientation,_velocit_body,_angular_velocity
      eigenOdometryFromMsg(odometry_msg, &odometry_);
      for (size_t i = 0; i < droneCount_; i++)
          dronestate[i].UpdateDistance(&odometry_);

      position_controller_.SetOdometryWithoutStateEstimator(odometry_);

      ROS_INFO_ONCE("MpcController got odometry message: x=%f y=%f z=%f (%d)", odometry_.position[0], odometry_.position[1], odometry_.position[2], enable_swarm_);

      if(enable_swarm_ == SWARM_DECLARATIVE_SIMPLE)
      {
          ROS_INFO_ONCE("MpcController starting swarm mode (SWARM_DECLARATIVE_SIMPLE)");

          std::stringstream tempDistance;
          tempDistance << odometry_.timeStampSec << "," << odometry_.timeStampNsec << ",";

          EigenOdometry swarm_center;
          for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
          {
              swarm_center.position[0] += dronestate[i].odometry_.position[0];
              swarm_center.position[1] += dronestate[i].odometry_.position[1];
              swarm_center.position[2] += dronestate[i].odometry_.position[2];
          }
          swarm_center.position[0] /= (float)droneCount_;
          swarm_center.position[1] /= (float)droneCount_;
          swarm_center.position[2] /= (float)droneCount_;
          ROS_INFO_ONCE("MpcController %d swarm center x=%f y=%f z=%f", droneNumber_, swarm_center.position[0], swarm_center.position[1], swarm_center.position[2]);

          float min_sum = FLT_MAX;
          int min_xi = 0;
          int min_yi = 0;
          int min_zi = 0;

          for(int xi = -2; xi <= 2; xi ++) // iterate over all possible next actions in x-, y- and z-dimension
          {
              for(int yi = -2; yi <= 2; yi ++)
              {
                  for(int zi = -2; zi <= 2; zi ++)
                  {
                      float cohesion_sum = 0;
                      float separation_sum = 0;
                      int neighbourhood_cnt = 0;
                      float total_sum = 0;
                      EigenOdometry potential_pos = odometry_;
                      potential_pos.position[0] += (float)xi * 0.07;
                      potential_pos.position[1] += (float)yi * 0.07;
                      potential_pos.position[2] += (float)zi * 0.07;
                      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
                      {
                          float dist = dronestate[i].GetDistance(&potential_pos);
                          if(dataStoring_active_ && xi == 0 && yi == 0 && zi == 0) // save distance to log file for current position
                              tempDistance << dist << ",";

                          if(i == droneNumber_) // skip for own quadcopter
                            continue;
//                          cohesion_sum += dist*dist;
                          if(dist < 0.85 && dist != 0) // neighbourhood for separation
                          {
                            neighbourhood_cnt ++;
                            cohesion_sum += dist*dist;
                            separation_sum += 1.0/(dist*dist);
                          }
                      }
                      // coehesion term
                      // total_sum = 20.0*cohesion_sum / ((float)droneCount_);
                      if(neighbourhood_cnt > 0)
                      {
                          // coehesion term
                          total_sum = 20.0*cohesion_sum / ((float)neighbourhood_cnt);
                          // separation term
                          total_sum += 2.5*separation_sum / ((float)neighbourhood_cnt);
                      }
                      // target direction term
                      if(target_swarm_.position_W[2] != 0) // target point is available (z != 0)
                      {
                          float target_distance_x = fabs(target_swarm_.position_W[0] - potential_pos.position[0]);
                          float target_distance_y = fabs(target_swarm_.position_W[1] - potential_pos.position[1]);
                          float target_distance_z = fabs(target_swarm_.position_W[2] - potential_pos.position[2]);
                          total_sum += 25.0*(target_distance_x*target_distance_x + target_distance_y*target_distance_y + target_distance_z*target_distance_z);
                          ROS_INFO_ONCE("MpcController %d swarm target x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
                      }
                      // keep moving term (leads to wobbly behaviour)
                      //float move_dist = sqrt(fabs(xi)*fabs(xi) + fabs(yi)*fabs(yi) + fabs(zi)*fabs(zi));
                      //total_sum += 1.0/(move_dist*move_dist);

                      // keep moving term (normal to swarm center)
                      EigenOdometry center_vector = Difference(&swarm_center, &odometry_);
                      EigenOdometry move_vector = Difference(&potential_pos, &odometry_);
                      EigenOdometry prod_vector = CrossProduct(&center_vector, &move_vector);
                      //total_sum += 0.01/SquaredScalarLength(&prod_vector);
                      //ROS_INFO("MpcController %d coh=%f sep=%f ssl=%f", droneNumber_, cohesion_sum, separation_sum, SquaredScalarLength(&prod_vector));

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
          ROS_INFO_ONCE("MpcController %d swarm direction xi=%d yi=%d zi=%d tsum=%f", droneNumber_, min_xi, min_yi, min_zi, min_sum);
          mav_msgs::EigenTrajectoryPoint new_setpoint;
          new_setpoint.position_W = odometry_.position;
          new_setpoint.position_W[0] += (float)min_xi * 0.07;
          new_setpoint.position_W[1] += (float)min_yi * 0.07;
          new_setpoint.position_W[2] += (float)min_zi * 0.07;
          position_controller_.SetTrajectoryPoint(new_setpoint);

          tempDistance << "\n";
          listDistance_.push_back(tempDistance.str());
    }
    else if(enable_swarm_ == SWARM_REYNOLDS)
    {
        ROS_INFO_ONCE("MpcController starting swarm mode (SWARM_REYNOLDS)");

        std::stringstream tempDistance;
        tempDistance << odometry_.timeStampSec << "," << odometry_.timeStampNsec << ",";

        ROS_INFO_ONCE("MpcController %d vel x=%f y=%f z=%f", droneNumber_, odometry_.velocity[0], odometry_.velocity[1], odometry_.velocity[2]);
        EigenOdometry integrated_velocity;
        float weigthed_delta_t = 0.5;
        integrated_velocity.position[0] = odometry_.velocity[0] * weigthed_delta_t;
        integrated_velocity.position[1] = odometry_.velocity[1] * weigthed_delta_t;
        integrated_velocity.position[2] = odometry_.velocity[2] * weigthed_delta_t;
        EigenOdometry position_next = Sum(&odometry_, &integrated_velocity);

        float abs_velocity = sqrt(SquaredScalarVelocity(&odometry_));
        ROS_INFO_ONCE("MpcController %d vel=%f", droneNumber_, abs_velocity);

        int neighbourhood_cnt = 0;
        EigenOdometry cohesion_sum;
        EigenOdometry separation_sum;
        for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
        {
            float dist_cur = dronestate[i].GetDistance(&odometry_);
            if(dataStoring_active_) // save distance to log file for current position
                tempDistance << dist_cur << ",";

            if(i == droneNumber_) // skip for own quadcopter
              continue;

            float dist = dronestate[i].GetDistance(&position_next);
            // if(dist < 99 && dist != 0) // global neighbourhood
            if(dist < 0.65 && dist != 0) // neighbourhood distance
            {
              neighbourhood_cnt ++;

              EigenOdometry cohesion_dist = Difference(&dronestate[i].odometry_, &position_next);
              cohesion_sum = Sum(&cohesion_dist, &cohesion_sum);

              EigenOdometry separation_dist = Difference(&position_next, &dronestate[i].odometry_);
              float separation_len = sqrt(SquaredScalarLength(&separation_dist));
              separation_len -= 0; // epsilon_D = 0.2
              if(separation_len <= 0)
                separation_len = 0.001;
              separation_len = separation_len*separation_len;
              separation_dist.position[0] /= separation_len;
              separation_dist.position[1] /= separation_len;
              separation_dist.position[2] /= separation_len;
              separation_sum = Sum(&separation_dist, &separation_sum);

              ROS_INFO_ONCE("MpcController %d (i=%d) accel x=%f y=%f z=%f", droneNumber_, (int)i, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2]);
            }
        }

        EigenOdometry cohesion_accel;
        EigenOdometry separation_accel;
        EigenOdometry target_accel;
        float global_factor = 5;
        float cohesion_factor = 0.4 * global_factor;
        float separation_factor = (0.1 + abs_velocity / 15) * global_factor;
        float target_factor = 0.08 * global_factor;
        float target_accel_limit = 0.07 * global_factor;
        if(neighbourhood_cnt != 0)
        {
          cohesion_accel.position[0] = cohesion_factor * (cohesion_sum.position[0] / (float)neighbourhood_cnt);
          cohesion_accel.position[1] = cohesion_factor * (cohesion_sum.position[1] / (float)neighbourhood_cnt);
          cohesion_accel.position[2] = cohesion_factor * (cohesion_sum.position[2] / (float)neighbourhood_cnt);
          separation_accel.position[0] = separation_factor * (separation_sum.position[0] / (float)neighbourhood_cnt);
          separation_accel.position[1] = separation_factor * (separation_sum.position[1] / (float)neighbourhood_cnt);
          separation_accel.position[2] = separation_factor * (separation_sum.position[2] / (float)neighbourhood_cnt);
        }
        target_accel.position[0] = target_factor * (target_swarm_.position_W[0] - position_next.position[0]);
        target_accel.position[1] = target_factor * (target_swarm_.position_W[1] - position_next.position[1]);
        target_accel.position[2] = target_factor * (target_swarm_.position_W[2] - position_next.position[2]);
        float abs_target_accel = sqrt(SquaredScalarLength(&target_accel)); // length of vector
        if(abs_target_accel > target_accel_limit) // if limit exceeded
          target_accel_limit = target_accel_limit / abs_target_accel;
        else
          target_accel_limit = 1;
        target_accel.position[0] *= target_accel_limit;
        target_accel.position[1] *= target_accel_limit;
        target_accel.position[2] *= target_accel_limit;

        EigenOdometry accel = Sum(&cohesion_accel, &separation_accel);
        accel = Sum(&accel, &target_accel);

        ROS_INFO_ONCE("MpcController %d (|H|=%d) cohesion_accel x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, cohesion_accel.position[0], cohesion_accel.position[1], cohesion_accel.position[2]);
        ROS_INFO_ONCE("MpcController %d (|H|=%d) separation_accel x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, separation_accel.position[0], separation_accel.position[1], separation_accel.position[2]);
        ROS_INFO_ONCE("MpcController %d target_accel=%f x=%f y=%f z=%f", droneNumber_, abs_target_accel, target_accel.position[0], target_accel.position[1], target_accel.position[2]);
        ROS_INFO_ONCE("MpcController %d (|H|=%d) accel x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, accel.position[0], accel.position[1], accel.position[2]);

        //position_controller_.SetSetPoint(1.5, atan(accel.position[0]), atan(0-accel.position[1]), 0);
        // MpcController::SetSetPoint(double z, double pitch, double roll, double yaw)

        mav_msgs::EigenTrajectoryPoint new_setpoint;
        new_setpoint.position_W = odometry_.position;
        new_setpoint.position_W[0] += accel.position[0];
        new_setpoint.position_W[1] += accel.position[1];
        new_setpoint.position_W[2] += accel.position[2];
        position_controller_.SetTrajectoryPoint(new_setpoint);

        tempDistance << "\n";
        listDistance_.push_back(tempDistance.str());
    }


      Eigen::Vector4d ref_rotor_velocities;
      position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

      //creating a new mav message. actuator_msg is used to send the velocities of the propellers.
      mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

      //we use clear because we later want to be sure that we used the previously calculated velocity.
      actuator_msg->angular_velocities.clear();
      //for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
      for (int i = 0; i < ref_rotor_velocities.size(); i++)
         actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
      actuator_msg->header.stamp = odometry_msg->header.stamp;

      motor_velocity_reference_pub_.publish(actuator_msg);

    }

}

// IMU messages are sent to the controller with a frequency of 500Hz. In other words, with a sampling time of 0.002 seconds
void PositionControllerMpc::CallbackIMUUpdate(const ros::TimerEvent& event){

    position_controller_.SetSensorData(sensors_);

    ROS_INFO_ONCE("IMU Message sent to position controller");

    if (waypointHasBeenPublished_){

	    Eigen::Vector4d ref_rotor_velocities;
	    position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

	    // A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
	    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	    // The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular velocities).
	    actuator_msg->angular_velocities.clear();
	    // for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	    for (int i = 0; i < ref_rotor_velocities.size(); i++)
	       actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	    actuator_msg->header.stamp = imu_msg_head_stamp_;

	    motor_velocity_reference_pub_.publish(actuator_msg);

    }

}


void DroneStateWithTime::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("DroneStateWithTime got first odometry message.");
    eigenOdometryFromMsg(odometry_msg, &odometry_);

    ROS_DEBUG("DroneStateWithTime got odometry message: x=%f y=%f z=%f (self:%d, other:%d)", odometry_.position[0], odometry_.position[1], odometry_.position[2], self_, other_);
}

float DroneStateWithTime::GetDistance(EigenOdometry* odometry) {
    if(self_ == other_)
        return 0;

    float distance_x = fabs(odometry_.position[0] - odometry->position[0]);
    float distance_y = fabs(odometry_.position[1] - odometry->position[1]);
    float distance_z = fabs(odometry_.position[2] - odometry->position[2]);
    return sqrt(distance_x*distance_x + distance_y*distance_y + distance_z*distance_z);
}

void DroneStateWithTime::UpdateDistance(EigenOdometry* odometry) {
    distance_ = this->GetDistance(odometry);

    ROS_INFO_ONCE("DroneStateWithTime distance=%f (self:%d, other:%d)", distance_, self_, other_);
}

void DroneStateWithTime::SetId(int self, int other)
{
    self_ = self;
    other_ = other;
}

EigenOdometry CrossProduct(EigenOdometry* a, EigenOdometry* b)
{
    EigenOdometry r;
    r.position[0] = a->position[1]*b->position[2] - a->position[2]*b->position[1];
    r.position[1] = a->position[2]*b->position[0] - a->position[0]*b->position[2];
    r.position[2] = a->position[0]*b->position[1] - a->position[1]*b->position[0];
    return r;
}

EigenOdometry Difference(EigenOdometry* a, EigenOdometry* b)
{
    EigenOdometry r;
    r.position[0] = a->position[0] - b->position[0];
    r.position[1] = a->position[1] - b->position[1];
    r.position[2] = a->position[2] - b->position[2];
    return r;
}

EigenOdometry Sum(EigenOdometry* a, EigenOdometry* b)
{
    EigenOdometry r;
    r.position[0] = a->position[0] + b->position[0];
    r.position[1] = a->position[1] + b->position[1];
    r.position[2] = a->position[2] + b->position[2];
    return r;
}

float SquaredScalarLength(EigenOdometry* a)
{
    return (float) (a->position[0]*a->position[0] + a->position[1]*a->position[1] + a->position[2]*a->position[2]);
}

float SquaredScalarVelocity(EigenOdometry* a)
{
    return (float) (a->velocity[0]*a->velocity[0] + a->velocity[1]*a->velocity[1] + a->velocity[2]*a->velocity[2]);
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node_with_stateEstimator");

    ros::NodeHandle nh2;

    rotors_control::PositionControllerMpc position_controller_node;

    ros::spin();

    return 0;
}
