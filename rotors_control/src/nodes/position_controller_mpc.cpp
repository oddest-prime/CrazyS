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
      ofstream fileMetrics;
      ofstream fileState;

      ROS_INFO("CallbackSavaData PositionControllerMpc. droneNumber: %d", droneNumber_);

      fileDistance.open(std::string("/tmp/log_output/Distance") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::app);
      fileMetrics.open(std::string("/tmp/log_output/Metrics") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::app);
      fileState.open(std::string("/tmp/log_output/State") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::app);

      // Saving distances from every to every dron in a file
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

      // Closing all opened files
      fileDistance.close();
      fileMetrics.close();
      fileState.close();

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

    GetRosParameter(pnh, "swarm/neighbourhood_distance", (float)99, &neighbourhood_distance_);
    GetRosParameter(pnh, "mpc1/eps_move", (float)0.1, &eps_move_);
    GetRosParameter(pnh, "mpc1/n_move_max", (int)2, &n_move_max_);
    GetRosParameter(pnh, "mpc1/mpc_cohesion_weight", (float)1.0, &mpc_cohesion_weight_);
    GetRosParameter(pnh, "mpc1/mpc_separation_weight", (float)1.0, &mpc_separation_weight_);
    GetRosParameter(pnh, "mpc1/mpc_target_weight", (float)1.0, &mpc_target_weight_);

    GetRosParameter(pnh, "reynolds/weighted_delta_t", (float)0.5, &weighted_delta_t_);
    GetRosParameter(pnh, "reynolds/global_factor", (float)0.5, &reynolds_global_factor_);
    GetRosParameter(pnh, "reynolds/velocity_factor", (float)0.5, &reynolds_velocity_factor_);
    GetRosParameter(pnh, "reynolds/cohesion_factor", (float)0.5, &reynolds_cohesion_factor_);
    GetRosParameter(pnh, "reynolds/separation_factor", (float)0.5, &reynolds_separation_factor_);
    GetRosParameter(pnh, "reynolds/target_factor", (float)0.5, &reynolds_target_factor_);
    GetRosParameter(pnh, "reynolds/target_accel_limit", (float)0.5, &reynolds_target_accel_limit_);
    GetRosParameter(pnh, "reynolds/accel_limit", (float)0.5, &reynolds_accel_limit_);

    GetRosParameter(pnh, "gradient/scale_factor", (float)0.01, &gradient_scale_factor_);

    reynolds_velocity_factor_ *= reynolds_global_factor_;
    reynolds_cohesion_factor_ *= reynolds_global_factor_;
    reynolds_separation_factor_ *= reynolds_global_factor_;
    reynolds_target_factor_ *= reynolds_global_factor_;
    reynolds_target_accel_limit_ *= reynolds_global_factor_;
    reynolds_accel_limit_ *= reynolds_global_factor_;

    ROS_INFO_ONCE("[Swarm Controller] GetRosParameter values:");
    ROS_INFO_ONCE("  swarm/neighbourhood_distance=%f", neighbourhood_distance_);
    ROS_INFO_ONCE("  mpc1/eps_move=%f", eps_move_);
    ROS_INFO_ONCE("  mpc1/n_move_max=%d", n_move_max_);
    ROS_INFO_ONCE("  mpc1/mpc_cohesion_weight=%f", mpc_cohesion_weight_);
    ROS_INFO_ONCE("  mpc1/mpc_separation_weight=%f", mpc_separation_weight_);
    ROS_INFO_ONCE("  mpc1/mpc_target_weight=%f", mpc_target_weight_);
    ROS_INFO_ONCE("  reynolds/weighted_delta_t=%f", weighted_delta_t_);
    ROS_INFO_ONCE("  reynolds/global_factor=%f", reynolds_global_factor_);
    ROS_INFO_ONCE("  reynolds/velocity_factor=%f", reynolds_velocity_factor_);
    ROS_INFO_ONCE("  reynolds/cohesion_factor=%f", reynolds_cohesion_factor_);
    ROS_INFO_ONCE("  reynolds/separation_factor=%f", reynolds_separation_factor_);
    ROS_INFO_ONCE("  reynolds/target_factor=%f", reynolds_target_factor_);
    ROS_INFO_ONCE("  reynolds/target_accel_limit=%f", reynolds_target_accel_limit_);
    ROS_INFO_ONCE("  reynolds/accel_limit=%f", reynolds_accel_limit_);
    ROS_INFO_ONCE("  gradient/scale_factor=%f", gradient_scale_factor_);

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
  ROS_INFO("MpcController got enable message: %d", enable_msg->data);

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

      // for logging into files
      std::stringstream tempDistance;
      tempDistance << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";
      std::stringstream tempMetrics;
      tempMetrics << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";
      std::stringstream tempState;
      tempState << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";

      // calculate swarm center, save distances to other drones
      EigenOdometry swarm_center;
      float dist_min = FLT_MAX;
      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
      {
          swarm_center = swarm_center + dronestate[i].odometry_;
          if(i != droneNumber_) // sum without own drone
          {
              float dist = dronestate[i].GetDistance(&odometry_);
              dist_min = min(dist, dist_min);
              if(dataStoring_active_) // save distance to log file for current position
                  tempDistance << dist << ",";
              ROS_INFO("MpcController %d distance to %d l=%f", droneNumber_, (int)i, dist);
          }
      }
      swarm_center.position[0] /= (float)droneCount_;
      swarm_center.position[1] /= (float)droneCount_;
      swarm_center.position[2] /= (float)droneCount_;
      ROS_INFO_ONCE("MpcController %d swarm center x=%f y=%f z=%f", droneNumber_, swarm_center.position[0], swarm_center.position[1], swarm_center.position[2]);

      if(dataStoring_active_) // save minimum distance to log file for current position
      {
         tempMetrics << dist_min << ",";

         EigenOdometry center_vector = swarm_center - odometry_;
         float dist_center = norm(center_vector); // length of vector, distance from the center_vector
         tempMetrics << dist_center << ",";

         float abs_state_velocity = sqrt(SquaredScalarVelocity(&odometry_)); // calculate length of vector
         tempState << odometry_.position[0] << "," << odometry_.position[1] << "," << odometry_.position[2] << ",";
         tempState << odometry_.velocity[0] << "," << odometry_.velocity[1] << "," << odometry_.velocity[2] << ",";
         tempState << abs_state_velocity << ",";
      }

      ROS_INFO_ONCE("MpcController got odometry message: x=%f y=%f z=%f (%d)", odometry_.position[0], odometry_.position[1], odometry_.position[2], enable_swarm_);

      // calculate neighbourhood independently from controller
      int neighbourhood_cnt = 0;
      bool neighbourhood_bool[N_DRONES_MAX];
      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
      {
          float dist = dronestate[i].GetDistance(&odometry_);

          if(dist < neighbourhood_distance_ && i != droneNumber_) // global neighbourhood
          {
              neighbourhood_cnt ++;
              neighbourhood_bool[i] = true;
          }
          else
              neighbourhood_bool[i] = false;
      }



      // ################################################################################
      if(enable_swarm_ & SWARM_DECLARATIVE_SIMPLE)
      {
          ROS_INFO_ONCE("MpcController starting swarm mode (SWARM_DECLARATIVE_SIMPLE)");

          float min_sum = FLT_MAX;
          int min_xi = 0;
          int min_yi = 0;
          int min_zi = 0;

          // calculate neighbourhood independently from potential position
          int neighbourhood_cnt = 0;
          bool neighbourhood_bool[N_DRONES_MAX];
          for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
          {
              float dist = dronestate[i].GetDistance(&odometry_);

              if(dist < neighbourhood_distance_ && i != droneNumber_) // global neighbourhood
              {
                  neighbourhood_cnt ++;
                  neighbourhood_bool[i] = true;
              }
              else
                  neighbourhood_bool[i] = false;
          }

          // float eps_move = 0.07;
          // float eps_move = 0.12;
          // float eps_move = 0.25;
          for(int xi = 0-n_move_max_; xi <= n_move_max_; xi ++) // iterate over all possible next actions in x-, y- and z-dimension
          {
              for(int yi = 0-n_move_max_; yi <= n_move_max_; yi ++)
              {
                  for(int zi = 0-n_move_max_; zi <= n_move_max_; zi ++)
                  {
                      float cohesion_sum = 0;
                      float separation_sum = 0;
                      float total_sum = 0;
                      EigenOdometry potential_pos = odometry_;
                      potential_pos.position[0] += (float)xi * eps_move_;
                      potential_pos.position[1] += (float)yi * eps_move_;
                      potential_pos.position[2] += (float)zi * eps_move_;
                      EigenOdometry potential_center = potential_pos;
                      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
                      {
                          float dist = dronestate[i].GetDistance(&potential_pos);

                          if(i == droneNumber_) // skip for own quadcopter
                            continue;
//                          cohesion_sum += dist*dist;
                          if(neighbourhood_bool[i]) // neighbourhood for separation
                          {
                            cohesion_sum += dist*dist;
                            separation_sum += 1.0/(dist*dist);

                            potential_center.position[0] += dronestate[i].odometry_.position[0];
                            potential_center.position[1] += dronestate[i].odometry_.position[1];
                            potential_center.position[2] += dronestate[i].odometry_.position[2];
                          }
                      }
                      potential_center.position[0] /= ((float)neighbourhood_cnt + 1);
                      potential_center.position[1] /= ((float)neighbourhood_cnt + 1);
                      potential_center.position[2] /= ((float)neighbourhood_cnt + 1);

                      // coehesion term
                      // total_sum = 20.0*cohesion_sum / ((float)droneCount_);
                      if(neighbourhood_cnt > 0)
                      {
                          // coehesion term
                          total_sum = mpc_cohesion_weight_ * cohesion_sum / ((float)neighbourhood_cnt);
                          // separation term
                          total_sum += mpc_separation_weight_ * separation_sum / ((float)neighbourhood_cnt);
                      }
/*
                      // target direction term
                      if(target_swarm_.position_W[2] != 0) // target point is available (z != 0)
                      {
                          float target_distance_x = fabs(target_swarm_.position_W[0] - potential_pos.position[0]);
                          float target_distance_y = fabs(target_swarm_.position_W[1] - potential_pos.position[1]);
                          float target_distance_z = fabs(target_swarm_.position_W[2] - potential_pos.position[2]);
                          total_sum += mpc_target_weight_*(target_distance_x*target_distance_x + target_distance_y*target_distance_y + target_distance_z*target_distance_z);
//                          total_sum += 25.0*(target_distance_x*target_distance_x + target_distance_y*target_distance_y + target_distance_z*target_distance_z);
                          ROS_INFO_ONCE("MpcController %d swarm target x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
                      }
*/
                      // target direction term for centroid
                      if(target_swarm_.position_W[2] != 0) // target point is available (z != 0)
                      {
                          float target_distance_x = fabs(target_swarm_.position_W[0] - potential_center.position[0]);
                          float target_distance_y = fabs(target_swarm_.position_W[1] - potential_center.position[1]);
                          float target_distance_z = fabs(target_swarm_.position_W[2] - potential_center.position[2]);
                          total_sum += mpc_target_weight_*(target_distance_x*target_distance_x + target_distance_y*target_distance_y + target_distance_z*target_distance_z);
                          ROS_INFO_ONCE("MpcController %d pot. center  x=%f y=%f z=%f (div %f)", droneNumber_, potential_center.position[0], potential_center.position[1], potential_center.position[2], ((float)neighbourhood_cnt + 1));
                          ROS_INFO_ONCE("MpcController %d swarm target x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
                      }

                      //total_sum += 0.01/norm(&prod_vector);
                      //ROS_INFO("MpcController %d coh=%f sep=%f ssl=%f", droneNumber_, cohesion_sum, separation_sum, norm(&prod_vector));

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
          new_setpoint.position_W[0] += (float)min_xi * eps_move_;
          new_setpoint.position_W[1] += (float)min_yi * eps_move_;
          new_setpoint.position_W[2] += (float)min_zi * eps_move_;
          position_controller_.SetTrajectoryPoint(new_setpoint);
    }
    // ################################################################################
    else if(enable_swarm_ & SWARM_GRADIENT)
    {
        ROS_INFO_ONCE("MpcController starting swarm mode (SWARM_GRADIENT)");

        EigenOdometry cohesion_sum;
        EigenOdometry separation_sum;
        for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
        {
            if(neighbourhood_bool[i] == false) // skip for non-neighbourhood
              continue;

            cohesion_sum = cohesion_sum + dronestate[i].odometry_;
            separation_sum = separation_sum + (dronestate[i].odometry_ - odometry_) / pow(norm_squared(dronestate[i].odometry_ - odometry_),2);

            ROS_INFO_ONCE("MpcController %d %d odo x=%f y=%f z=%f", droneNumber_, (int)i, odometry_.position[0], odometry_.position[1], odometry_.position[2]);
            ROS_INFO_ONCE("MpcController %d %d coh x=%f y=%f z=%f", droneNumber_, (int)i, cohesion_sum.position[0], cohesion_sum.position[1], cohesion_sum.position[2]);
            ROS_INFO_ONCE("MpcController %d %d sep x=%f y=%f z=%f", droneNumber_, (int)i, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2]);
        }
        cohesion_sum = (odometry_ - cohesion_sum / neighbourhood_cnt) * 2*mpc_cohesion_weight_;
        separation_sum = (separation_sum / neighbourhood_cnt) * 2*mpc_separation_weight_;

        ROS_INFO_ONCE("MpcController %d coh x=%f y=%f z=%f w=%f", droneNumber_, cohesion_sum.position[0], cohesion_sum.position[1], cohesion_sum.position[2], mpc_cohesion_weight_);
        ROS_INFO_ONCE("MpcController %d sep x=%f y=%f z=%f w=%f", droneNumber_, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2], mpc_separation_weight_);

        EigenOdometry gradient_sum = (cohesion_sum + separation_sum) * gradient_scale_factor_;
        float gradient_abs = norm(gradient_sum); // length of vector
        float dist_limit = eps_move_ * n_move_max_;
        if(gradient_abs > dist_limit) // limit distance for this controller
          dist_limit = dist_limit / gradient_abs;
        else
          dist_limit = 1;
        gradient_sum = gradient_sum * dist_limit; // rescale vector
        if(gradient_abs < eps_move_) // stepped function around 0
        {
          gradient_sum.position[0] = 0;
          gradient_sum.position[1] = 0;
          gradient_sum.position[2] = 0;
        }
        // gradient_abs = norm(gradient_sum); // length of vector

        ROS_INFO("MpcController %d sum x=%f y=%f z=%f l=%f", droneNumber_, gradient_sum.position[0], gradient_sum.position[1], gradient_sum.position[2], gradient_abs);

        mav_msgs::EigenTrajectoryPoint new_setpoint;
        new_setpoint.position_W = odometry_.position;
        new_setpoint.position_W[0] -= gradient_sum.position[0];
        new_setpoint.position_W[1] -= gradient_sum.position[1];
        new_setpoint.position_W[2] -= gradient_sum.position[2];
        position_controller_.SetTrajectoryPoint(new_setpoint);
    }
    // ################################################################################
    else if(enable_swarm_ & SWARM_REYNOLDS || enable_swarm_ & SWARM_REYNOLDS_LIMITED || enable_swarm_ & SWARM_REYNOLDS_VELOCITY)
    {
        if(enable_swarm_ & SWARM_REYNOLDS)
          ROS_INFO_ONCE("MpcController starting swarm mode (SWARM_REYNOLDS)");
        if(enable_swarm_ & SWARM_REYNOLDS_LIMITED)
          ROS_INFO_ONCE("MpcController starting swarm mode (SWARM_REYNOLDS_LIMITED)");
        if(enable_swarm_ & SWARM_REYNOLDS_VELOCITY)
          ROS_INFO_ONCE("MpcController starting swarm mode (SWARM_REYNOLDS_VELOCITY)");

        std::stringstream tempDistance;
        tempDistance << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";

        ROS_INFO_ONCE("MpcController %d vel x=%f y=%f z=%f", droneNumber_, odometry_.velocity[0], odometry_.velocity[1], odometry_.velocity[2]);
        EigenOdometry integrated_velocity;
        integrated_velocity.position[0] = odometry_.velocity[0] * weighted_delta_t_;
        integrated_velocity.position[1] = odometry_.velocity[1] * weighted_delta_t_;
        integrated_velocity.position[2] = odometry_.velocity[2] * weighted_delta_t_;
        EigenOdometry position_next = odometry_ + integrated_velocity;
        if(enable_swarm_ & SWARM_REYNOLDS_LIMITED || enable_swarm_ & SWARM_REYNOLDS_VELOCITY || weighted_delta_t_ < 0.001) // do not use next position estimate for this controller
          position_next = odometry_;

        float abs_velocity = sqrt(SquaredScalarVelocity(&odometry_));
        ROS_INFO_ONCE("MpcController %d vel=%f", droneNumber_, abs_velocity);

        int neighbourhood_cnt = 0;
        EigenOdometry velocity_sum;
        EigenOdometry cohesion_sum;
        EigenOdometry separation_sum;
        for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
        {
            float dist_cur = dronestate[i].GetDistance(&odometry_);
            // if(dataStoring_active_) // save distance to log file for current position
            //     tempDistance << dist_cur << ",";

            if(i == droneNumber_) // skip for own quadcopter
              continue;

            float dist = dronestate[i].GetDistance(&position_next);
            if(dist < neighbourhood_distance_ && dist != 0) // global neighbourhood
            {
              neighbourhood_cnt ++;

              EigenOdometry velocity_diff = DifferenceVelocity(&dronestate[i].odometry_, &position_next);
              velocity_sum = SumVelocity(&velocity_diff, &velocity_sum);

              EigenOdometry cohesion_dist = dronestate[i].odometry_ - position_next;
              cohesion_sum = cohesion_dist + cohesion_sum;

              EigenOdometry separation_dist = position_next - dronestate[i].odometry_;
              float separation_len = norm(separation_dist);
              separation_dist.position[0] /= separation_len;
              separation_dist.position[1] /= separation_len;
              separation_dist.position[2] /= separation_len;
              separation_sum = separation_dist + separation_sum;

              ROS_INFO_ONCE("MpcController %d (i=%d) vel x=%f y=%f z=%f", droneNumber_, (int)i, velocity_sum.velocity[0], velocity_sum.velocity[1], velocity_sum.velocity[2]);
              ROS_INFO_ONCE("MpcController %d (i=%d) accel x=%f y=%f z=%f", droneNumber_, (int)i, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2]);
            }
        }

        EigenOdometry velocity_accel;
        EigenOdometry cohesion_accel;
        EigenOdometry separation_accel;
        EigenOdometry target_accel;

        if(neighbourhood_cnt != 0)
        {
          velocity_accel.position[0] = reynolds_velocity_factor_ * (velocity_sum.velocity[0] / (float)neighbourhood_cnt);
          velocity_accel.position[1] = reynolds_velocity_factor_ * (velocity_sum.velocity[1] / (float)neighbourhood_cnt);
          velocity_accel.position[2] = reynolds_velocity_factor_ * (velocity_sum.velocity[2] / (float)neighbourhood_cnt);
          cohesion_accel.position[0] = reynolds_cohesion_factor_ * (cohesion_sum.position[0] / (float)neighbourhood_cnt);
          cohesion_accel.position[1] = reynolds_cohesion_factor_ * (cohesion_sum.position[1] / (float)neighbourhood_cnt);
          cohesion_accel.position[2] = reynolds_cohesion_factor_ * (cohesion_sum.position[2] / (float)neighbourhood_cnt);
          separation_accel.position[0] = reynolds_separation_factor_ * (separation_sum.position[0] / (float)neighbourhood_cnt);
          separation_accel.position[1] = reynolds_separation_factor_ * (separation_sum.position[1] / (float)neighbourhood_cnt);
          separation_accel.position[2] = reynolds_separation_factor_ * (separation_sum.position[2] / (float)neighbourhood_cnt);
        }
        target_accel.position[0] = reynolds_target_factor_ * (target_swarm_.position_W[0] - position_next.position[0]);
        target_accel.position[1] = reynolds_target_factor_ * (target_swarm_.position_W[1] - position_next.position[1]);
        target_accel.position[2] = reynolds_target_factor_ * (target_swarm_.position_W[2] - position_next.position[2]);
        float abs_target_accel = norm(target_accel); // length of vector
        float target_accel_limit = reynolds_target_accel_limit_; // * global_factor;
        if(abs_target_accel > target_accel_limit) // if limit exceeded
          target_accel_limit = target_accel_limit / abs_target_accel;
        else
          target_accel_limit = 1;
        target_accel.position[0] *= target_accel_limit;
        target_accel.position[1] *= target_accel_limit;
        target_accel.position[2] *= target_accel_limit;

        EigenOdometry accel = cohesion_accel + separation_accel;
        if(enable_swarm_ & SWARM_REYNOLDS_VELOCITY)
          accel = accel + velocity_accel;
        accel = accel + target_accel;

        float abs_accel = norm(accel); // length of vector
        if(reynolds_accel_limit_ > 0.001) // limit acceleration for this controller
        {
          float accel_limit = reynolds_accel_limit_; // * global_factor;
          if(abs_accel > accel_limit) // if limit exceeded
            accel_limit = accel_limit / abs_accel;
          else
            accel_limit = 1;
          accel.position[0] *= accel_limit;
          accel.position[1] *= accel_limit;
          accel.position[2] *= accel_limit;
        }
        abs_accel = norm(accel); // recalculate length of vector
        if(abs_accel < 0.1)
        {
          accel.position[0] = 0;
          accel.position[1] = 0;
          accel.position[2] = 0;
        }

        ROS_INFO_ONCE("MpcController %d (|H|=%d) cohesion_accel x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, cohesion_accel.position[0], cohesion_accel.position[1], cohesion_accel.position[2]);
        ROS_INFO_ONCE("MpcController %d (|H|=%d) separation_accel x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, separation_accel.position[0], separation_accel.position[1], separation_accel.position[2]);
        ROS_INFO_ONCE("MpcController %d target_accel=%f x=%f y=%f z=%f", droneNumber_, abs_target_accel, target_accel.position[0], target_accel.position[1], target_accel.position[2]);
        ROS_INFO_ONCE("MpcController %d (|H|=%d) accel=%f x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, abs_accel, accel.position[0], accel.position[1], accel.position[2]);

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
    // ################################################################################
    else if(enable_swarm_ & SWARM_REYNOLDS_VELOCITY && false)
    {
        ROS_INFO_ONCE("MpcController starting OLD! swarm mode (SWARM_REYNOLDS_VELOCITY)");

        ROS_INFO_ONCE("MpcController %d vel x=%f y=%f z=%f", droneNumber_, odometry_.velocity[0], odometry_.velocity[1], odometry_.velocity[2]);

        for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
        {
            if(i == droneNumber_) // skip for own quadcopter
              continue;

            ROS_INFO("MpcController %d (i=%d) vel x=%f y=%f z=%f", droneNumber_, (int)i, dronestate[i].odometry_.velocity[0], dronestate[i].odometry_.velocity[1], dronestate[i].odometry_.velocity[2]);
        }

        mav_msgs::EigenTrajectoryPoint new_setpoint;
        new_setpoint.position_W = odometry_.position;
        new_setpoint.position_W[0] = 0;
        new_setpoint.position_W[1] = 0;
        new_setpoint.position_W[2] = 1;
        position_controller_.SetTrajectoryPoint(new_setpoint);

    }



      if(dataStoring_active_) // save data for log files
      {
          tempDistance << "\n";
          listDistance_.push_back(tempDistance.str());
          tempMetrics << "\n";
          listMetrics_.push_back(tempMetrics.str());
          tempState << "\n";
          listState_.push_back(tempState.str());
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

EigenOdometry operator-(const EigenOdometry& a, const EigenOdometry& b)
{
    EigenOdometry r;
    r.position[0] = a.position[0] - b.position[0];
    r.position[1] = a.position[1] - b.position[1];
    r.position[2] = a.position[2] - b.position[2];
    return r;
}

EigenOdometry DifferenceVelocity(EigenOdometry* a, EigenOdometry* b)
{
    EigenOdometry r;
    r.velocity[0] = a->velocity[0] - b->velocity[0];
    r.velocity[1] = a->velocity[1] - b->velocity[1];
    r.velocity[2] = a->velocity[2] - b->velocity[2];
    return r;
}

EigenOdometry operator+(const EigenOdometry& a, const EigenOdometry& b)
{
    EigenOdometry r;
    r.position[0] = a.position[0] + b.position[0];
    r.position[1] = a.position[1] + b.position[1];
    r.position[2] = a.position[2] + b.position[2];
    return r;
}

EigenOdometry SumVelocity(EigenOdometry* a, EigenOdometry* b)
{
    EigenOdometry r;
    r.velocity[0] = a->velocity[0] + b->velocity[0];
    r.velocity[1] = a->velocity[1] + b->velocity[1];
    r.velocity[2] = a->velocity[2] + b->velocity[2];
    return r;
}

EigenOdometry operator*(const EigenOdometry& a, const float& b)
{
    EigenOdometry r;
    r.position[0] = a.position[0] * b;
    r.position[1] = a.position[1] * b;
    r.position[2] = a.position[2] * b;
    return r;
}

EigenOdometry operator/(const EigenOdometry& a, const float& b)
{
    EigenOdometry r;
    r.position[0] = a.position[0] / b;
    r.position[1] = a.position[1] / b;
    r.position[2] = a.position[2] / b;
    return r;
}


float norm_squared(const EigenOdometry& a)
{
    return (float) (a.position[0]*a.position[0] + a.position[1]*a.position[1] + a.position[2]*a.position[2]);
}

float norm(const EigenOdometry& a)
{
  return (float) sqrt(norm_squared(a));
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
