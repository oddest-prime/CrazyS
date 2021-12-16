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

#include "position_controller_light.h"

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

PositionControllerLight::PositionControllerLight() {

    ROS_INFO_ONCE("Started PositionControllerLight");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

    // Topics subscribe
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PositionControllerLight::OdometryCallback, this);
    setpoint_sub_ = nh.subscribe("set_point", 1, &PositionControllerLight::SetpointCallback, this);

    // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // To publish the current pose
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("lps_pose", 1);

    // To publish the current pose for gazebo visual
    gazebo_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

PositionControllerLight::~PositionControllerLight(){}

//The callback saves data come from simulation into csv files
void PositionControllerLight::CallbackSaveData(const ros::TimerEvent& event){

      if(!dataStoring_active_){
         return;
      }

      ofstream fileDistance;
      ofstream fileMetrics;
      ofstream fileState;

      ROS_INFO("CallbackSavaData PositionControllerLight. droneNumber: %d", droneNumber_);

      fileDistance.open(std::string("/tmp/log_output/PosDistance") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      fileMetrics.open(std::string("/tmp/log_output/PosMetrics") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      fileState.open(std::string("/tmp/log_output/PosState") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);

      // Saving distances from every to every drone in a file
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

void PositionControllerLight::InitializeParams() {
  ros::NodeHandle pnh("~");

  ROS_INFO_ONCE("[PositionControllerLight] InitializeParams");

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
    GetRosParameter(pnh, "mpc1/mpc_obstacle_weight", (float)1.0, &mpc_obstacle_weight_);

    GetRosParameter(pnh, "reynolds/weighted_delta_t", (float)0.5, &weighted_delta_t_);
    GetRosParameter(pnh, "reynolds/global_factor", (float)0.5, &reynolds_global_factor_);
    GetRosParameter(pnh, "reynolds/velocity_factor", (float)0.5, &reynolds_velocity_factor_);
    GetRosParameter(pnh, "reynolds/cohesion_factor", (float)0.5, &reynolds_cohesion_factor_);
    GetRosParameter(pnh, "reynolds/separation_factor", (float)0.5, &reynolds_separation_factor_);
    GetRosParameter(pnh, "reynolds/target_factor", (float)0.5, &reynolds_target_factor_);
    GetRosParameter(pnh, "reynolds/target_accel_limit", (float)0.5, &reynolds_target_accel_limit_);
    GetRosParameter(pnh, "reynolds/accel_limit", (float)0.5, &reynolds_accel_limit_);

    GetRosParameter(pnh, "gradient/scale_factor", (float)0.01, &gradient_scale_factor_);
    GetRosParameter(pnh, "inner/controller", (int)'a', &inner_controller_);

    reynolds_velocity_factor_ *= reynolds_global_factor_;
    reynolds_cohesion_factor_ *= reynolds_global_factor_;
    reynolds_separation_factor_ *= reynolds_global_factor_;
    reynolds_target_factor_ *= reynolds_global_factor_;
    reynolds_target_accel_limit_ *= reynolds_global_factor_;
    reynolds_accel_limit_ *= reynolds_global_factor_;

    ROS_INFO_ONCE("[PositionControllerLight] GetRosParameter values:");
    ROS_INFO_ONCE("  inner/controller=%d", inner_controller_);
    position_controller_.inner_controller_ = inner_controller_;

    //Reading the parameters come from the launch file
    std::string dataStoringActive;
    int droneNumber;
    int droneCount;
    double dataStoringTime;
    std::string user;

    neighbourhood_cnt_ = -1;

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
        if(dataStoringActive == "yes")
            position_controller_.dataStoring_active_ = true;

        dataStoring_active_ = true;
        if(dataStoringActive == "yes" || dataStoringActive == "distance")
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
    timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &PositionControllerLight::CallbackSaveData, this, false, true);

    position_controller_.SetLaunchFileParameters();
}

void PositionControllerLight::Publish(){
}

void PositionControllerLight::SetpointCallback(const geometry_msgs::PoseStampedConstPtr& setpoint_msg) {
  ROS_INFO_ONCE("PositionControllerLight %d Setpoint x=%f y=%f z=%f", droneNumber_, setpoint_msg->pose.position.x, setpoint_msg->pose.position.y, setpoint_msg->pose.position.z);
  mav_msgs::EigenTrajectoryPoint new_setpoint;
  new_setpoint.position_W[0] = setpoint_msg->pose.position.x;
  new_setpoint.position_W[1] = setpoint_msg->pose.position.y;
  new_setpoint.position_W[2] = setpoint_msg->pose.position.z;
  position_controller_.SetTrajectoryPoint(new_setpoint);

  waypointHasBeenPublished_ = true;
}

void PositionControllerLight::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionControllerLight got first odometry message.");

    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = odometry_msg->header.stamp;
    current_pose.pose = odometry_msg->pose.pose;
    pose_pub_.publish(current_pose);

    // debug, testing
    geometry_msgs::Point pr2_position;
    pr2_position.x = odometry_msg->pose.pose.position.x;
    pr2_position.y = odometry_msg->pose.pose.position.y;
    pr2_position.z = odometry_msg->pose.pose.position.z;
    geometry_msgs::Quaternion pr2_orientation;
    pr2_orientation.x = 0.0;
    pr2_orientation.y = 0.0;
    pr2_orientation.z = 0.0;
    pr2_orientation.w = 1.0;
    geometry_msgs::Pose pr2_pose;
    pr2_pose.position = pr2_position;
    pr2_pose.orientation = pr2_orientation;
    gazebo_msgs::ModelState pr2_modelstate;
    pr2_modelstate.model_name = (std::string) "marker0_model";
    pr2_modelstate.pose = pr2_pose;
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = pr2_modelstate;
    if(!gazebo_client_.call(srv))
      ROS_ERROR("Failed to move marker! Error msg:%s",srv.response.status_message.c_str());

    if(waypointHasBeenPublished_){

      //This functions allows us to put the odometry message into the odometry variable--> _position,
      //_orientation,_velocit_body,_angular_velocity
      eigenOdometryFromMsg(odometry_msg, &odometry_);
      position_controller_.SetOdometryWithoutStateEstimator(odometry_);

      ROS_INFO_ONCE("PositionControllerLight got odometry message: x=%f y=%f z=%f (%d)", odometry_.position[0], odometry_.position[1], odometry_.position[2], enable_swarm_);

      // for logging into files
      std::stringstream tempDistance;
      tempDistance << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";
      std::stringstream tempMetrics;
      tempMetrics << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";
      std::stringstream tempState;
      tempState << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";

      if(dataStoring_active_) // save minimum distance to log file for current position
      {
         float abs_state_velocity = sqrt(SquaredScalarVelocity(&odometry_)); // calculate length of vector
         tempState << odometry_.position[0] << "," << odometry_.position[1] << "," << odometry_.position[2] << ",";
         tempState << odometry_.velocity[0] << "," << odometry_.velocity[1] << "," << odometry_.velocity[2] << ",";
         tempState << abs_state_velocity << ",";
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

float SquaredScalarVelocity(EigenOdometry* a)
{
    return (float) (a->velocity[0]*a->velocity[0] + a->velocity[1]*a->velocity[1] + a->velocity[2]*a->velocity[2]);
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_light_node");

    ros::NodeHandle nh2;

    rotors_control::PositionControllerLight position_controller_node;

    ros::spin();

    return 0;
}
