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

#include "playback_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>
#include <fstream>
#include <iostream>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

PlaybackNode::PlaybackNode() {

    ROS_INFO_ONCE("Started PlaybackNode controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

    std::vector<SetPointWithTime> setpoints;

    std::string attitude_file;
    if (pnh_node.getParam("attitude_file", attitude_file))
      ROS_INFO("Got param 'attitude_file': %s", attitude_file.c_str());
    else
    {
      ROS_ERROR("Failed to get param 'attitude_file'");
      return;
    }

    std::ifstream wp_file(attitude_file.c_str());
    if (wp_file.is_open()) {
      double t, z, pitch, roll, yaw;
      while (wp_file >> t >> z >> pitch >> roll >> yaw) { // Only read complete setpoints.
        setpoints.push_back(SetPointWithTime(t, z, pitch, roll, yaw));
      }
      wp_file.close();
      ROS_INFO("Read %d setpoints.", (int) setpoints.size());
    } else {
      ROS_ERROR_STREAM("Unable to open setpoints file: " << attitude_file);
      return;
    }

    // Topics subscribe
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &PlaybackNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
      &PlaybackNode::OdometryCallback, this);

    // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // The subscription to the IMU topic is made only if it is available, when simulating the Crazyflie dynamics considering the also IMU values
    if(pnh_node.getParam("enable_state_estimator", enable_state_estimator_)){
      ROS_INFO("Got param 'enable_state_estimator': %d", enable_state_estimator_);
    }

}

PlaybackNode::~PlaybackNode(){}

void PlaybackNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
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

  // We can trigger the first command immediately.
  //position_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("Playback got first MultiDOFJointTrajectory message.");
  }

  ROS_DEBUG("Playback got MultiDOFJointTrajectory message: x=%f, y=%f, z=%f, yaw=%f", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2], eigen_reference.getYaw());

}

void PlaybackNode::InitializeParams() {
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
    bool dataStoringActive;
    int droneNumber;
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
     }
     else
        ROS_ERROR("Failed to get param 'droneNumber'");

    if (pnh.getParam("csvFilesStoring", dataStoringActive)){
    ROS_INFO("Got param 'csvFilesStoring': %d", dataStoringActive);
    position_controller_.dataStoring_active_ = dataStoringActive;
    }
    else
       ROS_ERROR("Failed to get param 'csvFilesStoring'");

    if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
    ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
    position_controller_.dataStoringTime_ = dataStoringTime;
    }
    else
       ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

    position_controller_.SetLaunchFileParameters();

   }

}

void PlaybackNode::Publish(){
}

void PlaybackNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("Playback got first imu message.");

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

void PlaybackNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("Playback got first odometry message.");

    position_controller_.SetSetPoint(1, 0, 0, 0);

    //This functions allows us to put the odometry message into the odometry variable--> _position,
    //_orientation,_velocit_body,_angular_velocity
    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    position_controller_.SetOdometryWithoutStateEstimator(odometry);

    ROS_DEBUG("Playback got odometry message: x=%f y=%f z=%f", odometry.position[0], odometry.position[1], odometry.position[2]);

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

// IMU messages are sent to the controller with a frequency of 500Hz. In other words, with a sampling time of 0.002 seconds
void PlaybackNode::CallbackIMUUpdate(const ros::TimerEvent& event){

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


}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node_with_stateEstimator");

    ros::NodeHandle nh2;

    rotors_control::PlaybackNode position_controller_node;

    ros::spin();

    return 0;
}
