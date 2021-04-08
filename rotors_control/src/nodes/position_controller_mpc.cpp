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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <time.h>
#include <chrono>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */
#define N_DRONES_MAX  20          /* maximum number of drones */

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

  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

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
    bool dataStoringActive;
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

void PositionControllerMpc::EnableCallback(const std_msgs::BoolConstPtr& bool_msg) {
  ROS_INFO_ONCE("MpcController got first enable message.");

  enable_swarm_ = bool_msg->data;
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

      if(enable_swarm_)
      {
          ROS_INFO_ONCE("MpcController starting swarm mode");

          float min_sum = FLT_MAX;
          int min_xi = 0;
          int min_yi = 0;
          int min_zi = 0;
          for(int xi = -1; xi <= 1; xi ++)
          {
              for(int yi = -1; yi <= 1; yi ++)
              {
                  for(int zi = -1; zi <= 1; zi ++)
                  {
                      float cohesion_sum = 0;
                      float separation_sum = 0;
                      float total_sum = 0;
                      for (size_t i = 0; i < droneCount_; i++)
                      {
                          if(i == droneNumber_)
                            continue;
                          EigenOdometry potential_pos = odometry_;
                          potential_pos.position[0] += (float)xi * 0.1;
                          potential_pos.position[1] += (float)yi * 0.1;
                          potential_pos.position[2] += (float)zi * 0.1;
                          float dist = dronestate[i].GetDistance(&potential_pos);
                          cohesion_sum += dist*dist;
                          separation_sum += 1.0/(dist*dist);
                      }
                      total_sum = 20*cohesion_sum + separation_sum;
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
          ROS_INFO("MpcController %d swarm direction xi=%d yi=%d zi=%d tsum=%f", droneNumber_, min_xi, min_yi, min_zi, min_sum);
          mav_msgs::EigenTrajectoryPoint new_setpoint;
          new_setpoint.position_W = odometry_.position;
          new_setpoint.position_W[0] += (float)min_xi * 0.1;
          new_setpoint.position_W[1] += (float)min_yi * 0.1;
          new_setpoint.position_W[2] += (float)min_zi * 0.1;
          position_controller_.SetTrajectoryPoint(new_setpoint);
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



}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node_with_stateEstimator");

    ros::NodeHandle nh2;

    rotors_control::PositionControllerMpc position_controller_node;

    ros::spin();

    return 0;
}
