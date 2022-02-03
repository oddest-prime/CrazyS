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

#include "swarm_controller.h"

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
#include <limits>
#include <random>

#include <sys/stat.h>
#include <sys/types.h>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

SwarmController::SwarmController() {

    ROS_INFO_ONCE("Started SwarmController");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");
    InitializeParams();

    // Topics subscribe
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &SwarmController::MultiDofJointTrajectoryCallback, this);

    pose_sub_ = nh.subscribe("lps_pose", 1, &SwarmController::PoseCallback, this);
    enable_sub_ = nh.subscribe("enable", 1, &SwarmController::EnableCallback, this);
    keyboard_sub_ = nh.subscribe("/key", 1, &SwarmController::KeyboardCallback, this);

    ros::NodeHandle nhq[N_DRONES_MAX];
    for (size_t i = 0; i < droneCount_; i++)
    {
      nhq[i] = ros::NodeHandle(std::string("/crazyflie2_") + std::to_string(i));

      dronestate[i].SetId(droneNumber_, i, position_noise_);
      ROS_INFO("SwarmController: Setup subscriber %s/lps_pose.", nhq[i].getNamespace().c_str());
      pose_other_sub_[i] = nhq[i].subscribe("lps_pose", 1, &DroneStateWithTime::PoseCallback, &dronestate[i]);
    }

    // To publish the set-point
    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("set_point", 1);

    rand_cnt_ = 0;
    rand_x_ = 0;
    rand_y_ = 0;
    rand_z_ = 0;
}

SwarmController::~SwarmController(){}

void SwarmController::KeyboardCallback(const std_msgs::Int32Ptr& msg) {
  ROS_INFO("SwarmController: Got data in keyboard_callback: %d", msg->data);
  if(msg->data == 'd') // save data
    FileSaveData();
  if(msg->data == 'l') // save when landing
    FileSaveData();
}

//The callback saves data into csv files
void SwarmController::CallbackSaveData(const ros::TimerEvent& event){
  ROS_INFO("SwarmController CallbackSavaData. droneNumber: %d", droneNumber_);
  FileSaveData();
}

void SwarmController::FileSaveData(void){

      if(!dataStoring_active_){
         return;
      }

      ofstream fileDistance;
      ofstream fileMetrics;
      ofstream fileState;

      ROS_INFO("SwarmController FileSaveData. droneNumber: %d", droneNumber_);

      if(mkdir("/tmp/log_output/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
          if(errno != EEXIST)
             ROS_ERROR("Cannot create directory /tmp/log_output/");

      fileDistance.open(std::string("/tmp/log_output/Distance") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      fileMetrics.open(std::string("/tmp/log_output/Metrics") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      fileState.open(std::string("/tmp/log_output/State") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);

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
      // dataStoring_active_ = false;
}

void SwarmController::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
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

  // set target point for hovering or swarm target if in swarm mode
  target_swarm_ = eigen_reference;

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("SwarmController got first MultiDOFJointTrajectory message.");
  }

  ROS_DEBUG("SwarmController got MultiDOFJointTrajectory message: x=%f, y=%f, z=%f, yaw=%f", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2], eigen_reference.getYaw());

}

void SwarmController::InitializeParams() {
  ros::NodeHandle pnh("~");

  ROS_INFO_ONCE("[SwarmController] InitializeParams");

  if(!enable_mellinger_controller_ && !enable_internal_model_controller_){

    GetRosParameter(pnh, "swarm/position_noise", (float)0.0, &position_noise_);
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

    reynolds_velocity_factor_ *= reynolds_global_factor_;
    reynolds_cohesion_factor_ *= reynolds_global_factor_;
    reynolds_separation_factor_ *= reynolds_global_factor_;
    reynolds_target_factor_ *= reynolds_global_factor_;
    reynolds_target_accel_limit_ *= reynolds_global_factor_;
    reynolds_accel_limit_ *= reynolds_global_factor_;

    ROS_INFO_ONCE("[Swarm Controller] GetRosParameter values:");
    ROS_INFO_ONCE("  swarm/position_noise=%f", position_noise_);
    ROS_INFO_ONCE("  swarm/neighbourhood_distance=%f", neighbourhood_distance_);
    ROS_INFO_ONCE("  mpc1/eps_move=%f", eps_move_);
    ROS_INFO_ONCE("  mpc1/n_move_max=%d", n_move_max_);
    ROS_INFO_ONCE("  mpc1/mpc_cohesion_weight=%f", mpc_cohesion_weight_);
    ROS_INFO_ONCE("  mpc1/mpc_separation_weight=%f", mpc_separation_weight_);
    ROS_INFO_ONCE("  mpc1/mpc_target_weight=%f", mpc_target_weight_);
    ROS_INFO_ONCE("  mpc1/mpc_obstacle_weight=%f", mpc_obstacle_weight_);
    ROS_INFO_ONCE("  reynolds/weighted_delta_t=%f", weighted_delta_t_);
    ROS_INFO_ONCE("  reynolds/global_factor=%f", reynolds_global_factor_);
    ROS_INFO_ONCE("  reynolds/velocity_factor=%f", reynolds_velocity_factor_);
    ROS_INFO_ONCE("  reynolds/cohesion_factor=%f", reynolds_cohesion_factor_);
    ROS_INFO_ONCE("  reynolds/separation_factor=%f", reynolds_separation_factor_);
    ROS_INFO_ONCE("  reynolds/target_factor=%f", reynolds_target_factor_);
    ROS_INFO_ONCE("  reynolds/target_accel_limit=%f", reynolds_target_accel_limit_);
    ROS_INFO_ONCE("  reynolds/accel_limit=%f", reynolds_accel_limit_);
    ROS_INFO_ONCE("  gradient/scale_factor=%f", gradient_scale_factor_);

    for (size_t i = 0; i < droneCount_; i++)
      dronestate[i].SetId(droneNumber_, i, position_noise_);

    //Reading the parameters come from the launch file
    std::string dataStoringActive;
    int droneNumber;
    int droneCount;
    int obstacleScenario;
    double dataStoringTime;
    std::string user;

    neighbourhood_cnt_ = -1;

    if (pnh.getParam("obstacleScenario", obstacleScenario)){
        ROS_INFO("Got param 'obstacleScenario': %d", obstacleScenario);
        obstacleScenario_ = obstacleScenario;
    }
    else
       ROS_ERROR("Failed to get param 'obstacleScenario'");

     if (pnh.getParam("droneNumber", droneNumber)){
         ROS_INFO("Got param 'droneNumber': %d", droneNumber);
         // position_controller_.droneNumber_ = droneNumber;
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

        dataStoring_active_ = true;
        if(dataStoringActive == "yes" || dataStoringActive == "distance")
            dataStoring_active_ = true;
    }
    else {
       ROS_ERROR("Failed to get param 'csvFilesStoring'");
       dataStoring_active_ = true;
    }

    if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
        ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
    }
    else {
       ROS_ERROR("Failed to get param 'csvFilesStoringTime'");
       dataStoringTime = 9999;
    }

    ros::NodeHandle nh;
    timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &SwarmController::CallbackSaveData, this, false, true);

   }

}

void SwarmController::Publish(){
}

void SwarmController::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("SwarmController got first imu message.");

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

void SwarmController::EnableCallback(const std_msgs::Int32ConstPtr& enable_msg) {
  ROS_INFO("SwarmController got enable message: %d", enable_msg->data);

  enable_swarm_ = enable_msg->data;
}

void SwarmController::PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
    ROS_INFO_ONCE("SwarmController got PoseStamped message.");

    // received message containing own position information
    // odometry_gt_ = EigenOdometry();
    odometry_gt_.timeStampSec = pose_msg->header.stamp.sec;
    odometry_gt_.timeStampNsec = pose_msg->header.stamp.nsec;
    odometry_gt_.position[0] = pose_msg->pose.position.x;
    odometry_gt_.position[1] = pose_msg->pose.position.y;
    odometry_gt_.position[2] = pose_msg->pose.position.z;
    odometry_ = odometry_gt_;

    // gaussian random number generator
    std::normal_distribution<float> dist(0.0, position_noise_);
    rand_cnt_++;
    if(rand_cnt_ > 10) // reduce frequency of noise
    {
      rand_cnt_ = 0;
      rand_x_ = max(-5*position_noise_, min((float)dist(generator), 5*position_noise_));
      rand_y_ = max(-5*position_noise_, min((float)dist(generator), 5*position_noise_));
      rand_z_ = max(-5*position_noise_, min((float)dist(generator), 5*position_noise_));
      ROS_INFO_ONCE("SwarmController %d random noise (stddev=%f) x=%f y=%f z=%f", droneNumber_, position_noise_, rand_x_, rand_y_, rand_z_);
    }
    odometry_.position[0] = odometry_gt_.position[0] + rand_x_;
    odometry_.position[1] = odometry_gt_.position[1] + rand_y_;
    odometry_.position[2] = odometry_gt_.position[2] + rand_z_;

    // setpoint for message to be sent to low-level controller
    geometry_msgs::PoseStamped set_point;
    set_point.header.stamp = pose_msg->header.stamp;

    for (size_t i = 0; i < droneCount_; i++)
        dronestate[i].UpdateDistance(&odometry_, &odometry_gt_);

    // obstacle definition
    int obstacle_count = 0;
    EigenOdometry obstacle_position[10];
    if(obstacleScenario_ == 1)
    {
      obstacle_count = 1;
      obstacle_position[0].position[0] = 2.5;
      obstacle_position[0].position[1] = 2.5;
    }
    else if(obstacleScenario_ == 2)
    {
      obstacle_count = 3;
      obstacle_position[0].position[0] = 2.0;
      obstacle_position[0].position[1] = 1.5;
      obstacle_position[1].position[0] = 3.5;
      obstacle_position[1].position[1] = 3.5;
      obstacle_position[2].position[0] = 0.0;
      obstacle_position[2].position[1] = 3.0;
    }
    obstacle_position[0].position[2] = odometry_gt_.position[2]; // todo, remove workaround for infinite z obstacles
    obstacle_position[1].position[2] = odometry_gt_.position[2]; // todo, remove workaround for infinite z obstacles
    obstacle_position[2].position[2] = odometry_gt_.position[2]; // todo, remove workaround for infinite z obstacles
    float obstacle_radius = 0.15;

    float obstacle_dist_min_gt = FLT_MAX;
    for(int i = 0; i < obstacle_count; i++) // iterate over all obstacles
    {
      float obstacle_dist_gt = norm(odometry_gt_ - obstacle_position[i]);
      obstacle_dist_min_gt = min(obstacle_dist_min_gt, obstacle_dist_gt);
    }
    obstacle_position[0].position[2] = odometry_.position[2]; // todo, remove workaround for infinite z obstacles
    obstacle_position[1].position[2] = odometry_.position[2]; // todo, remove workaround for infinite z obstacles
    obstacle_position[2].position[2] = odometry_.position[2]; // todo, remove workaround for infinite z obstacles

    // for logging into files
    std::stringstream tempDistance;
    tempDistance.precision(24);
    tempDistance << odometry_gt_.timeStampSec << "," << odometry_gt_.timeStampNsec << "," << enable_swarm_ << ",";
    std::stringstream tempMetrics;
    tempMetrics.precision(24);
    tempMetrics << odometry_gt_.timeStampSec << "," << odometry_gt_.timeStampNsec << "," << enable_swarm_ << ",";
    std::stringstream tempState;
    tempState.precision(24);
    tempState << odometry_gt_.timeStampSec << "," << odometry_gt_.timeStampNsec << "," << enable_swarm_ << ",";

    // calculate swarm center, save distances to other drones
    EigenOdometry swarm_center;
    EigenOdometry swarm_center_gt;
    float dist_min_gt = FLT_MAX;
    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
    {
        swarm_center = swarm_center + dronestate[i].odometry_;
        swarm_center_gt = swarm_center_gt + dronestate[i].odometry_gt_;
        if(i != droneNumber_) // sum without own drone
        {
            float dist_gt = dronestate[i].GetDistance_gt(&odometry_gt_);
            dist_min_gt = min(dist_gt, dist_min_gt);
            if(dataStoring_active_) // save distance to log file for current position
                tempDistance << dist_gt << ",";
            ROS_INFO_ONCE("SwarmController %d distance_gt to %d l=%f", droneNumber_, (int)i, dist_gt);
        }
    }
    swarm_center = swarm_center / (float)droneCount_;
    swarm_center_gt = swarm_center_gt / (float)droneCount_;
    ROS_INFO_ONCE("SwarmController %d swarm center_gt x=%f y=%f z=%f", droneNumber_, swarm_center_gt.position[0], swarm_center_gt.position[1], swarm_center_gt.position[2]);

    if(dataStoring_active_) // save minimum distance to log file for current position
    {
       tempMetrics << dist_min_gt << ",";

       EigenOdometry center_vector_gt = swarm_center_gt - odometry_gt_;
       float dist_center_gt = norm(center_vector_gt); // length of vector, distance from the center_vector
       tempMetrics << dist_center_gt << ",";

       tempMetrics << obstacle_dist_min_gt << ",";

       //float abs_state_velocity = sqrt(SquaredScalarVelocity(&odometry_gt_)); // calculate length of vector
       tempState << odometry_gt_.position[0] << "," << odometry_gt_.position[1] << "," << odometry_gt_.position[2] << ",";
       // tempState << odometry_gt_.velocity[0] << "," << odometry_gt_.velocity[1] << "," << odometry_gt_.velocity[2] << ",";
       // tempState << abs_state_velocity << ",";
    }

    ROS_INFO_ONCE("SwarmController got odometry message: x=%f y=%f z=%f (%d)", odometry_.position[0], odometry_.position[1], odometry_.position[2], enable_swarm_);

    // calculate neighbourhood independently from controller
    int neighbourhood_cnt = 0;
    bool neighbourhood_bool[N_DRONES_MAX];
    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
    {
        float dist = dronestate[i].GetDistance(&odometry_);

        if(i == droneNumber_) // skip own quadcopter in any case
        {
            neighbourhood_bool[i] = false;
        }
        else if(dist < neighbourhood_distance_) // neighbourhood based on distance parameter
        {
            neighbourhood_cnt ++;
            neighbourhood_bool[i] = true;
        }
        else
        {
            ROS_INFO_ONCE("SwarmController %d no-neighbour=%d dist=%f", droneNumber_, (int)i, dist);
            neighbourhood_bool[i] = false;
        }
    }

    if(neighbourhood_cnt_ != neighbourhood_cnt)
    {
        neighbourhood_cnt_ = neighbourhood_cnt;
        ROS_INFO_ONCE("SwarmController %d droneCount=%d neighbourhood_cnt=%d", droneNumber_, droneCount_, neighbourhood_cnt);
    }

    tempMetrics << droneCount_ << ",";
    tempMetrics << neighbourhood_cnt << ",";


    // ################################################################################
    if(enable_swarm_ == SWARM_DISABLED) // set target point if not in swarm mode
    {
        ROS_INFO_ONCE("SwarmController %d swarm disabled x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
        set_point.pose.position.x = target_swarm_.position_W[0];
        set_point.pose.position.y = target_swarm_.position_W[1];
        set_point.pose.position.z = target_swarm_.position_W[2];
    }
    else if(enable_swarm_ == SWARM_LANDING) // set keep target point and set small z if in landing mode
    {
        ROS_INFO_ONCE("SwarmController %d landing x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
        set_point.pose.position.x = odometry_.position[0];
        set_point.pose.position.y = odometry_.position[1];
        set_point.pose.position.z = max(0.0, min(odometry_.position[2] - 0.05, 0.1));
    }
    else if(enable_swarm_ & SWARM_DECLARATIVE_SIMPLE)
    {
        ROS_INFO_ONCE("SwarmController starting swarm mode (SWARM_DECLARATIVE_SIMPLE)");

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
        float min_sum = FLT_MAX;
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
                        ROS_INFO_ONCE("SwarmController %d swarm target x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
                    }
*/
                    // target direction term for centroid
                    if(target_swarm_.position_W[2] != 0) // target point is available (z != 0)
                    {
                        float target_distance_x = fabs(target_swarm_.position_W[0] - potential_center.position[0]);
                        float target_distance_y = fabs(target_swarm_.position_W[1] - potential_center.position[1]);
                        float target_distance_z = fabs(target_swarm_.position_W[2] - potential_center.position[2]);
                        total_sum += mpc_target_weight_*(target_distance_x*target_distance_x + target_distance_y*target_distance_y + target_distance_z*target_distance_z);
                        ROS_INFO_ONCE("SwarmController %d pot. center  x=%f y=%f z=%f (div %f)", droneNumber_, potential_center.position[0], potential_center.position[1], potential_center.position[2], ((float)neighbourhood_cnt + 1));
                        ROS_INFO_ONCE("SwarmController %d swarm target x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
                    }

                    //total_sum += 0.01/norm(&prod_vector);
                    //ROS_INFO("SwarmController %d coh=%f sep=%f ssl=%f", droneNumber_, cohesion_sum, separation_sum, norm(&prod_vector));

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
        ROS_INFO_ONCE("SwarmController %d swarm direction xi=%d yi=%d zi=%d tsum=%f", droneNumber_, min_xi, min_yi, min_zi, min_sum);
        set_point.pose.position.x = odometry_.position[0] + (float)min_xi * eps_move_;
        set_point.pose.position.y = odometry_.position[1] + (float)min_xi * eps_move_;
        set_point.pose.position.z = odometry_.position[2] + (float)min_xi * eps_move_;

/*
        mav_msgs::EigenTrajectoryPoint new_setpoint;
        new_setpoint.position_W = odometry_.position;
        new_setpoint.position_W[0] += ;
        new_setpoint.position_W[1] += (float)min_yi * eps_move_;
        new_setpoint.position_W[2] += (float)min_zi * eps_move_;
        position_controller_.SetTrajectoryPoint(new_setpoint);
        */
  }
  // ################################################################################
  else if(enable_swarm_ & SWARM_GRADIENT)
  {
      ROS_INFO_ONCE("SwarmController starting swarm mode (SWARM_GRADIENT)");

      EigenOdometry target_swarm;
      target_swarm.position[0] = target_swarm_.position_W[0];
      target_swarm.position[1] = target_swarm_.position_W[1];
      target_swarm.position[2] = target_swarm_.position_W[2];

      EigenOdometry cohesion_sum;
      EigenOdometry separation_sum;
      EigenOdometry target_sum;
      EigenOdometry obstacle_sum;
      EigenOdometry neighbourhood_center = odometry_;
      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
      {
          if(neighbourhood_bool[i]) // only for neighbourhood
          {
              cohesion_sum = cohesion_sum + dronestate[i].odometry_;
              separation_sum = separation_sum + (dronestate[i].odometry_ - odometry_) / pow(norm_squared(dronestate[i].odometry_ - odometry_),2);
              neighbourhood_center = neighbourhood_center + dronestate[i].odometry_;

              ROS_INFO_ONCE("SwarmController %d %d odo x=%f y=%f z=%f", droneNumber_, (int)i, odometry_.position[0], odometry_.position[1], odometry_.position[2]);
              ROS_INFO_ONCE("SwarmController %d %d coh x=%f y=%f z=%f", droneNumber_, (int)i, cohesion_sum.position[0], cohesion_sum.position[1], cohesion_sum.position[2]);
              ROS_INFO_ONCE("SwarmController %d %d sep x=%f y=%f z=%f", droneNumber_, (int)i, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2]);
          }
      }
      neighbourhood_center = neighbourhood_center / ((float)neighbourhood_cnt + 1.0);

      if(neighbourhood_cnt > 0)
      {
          cohesion_sum = (odometry_ - cohesion_sum / neighbourhood_cnt) * 2*mpc_cohesion_weight_;
          separation_sum = (separation_sum / neighbourhood_cnt) * 2*mpc_separation_weight_;
      }
      target_sum = (neighbourhood_center - target_swarm) * (2*mpc_target_weight_ / ((float)neighbourhood_cnt + 1.0));

      for(int i = 0; i < obstacle_count; i++) // iterate over all obstacles
      {
        float obstacle_dist = norm(odometry_ - obstacle_position[i]);
        obstacle_sum = obstacle_sum + (obstacle_position[i] - odometry_) * (2*mpc_obstacle_weight_ / (pow(obstacle_dist - obstacle_radius,3) * obstacle_dist));
      }
      ROS_INFO_ONCE("SwarmController %d obs x=%f y=%f z=%f", droneNumber_, obstacle_sum.position[0], obstacle_sum.position[1], obstacle_sum.position[2]);

      EigenOdometry gradient_sum = (cohesion_sum + separation_sum + target_sum + obstacle_sum) * gradient_scale_factor_;
      float gradient_abs = norm(gradient_sum); // length of vector
      ROS_INFO_ONCE("SwarmController %d sum x=%f y=%f z=%f l=%f", droneNumber_, gradient_sum.position[0], gradient_sum.position[1], gradient_sum.position[2], gradient_abs);
      //float dist_limit = eps_move_ * n_move_max_;
      float dist_limit = 3;
      if(gradient_abs > dist_limit) // limit distance for this controller
        dist_limit = dist_limit / gradient_abs;
      else
        dist_limit = 1;
      gradient_sum = gradient_sum * dist_limit; // rescale vector
      // */
      /* if(gradient_abs < eps_move_) // stepped function around 0
      {
        gradient_sum.position[0] = 0;
        gradient_sum.position[1] = 0;
        gradient_sum.position[2] = 0;
      }
      // */
      gradient_abs = norm(gradient_sum); // length of vector

      ROS_INFO_ONCE("SwarmController %d coh x=%f y=%f z=%f w=%f", droneNumber_, cohesion_sum.position[0], cohesion_sum.position[1], cohesion_sum.position[2], mpc_cohesion_weight_);
      ROS_INFO_ONCE("SwarmController %d sep x=%f y=%f z=%f w=%f", droneNumber_, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2], mpc_separation_weight_);
      ROS_INFO_ONCE("SwarmController %d tar x=%f y=%f z=%f w=%f", droneNumber_, target_sum.position[0], target_sum.position[1], target_sum.position[2], mpc_target_weight_);
      ROS_INFO_ONCE("SwarmController %d target_swarm x=%f y=%f z=%f", droneNumber_, target_swarm.position[0], target_swarm.position[1], target_swarm.position[2]);

      ROS_INFO_ONCE("SwarmController %d sum x=%f y=%f z=%f l=%f", droneNumber_, gradient_sum.position[0], gradient_sum.position[1], gradient_sum.position[2], gradient_abs);

      set_point.pose.position.x = odometry_.position[0] - gradient_sum.position[0];
      set_point.pose.position.y = odometry_.position[1] - gradient_sum.position[1];
      set_point.pose.position.z = odometry_.position[2] - gradient_sum.position[2];
  }
  // ################################################################################
  else if(enable_swarm_ & SWARM_GRADIENT_ENUM)
  {
      ROS_INFO_ONCE("SwarmController starting swarm mode (SWARM_GRADIENT_ENUM)");

      EigenOdometry target_swarm;
      target_swarm.position[0] = target_swarm_.position_W[0];
      target_swarm.position[1] = target_swarm_.position_W[1];
      target_swarm.position[2] = target_swarm_.position_W[2];

      EigenOdometry cohesion_sum;
      EigenOdometry separation_sum;
      EigenOdometry target_sum;
      EigenOdometry obstacle_sum;
      EigenOdometry neighbourhood_center = odometry_;
      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
      {
          if(neighbourhood_bool[i]) // only for neighbourhood
          {
              cohesion_sum = cohesion_sum + dronestate[i].odometry_;
              separation_sum = separation_sum + (dronestate[i].odometry_ - odometry_) / pow(norm_squared(dronestate[i].odometry_ - odometry_),2);
              neighbourhood_center = neighbourhood_center + dronestate[i].odometry_;

              ROS_INFO_ONCE("SwarmController %d %d odo x=%f y=%f z=%f", droneNumber_, (int)i, odometry_.position[0], odometry_.position[1], odometry_.position[2]);
              ROS_INFO_ONCE("SwarmController %d %d coh x=%f y=%f z=%f", droneNumber_, (int)i, cohesion_sum.position[0], cohesion_sum.position[1], cohesion_sum.position[2]);
              ROS_INFO_ONCE("SwarmController %d %d sep x=%f y=%f z=%f", droneNumber_, (int)i, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2]);
          }
      }
      neighbourhood_center = neighbourhood_center / ((float)neighbourhood_cnt + 1.0);

      if(neighbourhood_cnt > 0)
      {
          cohesion_sum = (odometry_ - cohesion_sum / neighbourhood_cnt) * 2*mpc_cohesion_weight_;
          separation_sum = (separation_sum / neighbourhood_cnt) * 2*mpc_separation_weight_;
      }
      target_sum = (neighbourhood_center - target_swarm) * (2*mpc_target_weight_ / ((float)neighbourhood_cnt + 1.0));

      for(int i = 0; i < obstacle_count; i++) // iterate over all obstacles
      {
        float obstacle_dist = norm(odometry_ - obstacle_position[i]);
        obstacle_sum = obstacle_sum + (obstacle_position[i] - odometry_) * (2*mpc_obstacle_weight_ / (pow(obstacle_dist - obstacle_radius,3) * obstacle_dist));
      }
      ROS_INFO_ONCE("SwarmController %d obs x=%f y=%f z=%f", droneNumber_, obstacle_sum.position[0], obstacle_sum.position[1], obstacle_sum.position[2]);

      EigenOdometry gradient_sum = cohesion_sum + separation_sum + target_sum + obstacle_sum;
      float gradient_abs = norm(gradient_sum); // length of vector
      gradient_sum = gradient_sum / gradient_abs; // normalize length

      float target_distance_current = norm(neighbourhood_center - target_swarm);
      float dyn_n_move_max = ceil((float)max(
                              (float)n_move_max_,
                              (float)min(
                                  (float)n_move_max_ * 3.0,
                                  (float)n_move_max_ * (((float)target_distance_current + 0.5) * (3.0 / 2))
                              )
                          ));
      ROS_INFO_ONCE("SwarmController %d dist_cur=%f n_move_max_= %d / %d", droneNumber_, target_distance_current, (int)dyn_n_move_max, n_move_max_);

      // determine length of target vector by enumeration
      float min_sum = FLT_MAX;
      int min_dist_i = 0;
      for(int dist_i = 0; dist_i <= (int)dyn_n_move_max; dist_i ++)
      {
          float cost_cohesion_sum = 0;
          float cost_separation_sum = 0;
          float cost_total_sum = 0;
          EigenOdometry potential_pos = odometry_ - gradient_sum * ((float)dist_i * (float)eps_move_);
          EigenOdometry potential_center = potential_pos;
          for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
          {
              if(neighbourhood_bool[i]) // only for neighbourhood
              {
                  float dist = dronestate[i].GetDistance(&potential_pos);

                  cost_cohesion_sum += dist*dist;
                  cost_separation_sum += 1.0/(dist*dist);
                  potential_center = potential_center + dronestate[i].odometry_;
              }
          }
          potential_center = potential_center / ((float)neighbourhood_cnt + 1.0);

          if(neighbourhood_cnt > 0)
          {
              cost_total_sum += mpc_cohesion_weight_ * cost_cohesion_sum / ((float)neighbourhood_cnt); // coehesion term
              cost_total_sum += mpc_separation_weight_ * cost_separation_sum / ((float)neighbourhood_cnt); // separation term
          }
          // target direction term for centroid
          if(target_swarm.position[2] != 0) // target point is available (z != 0)
          {
              EigenOdometry target_distance = potential_center - target_swarm;
              cost_total_sum += mpc_target_weight_*(target_distance.position[0]*target_distance.position[0] +
                                                    target_distance.position[1]*target_distance.position[1] +
                                                    target_distance.position[2]*target_distance.position[2]);
              ROS_INFO_ONCE("SwarmController %d pot. center  x=%f y=%f z=%f (div %f)", droneNumber_, potential_center.position[0], potential_center.position[1], potential_center.position[2], ((float)neighbourhood_cnt + 1));
              ROS_INFO_ONCE("SwarmController %d swarm target x=%f y=%f z=%f", droneNumber_, target_swarm.position[0], target_swarm.position[1], target_swarm.position[2]);
          }

          for(int i = 0; i < obstacle_count; i++) // iterate over all obstacles
          {
            obstacle_position[i].position[2] = potential_center.position[2]; // todo, remove fix for infinite z obstacles
            float obstacle_dist = norm(potential_pos - obstacle_position[i]);
            float tmp = mpc_obstacle_weight_ * (1.0/pow(obstacle_dist - obstacle_radius, 2));
            cost_total_sum += mpc_obstacle_weight_ * (1.0/pow(obstacle_dist - obstacle_radius, 2));
            ROS_INFO_ONCE("SwarmController %d obs %d cost=%f dist=%f", droneNumber_, i, tmp, obstacle_dist);
          }

          ROS_INFO_ONCE("SwarmController %d i=%d coh=%f sep=%f total=%f", droneNumber_, dist_i, cost_cohesion_sum, cost_separation_sum, cost_total_sum);

          if(cost_total_sum < min_sum)
          {
              min_sum = cost_total_sum;
              min_dist_i = dist_i;
          }
      }
      ROS_INFO_ONCE("SwarmController %d min_dist_i=%d", droneNumber_, min_dist_i);
      if(min_dist_i == 1)
          min_dist_i = 0;

      ROS_INFO_ONCE("SwarmController %d coh x=%f y=%f z=%f w=%f", droneNumber_, cohesion_sum.position[0], cohesion_sum.position[1], cohesion_sum.position[2], mpc_cohesion_weight_);
      ROS_INFO_ONCE("SwarmController %d sep x=%f y=%f z=%f w=%f", droneNumber_, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2], mpc_separation_weight_);
      ROS_INFO_ONCE("SwarmController %d tar x=%f y=%f z=%f w=%f", droneNumber_, target_sum.position[0], target_sum.position[1], target_sum.position[2], mpc_target_weight_);
      ROS_INFO_ONCE("SwarmController %d target_swarm x=%f y=%f z=%f", droneNumber_, target_swarm.position[0], target_swarm.position[1], target_swarm.position[2]);
      ROS_INFO_ONCE("SwarmController %d sum x=%f y=%f z=%f l=%f", droneNumber_, gradient_sum.position[0], gradient_sum.position[1], gradient_sum.position[2], gradient_abs);

      set_point.pose.position.x = odometry_.position[0] - gradient_sum.position[0] * ((float)min_dist_i * (float)eps_move_);
      set_point.pose.position.y = odometry_.position[1] - gradient_sum.position[1] * ((float)min_dist_i * (float)eps_move_);
      set_point.pose.position.z = odometry_.position[2] - gradient_sum.position[2] * ((float)min_dist_i * (float)eps_move_);
  }
  // ################################################################################
  else if(enable_swarm_ & SWARM_REYNOLDS || enable_swarm_ & SWARM_REYNOLDS_LIMITED || enable_swarm_ & SWARM_REYNOLDS_VELOCITY)
  {
      if(enable_swarm_ & SWARM_REYNOLDS)
        ROS_INFO_ONCE("SwarmController starting swarm mode (SWARM_REYNOLDS)");
      if(enable_swarm_ & SWARM_REYNOLDS_LIMITED)
        ROS_INFO_ONCE("SwarmController starting swarm mode (SWARM_REYNOLDS_LIMITED)");
      if(enable_swarm_ & SWARM_REYNOLDS_VELOCITY)
        ROS_INFO_ONCE("SwarmController starting swarm mode (SWARM_REYNOLDS_VELOCITY)");

      std::stringstream tempDistance;
      tempDistance << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";

      ROS_INFO_ONCE("SwarmController %d vel x=%f y=%f z=%f", droneNumber_, odometry_.velocity[0], odometry_.velocity[1], odometry_.velocity[2]);
      EigenOdometry integrated_velocity;
      integrated_velocity.position[0] = odometry_.velocity[0] * weighted_delta_t_;
      integrated_velocity.position[1] = odometry_.velocity[1] * weighted_delta_t_;
      integrated_velocity.position[2] = odometry_.velocity[2] * weighted_delta_t_;
      EigenOdometry position_next = odometry_ + integrated_velocity;
      if(enable_swarm_ & SWARM_REYNOLDS_LIMITED || enable_swarm_ & SWARM_REYNOLDS_VELOCITY || weighted_delta_t_ < 0.001) // do not use next position estimate for this controller
        position_next = odometry_;

      float abs_velocity = sqrt(SquaredScalarVelocity(&odometry_));
      ROS_INFO_ONCE("SwarmController %d vel=%f", droneNumber_, abs_velocity);

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

            ROS_INFO_ONCE("SwarmController %d (i=%d) vel x=%f y=%f z=%f", droneNumber_, (int)i, velocity_sum.velocity[0], velocity_sum.velocity[1], velocity_sum.velocity[2]);
            ROS_INFO_ONCE("SwarmController %d (i=%d) accel x=%f y=%f z=%f", droneNumber_, (int)i, separation_sum.position[0], separation_sum.position[1], separation_sum.position[2]);
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

      ROS_INFO_ONCE("SwarmController %d (|H|=%d) cohesion_accel x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, cohesion_accel.position[0], cohesion_accel.position[1], cohesion_accel.position[2]);
      ROS_INFO_ONCE("SwarmController %d (|H|=%d) separation_accel x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, separation_accel.position[0], separation_accel.position[1], separation_accel.position[2]);
      ROS_INFO_ONCE("SwarmController %d target_accel=%f x=%f y=%f z=%f", droneNumber_, abs_target_accel, target_accel.position[0], target_accel.position[1], target_accel.position[2]);
      ROS_INFO_ONCE("SwarmController %d (|H|=%d) accel=%f x=%f y=%f z=%f", droneNumber_, neighbourhood_cnt, abs_accel, accel.position[0], accel.position[1], accel.position[2]);

      //position_controller_.SetSetPoint(1.5, atan(accel.position[0]), atan(0-accel.position[1]), 0);
      // SwarmController::SetSetPoint(double z, double pitch, double roll, double yaw)

/*      mav_msgs::EigenTrajectoryPoint new_setpoint;
      new_setpoint.position_W = odometry_.position;
      new_setpoint.position_W[0] += accel.position[0];
      new_setpoint.position_W[1] += accel.position[1];
      new_setpoint.position_W[2] += accel.position[2];
      position_controller_.SetTrajectoryPoint(new_setpoint);
*/
      tempDistance << "\n";
      listDistance_.push_back(tempDistance.str());
  }
  // ################################################################################
  else if(enable_swarm_ & SWARM_REYNOLDS_VELOCITY && false)
  {
      ROS_INFO_ONCE("SwarmController starting OLD! swarm mode (SWARM_REYNOLDS_VELOCITY)");

      ROS_INFO_ONCE("SwarmController %d vel x=%f y=%f z=%f", droneNumber_, odometry_.velocity[0], odometry_.velocity[1], odometry_.velocity[2]);

      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
      {
          if(i == droneNumber_) // skip for own quadcopter
            continue;

          ROS_INFO_ONCE("SwarmController %d (i=%d) vel x=%f y=%f z=%f", droneNumber_, (int)i, dronestate[i].odometry_.velocity[0], dronestate[i].odometry_.velocity[1], dronestate[i].odometry_.velocity[2]);
      }

/*
      mav_msgs::EigenTrajectoryPoint new_setpoint;
      new_setpoint.position_W = odometry_.position;
      new_setpoint.position_W[0] = 0;
      new_setpoint.position_W[1] = 0;
      new_setpoint.position_W[2] = 1;
      position_controller_.SetTrajectoryPoint(new_setpoint);
*/
  }

  if(enable_swarm_ != SWARM_DISABLED || set_point.pose.position.z > 0.01) // do not enable drone until proper target point received
    setpoint_pub_.publish(set_point);

  if(dataStoring_active_) // save data for log files
  {
      tempDistance << "\n";
      listDistance_.push_back(tempDistance.str());
      tempMetrics << "\n";
      listMetrics_.push_back(tempMetrics.str());
      tempState << "\n";
      listState_.push_back(tempState.str());
  }
}

void DroneStateWithTime::PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
    ROS_INFO_ONCE("DroneStateWithTime got first odometry message.");

    // received message containing other drone position information
    // odometry_gt_ = EigenOdometry();
    odometry_gt_.timeStampSec = pose_msg->header.stamp.sec;
    odometry_gt_.timeStampNsec = pose_msg->header.stamp.nsec;
    odometry_gt_.position[0] = pose_msg->pose.position.x;
    odometry_gt_.position[1] = pose_msg->pose.position.y;
    odometry_gt_.position[2] = pose_msg->pose.position.z;
    odometry_ = odometry_gt_;

    // gaussian random number generator
    std::normal_distribution<float> dist(0.0, position_noise_);
    rand_cnt_++;
    if(rand_cnt_ > 10) // reduce frequency of noise
    {
      rand_cnt_ = 0;
      rand_x_ = max(-5*position_noise_, min((float)dist(generator), 5*position_noise_));
      rand_y_ = max(-5*position_noise_, min((float)dist(generator), 5*position_noise_));
      rand_z_ = max(-5*position_noise_, min((float)dist(generator), 5*position_noise_));
    }
    odometry_.position[0] = odometry_gt_.position[0] + rand_x_;
    odometry_.position[1] = odometry_gt_.position[1] + rand_y_;
    odometry_.position[2] = odometry_gt_.position[2] + rand_z_;

    ROS_INFO_ONCE("DroneStateWithTime %d/%d random noise (position_noise=%f) x=%f y=%f z=%f", self_, other_, position_noise_, rand_x_, rand_y_, rand_z_);
    ROS_INFO_ONCE("DroneStateWithTime got odometry message: x=%f y=%f z=%f (self:%d, other:%d)", odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2], self_, other_);
}

float DroneStateWithTime::GetDistance(EigenOdometry* odometry) {
    if(self_ == other_)
        return 0;

    float distance_x = fabs(odometry_.position[0] - odometry->position[0]);
    float distance_y = fabs(odometry_.position[1] - odometry->position[1]);
    float distance_z = fabs(odometry_.position[2] - odometry->position[2]);
    return sqrt(distance_x*distance_x + distance_y*distance_y + distance_z*distance_z);
}

float DroneStateWithTime::GetDistance_gt(EigenOdometry* odometry_gt) {
    if(self_ == other_)
        return 0;

    float distance_x = fabs(odometry_gt_.position[0] - odometry_gt->position[0]);
    float distance_y = fabs(odometry_gt_.position[1] - odometry_gt->position[1]);
    float distance_z = fabs(odometry_gt_.position[2] - odometry_gt->position[2]);
    return sqrt(distance_x*distance_x + distance_y*distance_y + distance_z*distance_z);
}

void DroneStateWithTime::UpdateDistance(EigenOdometry* odometry, EigenOdometry* odometry_gt) {
    distance_ = this->GetDistance(odometry);
    distance_gt_ = this->GetDistance_gt(odometry_gt);

    ROS_INFO_ONCE("DroneStateWithTime distance=%f distance_gt=%f (self:%d, other:%d)", distance_, distance_gt_, self_, other_);
}

void DroneStateWithTime::SetId(int self, int other, float position_noise)
{
    self_ = self;
    other_ = other;
    position_noise_ = position_noise;

    rand_cnt_ = 0;
    rand_x_ = 0;
    rand_y_ = 0;
    rand_z_ = 0;
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
    ros::init(argc, argv, "swarm_controller_node");

    ros::NodeHandle nh2;

    rotors_control::SwarmController swarm_controller_node;

    ros::spin();

    return 0;
}
