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

#include "relative_distance_controller.h"

#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <chrono>

#include <sys/stat.h>
#include <sys/types.h>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/Eigen.h"

namespace rotors_control {

RelativeDistanceController::RelativeDistanceController() {

    ROS_INFO("Started RelativeDistanceController");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();
    InitializeML();

    generator_.seed(droneNumber_ * 7); // make sure random numbers are different for each drone
    ROS_INFO("RelativeDistanceController: Seed random number generator (droneNumber_): %d", droneNumber_);

    history_cnt_ = 0;
    random_direction_[0] = 0;
    random_direction_[1] = 0;
    random_direction_[2] = 0;
    odometry_history1_.position[0] = 0;
    odometry_history1_.position[1] = 0;
    odometry_history1_.position[2] = 0;
    for (size_t i = 0; i < N_VECTORS_MAX; i++)
    {
        unit_vectors_[i][0] = 0;
        unit_vectors_[i][1] = 0;
        unit_vectors_[i][2] = 0;
        unit_vectors_age_[i] = -1;
    }
    transform_ok_ = 0;
    transform_available_ = 0;
    for (size_t i = 0; i < droneCount_; i++)
    {
        for (size_t j = 0; j < droneCount_; j++)
        {
            distances_[i][j] = 0;
            distances_filtered_[i][j] = 0;
        }
        for (size_t j = 0; j < beaconCount_; j++)
        {
            beacons_[i][j] = 0;
            beacons_filtered_[i][j] = 0;
        }
        elevation_filtered_[i] = 0;
    }

    cyclic_current_phase_ = CYCLIC_PHASE_REST;

    // Topics subscribe
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, &RelativeDistanceController::MultiDofJointTrajectoryCallback, this);
    odometry_sub_ = nh.subscribe("odometry", 1, &RelativeDistanceController::OdometryCallback, this);
    enable_sub_ = nh.subscribe("enable", 1, &RelativeDistanceController::EnableCallback, this);
    target_sub_ = nh.subscribe("target", 1, &RelativeDistanceController::TargetCallback, this);
    update_sub_ = nh.subscribe("update", 1, &RelativeDistanceController::UpdateCallback, this);
    logsave_sub_ = nh.subscribe("logsave", 1, &RelativeDistanceController::SaveLogCallback, this);
    distances_sub_ = nh.subscribe("drone_distances", 1, &RelativeDistanceController::DistancesCallback, this);
    positions_sub_ = nh.subscribe("/drone_positions", 1, &RelativeDistanceController::PositionsCallback, this); // ground truth information needed for metric calculation
    elevation_sub_ = nh.subscribe("/drone_elevation", 1, &RelativeDistanceController::ElevationCallback, this); // ground truth information needed for metric calculation
    beacons_sub_ = nh.subscribe("beacon_distances", 1, &RelativeDistanceController::BeaconsCallback, this);
    modelstate_sub_ = nh.subscribe("/gazebo/model_states", 1, &RelativeDistanceController::ModelstateCallback, this); // ground truth information needed for metric calculation

    // Absolute position of drones, only used for debugging!
    ros::NodeHandle nhq[N_DRONES_MAX];
    for (size_t i = 0; i < droneCount_; i++)
    {
      nhq[i] = ros::NodeHandle(std::string("/crazyflie2_") + std::to_string(i));

      dronestate_[i].SetId(droneNumber_, i);
      ROS_INFO("RelativeDistanceController: Setup subscriber %s/lps_pose.", nhq[i].getNamespace().c_str());
      pose_other_sub_[i] = nhq[i].subscribe("lps_pose", 1, &DroneStateWithTime::PoseCallback, &dronestate_[i]); // ground truth information needed for metric calculation
    }

    // To publish the set-point
    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("set_point", 1);

    // To publish the current pose for gazebo visual
    gazebo_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

RelativeDistanceController::~RelativeDistanceController(){}

void RelativeDistanceController::InitializeParams() {
    ros::NodeHandle pnh("~");

    ROS_INFO_ONCE("[RelativeDistanceController] InitializeParams");

    GetRosParameter(pnh, "swarm/drone_radius", (float)0.1, &drone_radius_);
    GetRosParameter(pnh, "swarm/neighbourhood_distance", (float)99, &neighbourhood_distance_);
    GetRosParameter(pnh, "dist/eps_move", (float)0.2, &eps_move_);
    GetRosParameter(pnh, "dist/n_move_max", (int)2, &n_move_max_);
    GetRosParameter(pnh, "dist/spc_cohesion_weight", (float)1.0, &spc_cohesion_weight_);
    GetRosParameter(pnh, "dist/spc_separation_weight", (float)1.0, &spc_separation_weight_);
    GetRosParameter(pnh, "dist/spc_target_weight", (float)1.0, &spc_target_weight_);
    GetRosParameter(pnh, "dist/spc_height_weight", (float)1.0, &spc_height_weight_);
    GetRosParameter(pnh, "dist/spc_limit_weight", (float)0.0, &spc_limit_weight_);
    GetRosParameter(pnh, "dist/spc_calm_weight", (float)1.0, &spc_calm_weight_);
    GetRosParameter(pnh, "dist/explore_command_length", (float)1.0, &explore_command_length_);
    GetRosParameter(pnh, "dist/explore_movement_thr", (float)1.0, &explore_movement_thr_);
    GetRosParameter(pnh, "dist/explore_distance_cap", (float)100.0, &explore_distance_cap_);
    GetRosParameter(pnh, "dist/velocity_scaling", (float)0.1, &velocity_scaling_);
    GetRosParameter(pnh, "dist/distance_iir_filter", (float)0.1, &distance_iir_filter_);
    GetRosParameter(pnh, "dist/elevation_iir_filter", (float)0.1, &elevation_iir_filter_);
    GetRosParameter(pnh, "dist/distance_min_filter", (float)0.01, &distance_min_filter_);
    GetRosParameter(pnh, "dist/distance_max_filter", (float)100.0, &distance_max_filter_);
    GetRosParameter(pnh, "dist/extra_separation_distance", (float)0.0, &extra_separation_distance_);
    GetRosParameter(pnh, "dist/noise_suppression", (int)1, &noise_suppression_);
    GetRosParameter(pnh, "elevation/target_elevation", (float)0.0, &target_elevation_);
    GetRosParameter(pnh, "elevation/swarm_elevation", (float)1.0, &swarm_elevation_);
    GetRosParameter(pnh, "inner/controller", (int)0, &inner_controller_);

    ROS_INFO_ONCE("[RelativeDistanceController] GetRosParameter values:");
    ROS_INFO_ONCE("  swarm/drone_radius=%f", drone_radius_);
    ROS_INFO_ONCE("  swarm/neighbourhood_distance=%f", neighbourhood_distance_);
    ROS_INFO_ONCE("  dist/eps_move=%f", eps_move_);
    ROS_INFO_ONCE("  dist/n_move_max=%d", n_move_max_);
    ROS_INFO_ONCE("  dist/spc_cohesion_weight=%f", spc_cohesion_weight_);
    ROS_INFO_ONCE("  dist/spc_separation_weight=%f", spc_separation_weight_);
    ROS_INFO_ONCE("  dist/spc_target_weight=%f", spc_target_weight_);
    ROS_INFO_ONCE("  dist/spc_height_weight=%f", spc_height_weight_);
    ROS_INFO_ONCE("  dist/spc_limit_weight=%f", spc_limit_weight_);
    ROS_INFO_ONCE("  dist/spc_cohesion_weight=%f", spc_cohesion_weight_);
    ROS_INFO_ONCE("  dist/explore_command_length=%f", explore_command_length_);
    ROS_INFO_ONCE("  dist/explore_movement_thr=%f", explore_movement_thr_);
    ROS_INFO_ONCE("  dist/explore_distance_cap=%f", explore_distance_cap_);
    ROS_INFO_ONCE("  dist/velocity_scaling=%f", velocity_scaling_);
    ROS_INFO_ONCE("  dist/distance_iir_filter=%f", distance_iir_filter_);
    ROS_INFO_ONCE("  dist/elevation_iir_filter=%f", elevation_iir_filter_);
    ROS_INFO_ONCE("  dist/distance_min_filter=%f", distance_min_filter_);
    ROS_INFO_ONCE("  dist/distance_max_filter=%f", distance_max_filter_);
    ROS_INFO_ONCE("  dist/extra_separation_distance=%f", extra_separation_distance_);
    ROS_INFO_ONCE("  dist/noise_suppression=%d", noise_suppression_);
    ROS_INFO_ONCE("  elevation/target_elevation=%f", target_elevation_);
    ROS_INFO_ONCE("  elevation/swarm_elevation=%f", swarm_elevation_);
    ROS_INFO_ONCE("  inner/controller=%d", inner_controller_);

    //Reading the parameters come from the launch file
    std::string dataStoringActive;
    int droneCount;
    int beaconCount;
    int droneNumber;
    double dataStoringTime;
    int isSim;

    if (pnh.getParam("droneCount", droneCount)){
       ROS_INFO("Got param 'droneCount': %d", droneCount);
       droneCount_ = droneCount;
    }
    else
       ROS_ERROR("Failed to get param 'droneCount'");
    if (pnh.getParam("beaconCount", beaconCount)){
       ROS_INFO("Got param 'beaconCount': %d", beaconCount);
       beaconCount_ = beaconCount;
    }
    else
       ROS_ERROR("Failed to get param 'beaconCount'");

    if (pnh.getParam("droneNumber", droneNumber)){
        ROS_INFO("Got param 'droneNumber': %d", droneNumber);
        droneNumber_ = droneNumber;
    }
    else
        ROS_ERROR("Failed to get param 'droneCount'");

    if (pnh.getParam("csvFilesStoring", dataStoringActive)){
        ROS_INFO("Got param 'csvFilesStoring': %s", dataStoringActive.c_str());

        dataStoring_active_ = false;
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

    if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
        ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
    }

    isSim_ = 1;
    if (pnh.getParam("isSim", isSim)){
       ROS_INFO("Got param 'isSim': %d", isSim);
       isSim_ = isSim;
//       if(isSim_ == 0)
//          enable_swarm_ = SWARM_SPC_DISTANCES_ONLY; // auto-enable, if not in simulation mode.
    }

    ros::NodeHandle nh;
    timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &RelativeDistanceController::CallbackSaveData, this, false, true);
}

void RelativeDistanceController::InitializeML() {
    try
    {
      torch::Tensor tensor = torch::tensor({{(float)0.0, (float)0.0, (float)0.0, (float)0.0}}, {torch::kFloat32});
      // Deserialize the ScriptModule from a file using torch::jit::load().
      for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
      {
          distances_model_[i] = torch::jit::load("/crazyflie_ws/src/crazys/rotors_control/resource/model_scripted.pt");
          auto ret = distances_model_[i].forward({tensor}).toTuple();
          distances_hx_[i] = ret->elements()[1];
      }
      for (size_t i = 0; i < beaconCount_; i++) // iterate over all quadcopters
      {
          beacons_model_[i] = torch::jit::load("/crazyflie_ws/src/crazys/rotors_control/resource/model_scripted.pt");
          auto ret = beacons_model_[i].forward({tensor}).toTuple();
          beacons_hx_[i] = ret->elements()[1];
      }

/*      torch::Tensor tensor = torch::tensor({{1, 2, 3, 4}}, {torch::kFloat32});

      auto ret = model_.forward({tensor}).toTuple();
      torch::Tensor outputs = ret->elements()[0].toTensor();
      c10::IValue hx = ret->elements()[1];

      torch::jit::script::Module distances_model_[N_DRONES_MAX];
      c10::IValue distances_hx_[N_DRONES_MAX];
      torch::jit::script::Module beacons_model_[N_DRONES_MAX];
      c10::IValue beacons_hx_[N_DRONES_MAX];

      ROS_INFO("RelativeDistanceController output=%f.", outputs[0].item<float>());
      ret = model_.forward({tensor, hx}).toTuple();

*/
    }
    catch (const c10::Error& e)
    {
      ROS_FATAL("RelativeDistanceController (%d) error loading the ML model: %s", droneNumber_, e.msg().c_str());
    }
    ROS_INFO("RelativeDistanceController (%d) InitializeML done.", droneNumber_);
}

void RelativeDistanceController::EnableCallback(const std_msgs::Int32ConstPtr& enable_msg) {
    ROS_INFO("RelativeDistanceController (%d) got enable message: %d", droneNumber_, enable_msg->data);

    enable_swarm_ = enable_msg->data;
}

void RelativeDistanceController::TargetCallback(const std_msgs::Int32ConstPtr& target_msg) {
    ROS_INFO("RelativeDistanceController (%d) got target message: %d", droneNumber_, target_msg->data);

    if(target_msg->data >= N_BEACONS_MAX)
        ROS_FATAL("RelativeDistanceController (%d) target message (%d) exceeds N_BEACONS_MAX (%d)", droneNumber_, target_msg->data, N_BEACONS_MAX);
    current_target_ = target_msg->data % N_BEACONS_MAX; // make sure to not exceed array bounds
}

void RelativeDistanceController::UpdateCallback(const std_msgs::Int32ConstPtr& update_msg) {
    ROS_INFO("RelativeDistanceController (%d) got update message: %d. Invalidate environment history data.", droneNumber_, update_msg->data);

    unit_vectors_age_[0] = -2;
    unit_vectors_age_[1] = -2;
    unit_vectors_age_[2] = -2;

    odometry_history1_ = odometry_;
    history_cnt_ = 0;
    random_direction_[0] = 0;
    random_direction_[1] = 0;
    random_direction_[2] = 0;

    transform_ok_ = 0; // need to do exploration now
}

//The callback saves data into csv files
void RelativeDistanceController::CallbackSaveData(const ros::TimerEvent& event){
  ROS_INFO("RelativeDistanceController (%d) CallbackSavaData.", droneNumber_);
  FileSaveData();
}

void RelativeDistanceController::SaveLogCallback(const std_msgs::Int32ConstPtr& save_msg) {
  ROS_INFO("RelativeDistanceController (%d) SaveLogCallback.", droneNumber_);
  FileSaveData();
}

void RelativeDistanceController::FileSaveData(void){

      if(!dataStoring_active_){
         return;
      }

      std::ofstream fileDistance;
      std::ofstream fileEnv;
      std::ofstream fileState;
      std::ofstream fileCost;
      std::ofstream fileVectors;

      ROS_INFO("RelativeDistanceController %d FileSaveData.", droneNumber_);

      if(mkdir("/tmp/log_output/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
          if(errno != EEXIST)
             ROS_ERROR("Cannot create directory /tmp/log_output/");

      if(droneNumber_ < 10)
      {
           fileDistance.open(std::string("/tmp/log_output/Distance0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileEnv.open(std::string("/tmp/log_output/Env0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileState.open(std::string("/tmp/log_output/State0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileCost.open(std::string("/tmp/log_output/Cost0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileVectors.open(std::string("/tmp/log_output/Vectors0") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      }
      else
      {
           fileDistance.open(std::string("/tmp/log_output/Distance") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileEnv.open(std::string("/tmp/log_output/Env") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileState.open(std::string("/tmp/log_output/State") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileCost.open(std::string("/tmp/log_output/Cost") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
           fileVectors.open(std::string("/tmp/log_output/Vectors") + std::to_string(droneNumber_) + std::string(".csv"), std::ios_base::trunc);
      }

      // Saving distances from every to every drone in a file
      for (unsigned n=0; n < listDistance_.size(); ++n) {
          fileDistance << listDistance_.at( n );
      }
      // Saving quality metrics in a file
      for (unsigned n=0; n < listEnv_.size(); ++n) {
          fileEnv << listEnv_.at( n );
      }
      // Saving states in a file
      for (unsigned n=0; n < listState_.size(); ++n) {
          fileState << listState_.at( n );
      }
      // Saving cost function values in a file
      for (unsigned n=0; n < listCost_.size(); ++n) {
          fileCost << listCost_.at( n );
      }
      // Saving vectors in a file
      for (unsigned n=0; n < listVectors_.size(); ++n) {
          fileVectors << listVectors_.at( n );
      }

      // Closing all opened files
      fileDistance.close();
      fileEnv.close();
      fileState.close();
      fileCost.close();

      // To have a one shot storing
      dataStoring_active_ = false;

      ROS_INFO("RelativeDistanceController %d FileSaveData. Done.", droneNumber_);
}

void RelativeDistanceController::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  ROS_INFO_ONCE("SwarmController got MultiDOFJointTrajectory message: x=%f, y=%f, z=%f, yaw=%f", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2], eigen_reference.getYaw());

  // set target point for hovering or swarm target if in swarm mode
  target_swarm_ = eigen_reference;
}

void RelativeDistanceController::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    ROS_INFO_ONCE("RelativeDistanceController got first odometry message.");

    // received message drone position information, store ground-truth pose
    odometry_.timeStampSec = odometry_msg->header.stamp.sec;
    odometry_.timeStampNsec = odometry_msg->header.stamp.nsec;
    odometry_.position[0] = odometry_msg->pose.pose.position.x;
    odometry_.position[1] = odometry_msg->pose.pose.position.y;
    odometry_.position[2] = odometry_msg->pose.pose.position.z;
    ROS_INFO_ONCE("RelativeDistanceController (%d) odometry: x=%f y=%f z=%f", droneNumber_, odometry_.position[0], odometry_.position[1], odometry_.position[2]);

    // this replaces ElevationCallback
    elevation_[droneNumber_] = odometry_msg->pose.pose.position.z;
    //elevation_filtered_[droneNumber_] = elevation_[droneNumber_]; // do not filter elevation for Hardware (testing)
    if(noise_suppression_ == 1)
      elevation_filtered_[droneNumber_] = elevation_filtered_[droneNumber_]*(1.0-elevation_iir_filter_) + elevation_[droneNumber_]*(elevation_iir_filter_); // IIR lowpass filter for distance measurements
    else if(noise_suppression_ == 0)
      elevation_filtered_[droneNumber_] = elevation_[droneNumber_];

    // for logging into files
    std::stringstream tempDistance;
    tempDistance.precision(24);
    tempDistance << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";
    for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
    {
        tempDistance << distances_[droneNumber_][i] << ",";
        tempDistance << distances_filtered_[droneNumber_][i] << ",";
    }
    for (size_t i = 0; i < beaconCount_; i++) // iterate over all quadcopters
    {
        tempDistance << beacons_[droneNumber_][i] << ",";
        tempDistance << beacons_filtered_[droneNumber_][i] << ",";
    }
    std::stringstream tempEnv;
    tempEnv.precision(24);
    tempEnv << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";
    std::stringstream tempState;
    tempState.precision(24);
    tempState << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";
    tempState << odometry_.position[0] << "," << odometry_.position[1] << "," << odometry_.position[2] << ",";
    tempState << current_target_ << ",";
    std::stringstream tempCost;
    tempCost.precision(24);
    tempCost << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";

    std::stringstream tempVectors;
    tempVectors.precision(6);
    tempVectors << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << enable_swarm_ << ",";

    tempVectors << unit_vectors_[0][0] << "," << unit_vectors_[0][1] << "," << unit_vectors_[0][2] << ","; // vector 0; x,y,z
    tempVectors << unit_vectors_[1][0] << "," << unit_vectors_[1][1] << "," << unit_vectors_[1][2] << ","; // vector 1; x,y,z
    tempVectors << unit_vectors_[2][0] << "," << unit_vectors_[2][1] << "," << unit_vectors_[2][2] << ","; // vector 2; x,y,z

    for (size_t i = 0; i < droneCount_; i++)
        tempVectors << distances_differences_[0][i] << "," << distances_differences_[1][i] << "," << distances_differences_[2][i] << ","; // vector 0; x,y,z
    for (size_t i = 0; i < beaconCount_; i++)
        tempVectors << beacons_differences_[0][i] << "," << beacons_differences_[1][i] << "," << beacons_differences_[2][i] << ","; // vector 0; x,y,z


/*
    // distance measurements from previous message (to check for large changes, when target is updated) -- was replaced by update_sub_ !
    int beacons_moved = 0;
    for (size_t i = 0; i < beaconCount_; i++)
    {
        if(fabs(beacons_last_[i] - beacons_[droneNumber_][i]) > 1.5) // distance changed for more than 150cm within one timestep
        {
            beacons_moved = 1;
            ROS_INFO("OdometryCallback (%d) beacon %d moved (distance old: %f, new: %f).", droneNumber_, (int)i, beacons_last_[i], beacons_[droneNumber_][i]);
        }
        beacons_last_[i] = beacons_[droneNumber_][i];
    }
/*
    if(beacons_moved)
    {
        ROS_INFO("OdometryCallback (%d) at least one beacon moved. Invalidate environment history data.", droneNumber_); // TODO: maybe there is a better way than deleting all data?

        unit_vectors_age_[0] = -2;
        unit_vectors_age_[1] = -2;
        unit_vectors_age_[2] = -2;

        odometry_history1_ = odometry_;
        history_cnt_ = 0;
        random_direction_[0] = 0;
        random_direction_[1] = 0;
        random_direction_[2] = 0;
    }
    */

    // calculate movement vector based on current ground-truth position and history (TODO: use accelerometer data)
    Vector3f movement;
    movement[0] = odometry_.position[0] - odometry_history1_.position[0];
    movement[1] = odometry_.position[1] - odometry_history1_.position[1];
    movement[2] = odometry_.position[2] - odometry_history1_.position[2];
    float movement_norm = movement.norm();

    // setpoint for message to be sent to low-level controller
    geometry_msgs::PoseStamped set_point;
    Vector3f set_point_marker;
    set_point.header.stamp = odometry_msg->header.stamp;

    if(!(enable_swarm_ & SWARM_SPC_CYCLIC))
    {
        if(history_cnt_ < HISTORY_CNT_MAX)
        {
            for (size_t i = 0; i < droneCount_; i++)
                distances_history_[i][history_cnt_] = distances_[droneNumber_][i];
                //distances_history_[i][history_cnt_] = distances_filtered_[droneNumber_][i]; // TODO: is filtered version better, or non filtered?
            for (size_t i = 0; i < beaconCount_; i++)
                beacons_history_[i][history_cnt_] = beacons_[droneNumber_][i];
                //beacons_history_[i][history_cnt_] = beacons_filtered_[droneNumber_][i]; // TODO: is filtered version better, or non filtered?
        }
        if(unit_vectors_age_[0] >= 0) unit_vectors_age_[0] ++; // age of -1 means this unit vector is invalid
        if(unit_vectors_age_[1] >= 0) unit_vectors_age_[1] ++; // age of -1 means this unit vector is invalid
        if(unit_vectors_age_[2] >= 0) unit_vectors_age_[2] ++; // age of -1 means this unit vector is invalid

        if(unit_vectors_age_[0] > HISTORY_CNT_MAX*5) // delete old unit vectors
        {
            Vector3f tmp = unit_vectors_[0];
            unit_vectors_[0] = unit_vectors_[2];
            unit_vectors_[2] = tmp;
            unit_vectors_age_[0] = unit_vectors_age_[2];
            unit_vectors_age_[2] = -1;
            transform_ok_ = 0; // need to do exploration now
            ROS_INFO("OdometryCallback (%d) deleted old unit_vector_0 (moved to 2)", droneNumber_);
        }
        else if(unit_vectors_age_[1] > HISTORY_CNT_MAX*5) // delete old unit vectors
        {
            Vector3f tmp = unit_vectors_[1];
            unit_vectors_[1] = unit_vectors_[2];
            unit_vectors_[2] = tmp;
            unit_vectors_age_[1] = unit_vectors_age_[2];
            unit_vectors_age_[2] = -1;
            transform_ok_ = 0; // need to do exploration now
            ROS_INFO("OdometryCallback (%d) deleted old unit_vector_1 (moved to 2)", droneNumber_);
        }
        else if(unit_vectors_age_[2] > HISTORY_CNT_MAX*5) // delete old unit vectors
        {
            unit_vectors_age_[2] = -1;
            transform_ok_ = 0; // need to do exploration now
            ROS_INFO("OdometryCallback (%d) deleted old unit_vector_2", droneNumber_);
        }

        if(history_cnt_ >= HISTORY_CNT_MAX)
        {
            ROS_INFO("OdometryCallback (%d) Over threshold: %d. Reset history_cnt_: %d.", droneNumber_, HISTORY_CNT_MAX, history_cnt_);
            odometry_history1_ = odometry_;
            history_cnt_ = 0;
            random_direction_[0] = 0;
            random_direction_[1] = 0;
            random_direction_[2] = 0;
        }
        else if(movement_norm > explore_movement_thr_) // last move was at least Xcm, TODO: find better threshold?
        {
            ROS_INFO_ONCE("OdometryCallback (%d) movement over threshold (%f): %s", droneNumber_, explore_movement_thr_, VectorToString(movement).c_str());

            if(enable_swarm_ != SWARM_DISABLED)
            {
                movement = movement / movement_norm; // divide by norm to get unit vector

                // calculate similarity of unit_vectors to each other
                float same_dot[3];
                same_dot[0] = abs(unit_vectors_[0].dot(unit_vectors_[1])) + abs(unit_vectors_[0].dot(unit_vectors_[2]));
                same_dot[1] = abs(unit_vectors_[1].dot(unit_vectors_[0])) + abs(unit_vectors_[1].dot(unit_vectors_[2]));
                same_dot[2] = abs(unit_vectors_[2].dot(unit_vectors_[1])) + abs(unit_vectors_[2].dot(unit_vectors_[0]));

                // find unit vector closest to the current movement vector: having the largest dot product
                float max_dot_product = 0;
                size_t index_dot_product = 0;
                for (size_t i = 0; i < N_VECTORS_MAX; i++)
                {
                    if(unit_vectors_age_[i] < 0) // this unit vector is not yet initialized, use it
                    {
                        ROS_INFO_ONCE("OdometryCallback (%d) this unit vector is not yet initialized, use it: %d", droneNumber_, (int)i);
                        index_dot_product = i;
                        break;
                    }
                    float dot_product = abs(unit_vectors_[i].dot(movement)); // TODO: find factor for penalizing same similarity
                    //ROS_INFO("%s * %s (%d) = %f (%f)", VectorToString(unit_vectors_[i]).c_str(), VectorToString(movement).c_str(), (int)i, dot_product, same_dot[i]);

                    if(dot_product*same_dot[i] > max_dot_product)
                    {
                        index_dot_product = i;
                        max_dot_product = dot_product*same_dot[i];
                    }
                }

                ROS_INFO_ONCE("OdometryCallback (%d) replace unit vector: %d", droneNumber_, (int)index_dot_product);
                unit_vectors_[index_dot_product] = movement;
                unit_vectors_age_[index_dot_product] = 0;

                if(noise_suppression_ == 0)
                {
                  // old version using difference only
                  for (size_t i = 0; i < droneCount_; i++)
                      distances_differences_[index_dot_product][i] = (distances_[droneNumber_][i] - distances_history_[i][0]) / movement_norm;
                  for (size_t i = 0; i < beaconCount_; i++)
                      beacons_differences_[index_dot_product][i] = (beacons_[droneNumber_][i] - beacons_history_[i][0]) / movement_norm;
                }
                else if(noise_suppression_ == 1)
                {
                  // new version using linear least squares approximation
                  for (size_t i = 0; i < droneCount_; i++)
                  {
                      if(i == droneNumber_) // skip for own quadcopter
                      {
                          distances_differences_[index_dot_product][i] = 0;
                          continue;
                      }

                      float alpha0, alpha1;
                      linearLeastSquaresApproximation(distances_history_[i], (size_t)history_cnt_, &alpha0, &alpha1);
                      distances_differences_[index_dot_product][i] = (alpha1 * history_cnt_) / movement_norm;
                      // cap by +/- 1, as the change should not be larger than the movement-norm
                      if(distances_differences_[index_dot_product][i] < 0-explore_distance_cap_)
                          distances_differences_[index_dot_product][i] = 0-explore_distance_cap_;
                      if(distances_differences_[index_dot_product][i] > explore_distance_cap_)
                          distances_differences_[index_dot_product][i] = explore_distance_cap_;

                      /*float distances_differences_diff = (distances_[droneNumber_][i] - distances_history_[i][0]) / movement_norm;
                      ROS_INFO("OdometryCallback distances_=%f distances_history_=%f", distances_[droneNumber_][i], distances_history_[i][0]);
                      ROS_INFO("OdometryCallback (%d) to %d (history_cnt_=%d) LSQApr=%f, diff=%f ", droneNumber_, (int)i, history_cnt_, distances_differences_[index_dot_product][i], distances_differences_diff);
                      */
                  }
                  for (size_t i = 0; i < beaconCount_; i++)
                  {
                      float alpha0, alpha1;
                      linearLeastSquaresApproximation(beacons_history_[i], (size_t)history_cnt_, &alpha0, &alpha1);
                      beacons_differences_[index_dot_product][i] = (alpha1 * history_cnt_) / movement_norm;
                      // cap by +/- 1, as the change should not be larger than the movement-norm
                      if(beacons_differences_[index_dot_product][i] < 0-explore_distance_cap_)
                          beacons_differences_[index_dot_product][i] = 0-explore_distance_cap_;
                      if(beacons_differences_[index_dot_product][i] > explore_distance_cap_)
                          beacons_differences_[index_dot_product][i] = explore_distance_cap_;
                  }
                }
                else
                  ROS_FATAL("OdometryCallback (%d) invalid value for noise_suppression: %d", droneNumber_, noise_suppression_);

                ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_0 (age:%d): %s", droneNumber_, unit_vectors_age_[0], VectorToString(unit_vectors_[0]).c_str());
                ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_1 (age:%d): %s", droneNumber_, unit_vectors_age_[1], VectorToString(unit_vectors_[1]).c_str());
                ROS_INFO_ONCE("OdometryCallback (%d) unit_vectors_2 (age:%d): %s\n", droneNumber_, unit_vectors_age_[2], VectorToString(unit_vectors_[2]).c_str());

                Vector3f span_vectors;
                span_vectors[0] = fmax(unit_vectors_[0][0], fmax(unit_vectors_[1][0], unit_vectors_[2][0])) - fmin(unit_vectors_[0][0], fmin(unit_vectors_[1][0], unit_vectors_[2][0]));
                span_vectors[1] = fmax(unit_vectors_[0][1], fmax(unit_vectors_[1][1], unit_vectors_[2][1])) - fmin(unit_vectors_[0][1], fmin(unit_vectors_[1][1], unit_vectors_[2][1]));
                span_vectors[2] = fmax(unit_vectors_[0][2], fmax(unit_vectors_[1][2], unit_vectors_[2][2])) - fmin(unit_vectors_[0][2], fmin(unit_vectors_[1][2], unit_vectors_[2][2]));
                ROS_INFO_ONCE("OdometryCallback (%d) span_vectors: %s", droneNumber_, VectorToString(span_vectors).c_str());

                if(unit_vectors_[0].norm() > 0.01 &&
                   unit_vectors_[1].norm() > 0.01 &&
                   unit_vectors_[2].norm() > 0.01 &&
                   span_vectors[0] > 0.01 && span_vectors[1] > 0.01 && span_vectors[2] > 0.01 &&
                   abs(unit_vectors_[0].dot(unit_vectors_[1])) < 0.9 &&
                   abs(unit_vectors_[1].dot(unit_vectors_[2])) < 0.9 &&
                   abs(unit_vectors_[2].dot(unit_vectors_[0])) < 0.9)
                {
                    // calculate basis transform Matrix
                    Matrix3f transform_tmp;
                    transform_tmp(0,0) = unit_vectors_[0][0];
                    transform_tmp(1,0) = unit_vectors_[0][1];
                    transform_tmp(2,0) = unit_vectors_[0][2];
                    transform_tmp(0,1) = unit_vectors_[1][0];
                    transform_tmp(1,1) = unit_vectors_[1][1];
                    transform_tmp(2,1) = unit_vectors_[1][2];
                    transform_tmp(0,2) = unit_vectors_[2][0];
                    transform_tmp(1,2) = unit_vectors_[2][1];
                    transform_tmp(2,2) = unit_vectors_[2][2];
                    transform_vectors_ = transform_tmp.inverse();
                    ROS_INFO_ONCE("OdometryCallback (%d) transform_vectors_: %s", droneNumber_, MatrixToString(transform_vectors_).c_str());
                    if(unit_vectors_age_[0] >= 0 && unit_vectors_age_[1] >= 0 && unit_vectors_age_[2] >= 0)
                        transform_ok_ = 1;
                    else
                        transform_ok_ = 0;
                    transform_available_ = 1;
                }
                else
                {
                    ROS_INFO_ONCE("OdometryCallback (%d) Bad unit vectors, cannot calculate transform_vectors.", droneNumber_);
                    transform_ok_ = 0;
                    transform_available_ = 0;
                }
            }

            odometry_history1_ = odometry_;
            ROS_INFO_ONCE("OdometryCallback (%d) Reset history_cnt_: %d.", droneNumber_, history_cnt_);
            history_cnt_ = 0;
            random_direction_[0] = 0;
            random_direction_[1] = 0;
            random_direction_[2] = 0;
        }
        else
            history_cnt_ ++;
    }

    int exploration_info = 0;

    // ################################################################################
    if(enable_swarm_ == SWARM_DISABLED) // set target point if not in swarm mode
    {
        ROS_INFO_ONCE("RelativeDistanceController %d swarm disabled target_swarm_=%f/%f/%f odometry_=%f/%f/%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2], odometry_.position[0], odometry_.position[1], odometry_.position[2]);
        if(inner_controller_ == 3) // velocity controller
        {
          set_point.pose.position.x = 0;
          set_point.pose.position.y = 0;
          if(odometry_.position[2] < target_swarm_.position_W[2])
              set_point.pose.position.z = 2.0;
          else
              set_point.pose.position.z = 0;
        }
        else
        {
            set_point.pose.position.x = target_swarm_.position_W[0];
            set_point.pose.position.y = target_swarm_.position_W[1];
            set_point.pose.position.z = target_swarm_.position_W[2];
        }
        set_point_marker[0] = set_point.pose.position.x;
        set_point_marker[1] = set_point.pose.position.y;
        set_point_marker[2] = set_point.pose.position.z;
    }
    else if(enable_swarm_ == SWARM_LANDING) // set keep target point and set small z if in landing mode
    {
/*        ROS_INFO_ONCE("SwarmController %d landing x=%f y=%f z=%f", droneNumber_, target_swarm_.position_W[0], target_swarm_.position_W[1], target_swarm_.position_W[2]);
        set_point.pose.position.x = odometry_.position[0];
        set_point.pose.position.y = odometry_.position[1];
        set_point.pose.position.z = max(0.0, min(odometry_.position[2] - 0.05, 0.1));*/
    }
    else if(enable_swarm_ & SWARM_SPC_DISTANCES_ONLY ||
            enable_swarm_ & SWARM_SPC_DISTANCES_ELEV ||
            enable_swarm_ & SWARM_SPC_DISTANCES_CHAIN)
    {
        if(enable_swarm_ & SWARM_SPC_DISTANCES_ONLY)
            ROS_INFO_ONCE("RelativeDistanceController %d starting swarm mode: SWARM_SPC_DISTANCES_ONLY", droneNumber_);
        else if(enable_swarm_ & SWARM_SPC_DISTANCES_ELEV)
            ROS_INFO_ONCE("RelativeDistanceController %d starting swarm mode: SWARM_SPC_DISTANCES_ELEV", droneNumber_);
        else if(enable_swarm_ & SWARM_SPC_DISTANCES_CHAIN)
            ROS_INFO_ONCE("RelativeDistanceController %d starting swarm mode: SWARM_SPC_DISTANCES_CHAIN", droneNumber_);

        // calculate neighbourhood independently from potential position
        int neighbourhood_cnt = 0;
        bool neighbourhood_bool[N_DRONES_MAX];
        for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
        {
            if(distances_filtered_[droneNumber_][i] < neighbourhood_distance_ && i != droneNumber_) // distance smaller than threshold and not own quadcopter
            {
                neighbourhood_cnt ++;
                neighbourhood_bool[i] = true;
            }
            else
                neighbourhood_bool[i] = false;
        }

        //if(!transform_ok_) // need to do exploration
        if(!transform_ok_ && !(enable_swarm_ & SWARM_USE_GROUND_TRUTH)) // need to do exploration (no exploration if ground truth data used for debugging)
        {
            if(random_direction_.norm() <= 0.001) // need new random value, if value was cleared
            {
                std::normal_distribution<float> dist(0.0, 1); // gaussian random number generator
                random_direction_[0] = dist(generator_);
                random_direction_[1] = dist(generator_);
                random_direction_[2] = dist(generator_);
            }
            exploration_info = 18;
            Vector3f direction = {1, 1, 1};
            if(unit_vectors_[0].norm() <= 0.02 || unit_vectors_age_[0] < 0) // all unit vectors empty, since they are populated sequentially
            {
                direction = random_direction_; // go to random direction
                exploration_info = 5;
            }
            else if(unit_vectors_[1].norm() <= 0.02 || unit_vectors_age_[1] < 0) // only vector 0 is not empty
            {
                Vector3f tmp = unit_vectors_[0] + random_direction_;
                direction = unit_vectors_[0].cross(tmp); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                exploration_info = 6;
            }
            else if(unit_vectors_[2].norm() <= 0.02 || unit_vectors_age_[2] < 0) // vector 0 and 1 are not empty
            {
                if(abs(unit_vectors_[0].dot(unit_vectors_[1])) > 0.99) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors go into the same direction, need some orthogonal move
                    Vector3f tmp = unit_vectors_[1] + random_direction_;
                    direction = unit_vectors_[0].cross(tmp); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 7;
                }
                else
                { // vectors go into different directions, use them to calculate orthogonal move
                    direction = unit_vectors_[0].cross(unit_vectors_[1]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 8;
                }
            }
            else // vector 0, 1 and 2 are not empty
            {
                if(abs(unit_vectors_[0].dot(unit_vectors_[1])) < 0.5) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 0 and 1 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[0].cross(unit_vectors_[1]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 10;
                }
                else if(abs(unit_vectors_[1].dot(unit_vectors_[2])) < 0.5) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 1 and 2 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[1].cross(unit_vectors_[2]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 11;
                }
                else if(abs(unit_vectors_[2].dot(unit_vectors_[0])) < 0.5) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 2 and 0 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[2].cross(unit_vectors_[0]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 12;
                }
                else if(abs(unit_vectors_[0].dot(unit_vectors_[1])) < 0.9) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 0 and 1 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[0].cross(unit_vectors_[1]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 13;
                }
                else if(abs(unit_vectors_[1].dot(unit_vectors_[2])) < 0.9) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 1 and 2 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[1].cross(unit_vectors_[2]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 14;
                }
                else if(abs(unit_vectors_[2].dot(unit_vectors_[0])) < 0.9) // dot-procut: same direction = 1; orthogonal = 0
                { // vectors 2 and 0 are good enough, use to calculate orthogonal move
                    direction = unit_vectors_[2].cross(unit_vectors_[0]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    exploration_info = 15;
                }
                else
                { // this should not happen, den all of the three vectors are nearly parallel
                    direction = unit_vectors_[0].cross(unit_vectors_[1]); // get direction by cross-product, s.t. it is orthogonal to unit_vectors_[0]
                    if(odometry_.position[2] > 0.1) // only report this, if ground truth position z-axis is above 10cm. otherwise the drone is obviously crashed.
                        ROS_INFO("RelativeDistanceController %d all unit_vectors_ dot products >= 0.9", droneNumber_);
                    // ROS_INFO("RelativeDistanceController %d unit_vectors_[0].dot(unit_vectors_[1]):%f", droneNumber_, unit_vectors_[0].dot(unit_vectors_[1]));
                    // ROS_INFO("RelativeDistanceController %d unit_vectors_[1].dot(unit_vectors_[2]):%f", droneNumber_, unit_vectors_[1].dot(unit_vectors_[2]));
                    // ROS_INFO("RelativeDistanceController %d unit_vectors_[2].dot(unit_vectors_[0]):%f", droneNumber_, unit_vectors_[2].dot(unit_vectors_[0]));
                    exploration_info = 17;
                }
            }

            direction = direction / direction.norm(); // calculate unit vector of length 1

            if(transform_available_) // check if opposite direction might be more useful (less collision probability but still going towards target location)
            {
                Vector3f potential_movement_transformed_positive = transform_vectors_ * (direction * eps_move_*2);
                Vector3f potential_movement_transformed_negative = transform_vectors_ * ((direction * -1) * eps_move_*2);

                float separation_sum_positive = 0;
                float separation_sum_negative = 0;
                for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
                {
                   float dist_positive = distances_filtered_[droneNumber_][i] +
                              distances_differences_[0][i] * potential_movement_transformed_positive[0] +
                              distances_differences_[1][i] * potential_movement_transformed_positive[1] +
                              distances_differences_[2][i] * potential_movement_transformed_positive[2];
                   float dist_negative = distances_filtered_[droneNumber_][i] +
                              distances_differences_[0][i] * potential_movement_transformed_negative[0] +
                              distances_differences_[1][i] * potential_movement_transformed_negative[1] +
                              distances_differences_[2][i] * potential_movement_transformed_negative[2];

                    if(i == droneNumber_) // skip for own quadcopter
                        continue;

                    if(neighbourhood_bool[i]) // separation only calculated for quadcopters in neighbourhood
                    {
                        separation_sum_positive += 1.0/pow(fmax(0.000001, dist_positive - 2*drone_radius_ - extra_separation_distance_), 2);
                        separation_sum_negative += 1.0/pow(fmax(0.000001, dist_negative - 2*drone_radius_ - extra_separation_distance_), 2);
                    }
                }

                float dist_beacon0_positive = beacons_filtered_[droneNumber_][current_target_] +
                             beacons_differences_[0][current_target_] * potential_movement_transformed_positive[0] +
                             beacons_differences_[1][current_target_] * potential_movement_transformed_positive[1] +
                             beacons_differences_[2][current_target_] * potential_movement_transformed_positive[2];
                float dist_beacon0_negative = beacons_filtered_[droneNumber_][current_target_] +
                             beacons_differences_[0][current_target_] * potential_movement_transformed_negative[0] +
                             beacons_differences_[1][current_target_] * potential_movement_transformed_negative[1] +
                             beacons_differences_[2][current_target_] * potential_movement_transformed_negative[2];

                float total_sum_positive;
                float total_sum_negative;
                if(neighbourhood_cnt != 0) // no neighbours means there would be a division by 0
                {
                    total_sum_positive = spc_separation_weight_ * separation_sum_positive / ((float)neighbourhood_cnt) + spc_target_weight_ * dist_beacon0_positive;
                    total_sum_negative = spc_separation_weight_ * separation_sum_negative / ((float)neighbourhood_cnt) + spc_target_weight_ * dist_beacon0_negative;
                }
                else
                {
                    total_sum_positive = spc_target_weight_ * dist_beacon0_positive;
                    total_sum_negative = spc_target_weight_ * dist_beacon0_negative;
                }

                if( total_sum_positive < total_sum_negative )
                {
                    ROS_INFO_ONCE("RelativeDistanceController %d exploration positive better (%f*%f+%f*%f < %f*%f+%f*%f)", droneNumber_,
                      spc_separation_weight_, separation_sum_positive, spc_target_weight_, dist_beacon0_positive,
                      spc_separation_weight_, separation_sum_negative, spc_target_weight_, dist_beacon0_negative
                    );
                }
                else
                {
                    ROS_INFO_ONCE("RelativeDistanceController %d exploration negative better (%f*%f+%f*%f >= %f*%f+%f*%f)", droneNumber_,
                      spc_separation_weight_, separation_sum_positive, spc_target_weight_, dist_beacon0_positive,
                      spc_separation_weight_, separation_sum_negative, spc_target_weight_, dist_beacon0_negative
                    );
                    direction = direction * -1; // invert exploration vector
                }
            }

            direction = direction * explore_command_length_; // length for exploration vector

            if(inner_controller_ == 3) // velocity controller
            {
                set_point.pose.position.x = direction[0] * velocity_scaling_;
                set_point.pose.position.y = direction[1] * velocity_scaling_;
                set_point.pose.position.z = direction[2] * velocity_scaling_;
                ROS_INFO_ONCE("RelativeDistanceController %d explore:%d (velocity) direction:%s", droneNumber_, exploration_info, VectorToString(direction).c_str());
                set_point_marker[0] = odometry_.position[0] + direction[0] * velocity_scaling_;
                set_point_marker[1] = odometry_.position[1] + direction[1] * velocity_scaling_;
                set_point_marker[2] = odometry_.position[2] + direction[2] * velocity_scaling_;
            }
            else if(inner_controller_ == 4) // HW controller
            {
                set_point.pose.position.x = direction[0] * velocity_scaling_;
                set_point.pose.position.y = direction[1] * velocity_scaling_;
                set_point.pose.position.z = odometry_.position[2] + direction[2]*0.05;
                ROS_INFO_ONCE("RelativeDistanceController %d explore:%d (velocity) direction:%s", droneNumber_, exploration_info, VectorToString(direction).c_str());
                set_point_marker[0] = odometry_.position[0] + direction[0] * velocity_scaling_;
                set_point_marker[1] = odometry_.position[1] + direction[1] * velocity_scaling_;
                set_point_marker[2] = odometry_.position[2] + direction[2] * velocity_scaling_;
            }
            else
            {
                set_point.pose.position.x = odometry_.position[0] + direction[0];
                set_point.pose.position.y = odometry_.position[1] + direction[1];
                set_point.pose.position.z = odometry_.position[2] + direction[2];
                ROS_INFO_ONCE("RelativeDistanceController %d explore:%d direction:%s", droneNumber_, exploration_info, VectorToString(direction).c_str());
                set_point_marker[0] = set_point.pose.position.x;
                set_point_marker[1] = set_point.pose.position.y;
                set_point_marker[2] = set_point.pose.position.z;
            }
            tempEnv << exploration_info << "," << direction[0] << "," << direction[1] << "," << direction[2] << "," << -1 << "," << history_cnt_ << ",";
        }
        else if(enable_swarm_ & SWARM_USE_ML) // use N-step prediction instead of matrix transformed estimation
        {
            Vector3f best_movement;
            float best_sum = FLT_MAX;
            float min_coehesion_term;
            float min_separation_term;
            float min_target_term;
            float min_calm_term;
            float min_height_term;

            float still_sum = FLT_MAX;
            float still_coehesion_term;
            float still_separation_term;
            float still_target_term;
            float still_calm_term;
            float still_height_term;

            for(int ai = 0; ai <= n_move_max_; ai ++) // iterate over all possible next actions in x-, y- and z-dimension; and over length n_move_max_
            {
                for(int xi = -1; xi <= 1; xi ++)
                {
                    for(int yi = -1; yi <= 1; yi ++)
                    {
                        for(int zi = -1; zi <= 1; zi ++)
                        {
                            Vector3f potential_movement = {(float)xi, (float)yi, (float)zi}; // 3-dimensional vector of movement
                            float potential_movement_norm = potential_movement.norm();
                            if(xi != 0 || yi != 0 || zi != 0)
                                potential_movement = (potential_movement / potential_movement_norm) * eps_move_ * ai; // normalize movement vector to get length 1 and then scale by desired length

                            float cohesion_sum = 0;
                            float separation_sum = 0;
                            float total_sum = 0;

                            for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
                            {
                                if(i == droneNumber_) // skip for own quadcopter
                                    continue;

                                float dist = distances_filtered_[droneNumber_][i];
                                c10::IValue hx = distances_hx_[i];

                                for (size_t k = 0; k < N_STEPS; k++) // predict for N steps
                                {
                                    torch::Tensor tensor = torch::tensor({{(float)(potential_movement[0] / (float)1),
                                                                           (float)(potential_movement[1] / (float)1),
                                                                           (float)(potential_movement[2] / (float)1),
                                                                           dist}}, {torch::kFloat32});
                                    auto ret = distances_model_[i].forward({tensor, hx}).toTuple();
                                    torch::Tensor outputs = ret->elements()[0].toTensor();
                                    dist = outputs[0].item<float>();
                                    hx = ret->elements()[1];
                                }

                                float dist_gt = dronestate_[i].GetDistance_sim_gt(&dronestate_[droneNumber_], potential_movement);
                                if(enable_swarm_ & SWARM_USE_GROUND_TRUTH) // only for debug! using ground truth positions to infer distances.
                                    dist = dist_gt;

//                                    ROS_INFO("dr.%d (%2d/%2d/%2d|%2d) cur=%f est=%f gt=%f", droneNumber_, xi, yi, zi, ai, distances_filtered_[droneNumber_][i], dist, dist_gt);

                                if(neighbourhood_bool[i]) // separation and cohesion only calculated for quadcopters in neighbourhood
                                {
                                    cohesion_sum += pow(dist, 2);
                                    separation_sum += 1.0/pow(fmax(0.000001, dist - 2*drone_radius_ - extra_separation_distance_), 2);
                                }
                                ROS_INFO_ONCE("dr.%d (%2d/%2d/%2d|%2d) i=%d, dist=%f, dist_gt=%f, cohesion_sum=%f, separation_sum=%f", droneNumber_, xi, yi, zi, ai, (int)i, dist, dist_gt, cohesion_sum, separation_sum);
                            }

                            float dist_beacon[N_BEACONS_MAX], dist_gt_beacon[N_BEACONS_MAX];
                            for (size_t j = 0; j < beaconCount_; j++) // iterate over all beacons
                            {
                                dist_beacon[j] = beacons_filtered_[droneNumber_][j];
                                c10::IValue hx = beacons_hx_[j];

                                for (size_t k = 0; k < N_STEPS; k++) // predict for N steps
                                {
                                    torch::Tensor tensor = torch::tensor({{(float)(potential_movement[0] / (float)1),
                                                                           (float)(potential_movement[1] / (float)1),
                                                                           (float)(potential_movement[2] / (float)1),
                                                                           dist_beacon[j]}}, {torch::kFloat32});
                                    auto ret = beacons_model_[j].forward({tensor, hx}).toTuple();
                                    torch::Tensor outputs = ret->elements()[0].toTensor();
                                    dist_beacon[j] = outputs[0].item<float>();
                                    hx = ret->elements()[1];
                                }

                                dist_gt_beacon[j] = sqrt(pow(beacon_gt_[j][0] - (odometry_.position[0] + potential_movement[0]), 2) +
                                                         pow(beacon_gt_[j][1] - (odometry_.position[1] + potential_movement[1]), 2) +
                                                         pow(beacon_gt_[j][2] - (odometry_.position[2] + potential_movement[2]), 2));

                                // ROS_INFO("dr.%d bc.%d (%2d/%2d/%2d|%2d) dist_beacon=%f, dist_gt_beacon=%f", droneNumber_, (int)j, xi, yi, zi, ai, dist_beacon[j], dist_gt_beacon[j]);
                            }

                            if(enable_swarm_ & SWARM_USE_GROUND_TRUTH) // only for debug! using ground truth positions to infer distances.
                               for (size_t j = 0; j < beaconCount_; j++) // iterate over all beacons
                                  dist_beacon[j] = dist_gt_beacon[j];

                            float target_sum;
                            if(enable_swarm_ & SWARM_SPC_DISTANCES_ONLY || enable_swarm_ & SWARM_SPC_DISTANCES_ELEV)
                                target_sum = fabs(dist_beacon[current_target_]);
                            if(enable_swarm_ & SWARM_SPC_DISTANCES_CHAIN)
                                target_sum = fabs(dist_beacon[0]) + fabs(dist_beacon[1]);

                            float height_diff = swarm_elevation_ - (elevation_filtered_[droneNumber_] + potential_movement[2]);
                            float height_sum = height_diff*height_diff;

                            float coehesion_term = spc_cohesion_weight_ * cohesion_sum / ((float)neighbourhood_cnt);
                            float separation_term = spc_separation_weight_ * separation_sum / ((float)neighbourhood_cnt);
                            float target_term = spc_target_weight_ * target_sum;
                            float calm_term = spc_calm_weight_ * potential_movement.norm();
                            float height_term = 0;

                            if(enable_swarm_ & SWARM_SPC_DISTANCES_ELEV)
                                height_term += spc_height_weight_ * height_sum;

                            if(neighbourhood_cnt != 0) // no neighbours means there was a division by 0
                                total_sum = coehesion_term + separation_term + target_term + calm_term + height_term;
                            else
                                total_sum = target_term + calm_term + height_term;

                            ROS_INFO_ONCE("dr.%d (%2d/%2d/%2d|%2d) coh=%7.1f sep=%7.1f tar=%7.1f calm=%7.1f total=%7.1f len=%f", droneNumber_, xi, yi, zi, ai, coehesion_term, separation_term, target_term, calm_term, total_sum, potential_movement.norm());
                            //ROS_INFO("dr.%d (%2d/%2d/%2d|%2d) tar=%f", droneNumber_, xi, yi, zi, ai, target_sum);

                            if(total_sum < best_sum)
                            {
                                best_sum = total_sum;

                                best_movement = potential_movement; // save movement vector

                                min_coehesion_term = coehesion_term;
                                min_separation_term = separation_term;
                                min_target_term = target_term;
                                min_calm_term = calm_term;
                                min_height_term = height_term;
                            }

                            if(xi == 0 && yi == 0 && zi == 0)
                            {
                                still_sum = total_sum;

                                still_coehesion_term = coehesion_term;
                                still_separation_term = separation_term;
                                still_target_term = target_term;
                                still_calm_term = calm_term;
                                still_height_term = height_term;
                            }

                        }
                    }
                }
            }

            ROS_INFO_ONCE("drone%d coh=%7.1f sep=%7.1f tar=%7.1f calm=%7.1f total=%7.1f", droneNumber_, min_coehesion_term, min_separation_term, min_target_term, min_calm_term, best_sum);

            if(inner_controller_ == 3) // velocity controller
            {
                set_point.pose.position.x = best_movement(0) * velocity_scaling_;
                set_point.pose.position.y = best_movement(1) * velocity_scaling_;
                set_point.pose.position.z = best_movement(2) * velocity_scaling_;
                ROS_INFO_ONCE("RelativeDistanceController %d exploitation (velocity) tsum=%f scal=%f", droneNumber_, best_sum, velocity_scaling_);
                set_point_marker[0] = odometry_.position[0] + best_movement(0) * velocity_scaling_;
                set_point_marker[1] = odometry_.position[1] + best_movement(1) * velocity_scaling_;
                set_point_marker[2] = odometry_.position[2] + best_movement(2) * velocity_scaling_;
                tempEnv << exploration_info << "," << best_movement(0) * velocity_scaling_ << "," << best_movement(1) * velocity_scaling_ << "," << best_movement(2) * velocity_scaling_ << "," << (float)best_sum << "," << history_cnt_ << ",";
            }
            else if(inner_controller_ == 4) // HW controller
            {
                set_point.pose.position.x = best_movement(0) * velocity_scaling_;
                set_point.pose.position.y = best_movement(1) * velocity_scaling_;
                set_point.pose.position.z = odometry_.position[2] + best_movement(2)*0.5; // TODO: proper scaling
                ROS_INFO_ONCE("RelativeDistanceController %d exploitation (velocity) tsum=%f scal=%f", droneNumber_, best_sum, velocity_scaling_);
                set_point_marker[0] = odometry_.position[0] + best_movement(0) * velocity_scaling_;
                set_point_marker[1] = odometry_.position[1] + best_movement(1) * velocity_scaling_;
                set_point_marker[2] = odometry_.position[2] + best_movement(2) * velocity_scaling_;
                tempEnv << exploration_info << "," << best_movement(0) * velocity_scaling_ << "," << best_movement(1) * velocity_scaling_ << "," << best_movement(2) * velocity_scaling_ << "," << (float)best_sum << "," << history_cnt_ << ",";
            }
            else
            {
                set_point.pose.position.x = odometry_.position[0] + best_movement(0);
                set_point.pose.position.y = odometry_.position[1] + best_movement(1);
                set_point.pose.position.z = odometry_.position[2] + best_movement(2)*1.5; // TODO: proper scaling
                ROS_INFO_ONCE("RelativeDistanceController %d exploitation tsum=%f", droneNumber_, best_sum);
                set_point_marker[0] = set_point.pose.position.x;
                set_point_marker[1] = set_point.pose.position.y;
                set_point_marker[2] = set_point.pose.position.z;
                tempEnv << exploration_info << "," << best_movement(0) << "," << best_movement(1) << "," << best_movement(2)*1.5 << "," << (float)best_sum << "," << history_cnt_ << ",";
            }
            tempCost << best_sum << "," << min_coehesion_term << "," << min_separation_term << "," << min_target_term << "," << min_calm_term << "," << min_height_term << ",";
            tempCost << still_sum << "," << still_coehesion_term << "," << still_separation_term << "," << still_target_term << "," << still_calm_term << "," << still_height_term << ",";
        }
        else // possible to do exploitation
        {
            Vector3f best_movement;
            float best_sum = FLT_MAX;
            float min_coehesion_term;
            float min_separation_term;
            float min_target_term;
            float min_calm_term;
            float min_height_term;

            float still_sum = FLT_MAX;
            float still_coehesion_term;
            float still_separation_term;
            float still_target_term;
            float still_calm_term;
            float still_height_term;

            for(int ai = 0; ai <= n_move_max_; ai ++) // iterate over all possible next actions in x-, y- and z-dimension; and over length n_move_max_
            {
                for(int xi = -1; xi <= 1; xi ++)
                {
                    for(int yi = -1; yi <= 1; yi ++)
                    {
                        for(int zi = -1; zi <= 1; zi ++)
                        {
                            Vector3f potential_movement = {(float)xi, (float)yi, (float)zi}; // 3-dimensional vector of movement
                            float potential_movement_norm = potential_movement.norm();
                            if(xi != 0 || yi != 0 || zi != 0)
                                potential_movement = (potential_movement / potential_movement_norm) * eps_move_ * ai; // normalize movement vector to get length 1 and then scale by desired length
                            Vector3f potential_movement_transformed = transform_vectors_ * potential_movement;

                            float cohesion_sum = 0;
                            float separation_sum = 0;
                            float total_sum = 0;

                            for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
                            {
                                if(i == droneNumber_) // skip for own quadcopter
                                    continue;

                                float dist = distances_filtered_[droneNumber_][i] +
                                             distances_differences_[0][i] * potential_movement_transformed[0] +
                                             distances_differences_[1][i] * potential_movement_transformed[1] +
                                             distances_differences_[2][i] * potential_movement_transformed[2];

                                float dist_gt = dronestate_[i].GetDistance_sim_gt(&dronestate_[droneNumber_], potential_movement);
                                if(enable_swarm_ & SWARM_USE_GROUND_TRUTH) // only for debug! using ground truth positions to infer distances.
                                    dist = dist_gt;

                                if(neighbourhood_bool[i]) // separation and cohesion only calculated for quadcopters in neighbourhood
                                {
                                    cohesion_sum += pow(dist, 2);
                                    separation_sum += 1.0/pow(fmax(0.000001, dist - 2*drone_radius_ - extra_separation_distance_), 2);
                                }
                                ROS_INFO_ONCE("dr.%d (%2d/%2d/%2d|%2d) i=%d, dist=%f, dist_gt=%f, cohesion_sum=%f, separation_sum=%f", droneNumber_, xi, yi, zi, ai, (int)i, dist, dist_gt, cohesion_sum, separation_sum);
                            }

                            float dist_beacon[N_BEACONS_MAX], dist_gt_beacon[N_BEACONS_MAX];
                            for (size_t j = 0; j < beaconCount_; j++) // iterate over all beacons
                            {
                                dist_beacon[j] = beacons_filtered_[droneNumber_][j] +
                                             beacons_differences_[0][j] * potential_movement_transformed[0] +
                                             beacons_differences_[1][j] * potential_movement_transformed[1] +
                                             beacons_differences_[2][j] * potential_movement_transformed[2];
                                dist_gt_beacon[j] = sqrt(pow(beacon_gt_[j][0] - (odometry_.position[0] + potential_movement[0]), 2) +
                                                         pow(beacon_gt_[j][1] - (odometry_.position[1] + potential_movement[1]), 2) +
                                                         pow(beacon_gt_[j][2] - (odometry_.position[2] + potential_movement[2]), 2));

                                // ROS_INFO("dr.%d bc.%d (%2d/%2d/%2d|%2d) dist_beacon=%f, dist_gt_beacon=%f", droneNumber_, (int)j, xi, yi, zi, ai, dist_beacon[j], dist_gt_beacon[j]);
                            }

                            if(enable_swarm_ & SWARM_SPC_DISTANCES_ELEV)
                            {
                                for (size_t j = 0; j < beaconCount_; j++) // iterate over all beacons
                                {
                                    float elevation_over_beacon = elevation_filtered_[droneNumber_] + potential_movement[2] - target_elevation_;
                                    float dist_projected_beacon_squared = pow(dist_beacon[j], 2) - pow(elevation_over_beacon, 2);
                                    if(dist_projected_beacon_squared > 0)
                                        dist_beacon[j] = sqrt(dist_projected_beacon_squared);
                                    else
                                        dist_beacon[j] = 0;

                                    float elevation_gt_over_beacon = odometry_.position[2] + potential_movement[2] - target_elevation_;
                                    float dist_gt_projected_beacon_squared = pow(dist_gt_beacon[j], 2) - pow(elevation_gt_over_beacon, 2);
                                    if(dist_gt_projected_beacon_squared > 0)
                                        dist_gt_beacon[j] = sqrt(dist_gt_projected_beacon_squared);
                                    else
                                        dist_gt_beacon[j] = 0;

                                    ROS_INFO_ONCE("dr.%d bc.%d (%2d/%2d/%2d|%2d) elevation_over_beacon=%f, dist_beacon=%f, elevation_gt_over_beacon=%f, dist_gt_beacon=%f", droneNumber_, (int)j, xi, yi, zi, ai, elevation_over_beacon, dist_beacon[j], elevation_gt_over_beacon, dist_gt_beacon[j]);
                                }
                            }

                            if(enable_swarm_ & SWARM_USE_GROUND_TRUTH) // only for debug! using ground truth positions to infer distances.
                               for (size_t j = 0; j < beaconCount_; j++) // iterate over all beacons
                                  dist_beacon[j] = dist_gt_beacon[j];

                            float target_sum;
                            if(enable_swarm_ & SWARM_SPC_DISTANCES_ONLY || enable_swarm_ & SWARM_SPC_DISTANCES_ELEV)
                                target_sum = fabs(dist_beacon[current_target_]);
                            if(enable_swarm_ & SWARM_SPC_DISTANCES_CHAIN)
                                target_sum = fabs(dist_beacon[0]) + fabs(dist_beacon[1]);

                            //if(enable_swarm_ & SWARM_SPC_DISTANCES_CHAIN)
                            //    ROS_INFO("dr.%d dist_beacon[0]=%f, dist_beacon[1]=%f, cnt=%d", droneNumber_, dist_beacon[0], dist_beacon[1], beaconCount_);

                            float height_diff = swarm_elevation_ - (elevation_filtered_[droneNumber_] + potential_movement[2]);
                            float height_sum = height_diff*height_diff;

                            float coehesion_term = spc_cohesion_weight_ * cohesion_sum / ((float)neighbourhood_cnt);
                            float separation_term = spc_separation_weight_ * separation_sum / ((float)neighbourhood_cnt);
                            float target_term = spc_target_weight_ * target_sum;
                            float calm_term = spc_calm_weight_ * potential_movement.norm();
                            float height_term = 0;

                            if(enable_swarm_ & SWARM_SPC_DISTANCES_ELEV)
                                height_term += spc_height_weight_ * height_sum;

                            if(spc_limit_weight_ > 0.5)
                            {
                                if((elevation_filtered_[droneNumber_] + potential_movement[2]) > (float)2.0) // limit max height
                                {
                                  height_term += spc_limit_weight_ * potential_movement[2];
                                  //ROS_INFO("dr.%d (%2d/%2d/%2d|%2d) limit max plus=%f (z=%f)", droneNumber_, xi, yi, zi, ai, spc_limit_weight_ * potential_movement[2], elevation_filtered_[droneNumber_]);
                                }
                                if((elevation_filtered_[droneNumber_] + potential_movement[2]) < (float)0.5) // limit min height
                                {
                                  //ROS_INFO("dr.%d (%2d/%2d/%2d|%2d) limit min plus=%f (z=%f)", droneNumber_, xi, yi, zi, ai, spc_limit_weight_ * potential_movement[2], elevation_filtered_[droneNumber_]);
                                  height_term += spc_limit_weight_ * (-1.0) * potential_movement[2];
                                }
                            }

                            if(neighbourhood_cnt != 0) // no neighbours means there was a division by 0
                                total_sum = coehesion_term + separation_term + target_term + calm_term + height_term;
                            else
                                total_sum = target_term + calm_term + height_term;

                            ROS_INFO_ONCE("dr.%d (%2d/%2d/%2d|%2d) coh=%7.1f sep=%7.1f tar=%7.1f calm=%7.1f total=%7.1f len=%f", droneNumber_, xi, yi, zi, ai, coehesion_term, separation_term, target_term, calm_term, total_sum, potential_movement.norm());
                            //ROS_INFO("dr.%d (%2d/%2d/%2d|%2d) tar=%f", droneNumber_, xi, yi, zi, ai, target_sum);

                            if(total_sum < best_sum)
                            {
                                best_sum = total_sum;

                                best_movement = potential_movement; // save movement vector

                                min_coehesion_term = coehesion_term;
                                min_separation_term = separation_term;
                                min_target_term = target_term;
                                min_calm_term = calm_term;
                                min_height_term = height_term;
                            }

                            if(xi == 0 && yi == 0 && zi == 0)
                            {
                                still_sum = total_sum;

                                still_coehesion_term = coehesion_term;
                                still_separation_term = separation_term;
                                still_target_term = target_term;
                                still_calm_term = calm_term;
                                still_height_term = height_term;
                            }

                        }
                    }
                }
            }

            ROS_INFO_ONCE("drone%d coh=%7.1f sep=%7.1f tar=%7.1f calm=%7.1f total=%7.1f", droneNumber_, min_coehesion_term, min_separation_term, min_target_term, min_calm_term, best_sum);

            if(inner_controller_ == 3) // velocity controller
            {
                set_point.pose.position.x = best_movement(0) * velocity_scaling_;
                set_point.pose.position.y = best_movement(1) * velocity_scaling_;
                set_point.pose.position.z = best_movement(2) * velocity_scaling_;
                ROS_INFO_ONCE("RelativeDistanceController %d exploitation (velocity) tsum=%f scal=%f", droneNumber_, best_sum, velocity_scaling_);
                set_point_marker[0] = odometry_.position[0] + best_movement(0) * velocity_scaling_;
                set_point_marker[1] = odometry_.position[1] + best_movement(1) * velocity_scaling_;
                set_point_marker[2] = odometry_.position[2] + best_movement(2) * velocity_scaling_;
                tempEnv << exploration_info << "," << best_movement(0) * velocity_scaling_ << "," << best_movement(1) * velocity_scaling_ << "," << best_movement(2) * velocity_scaling_ << "," << (float)best_sum << "," << history_cnt_ << ",";
            }
            else if(inner_controller_ == 4) // HW controller
            {
                set_point.pose.position.x = best_movement(0) * velocity_scaling_;
                set_point.pose.position.y = best_movement(1) * velocity_scaling_;
                set_point.pose.position.z = odometry_.position[2] + best_movement(2)*0.5; // TODO: proper scaling
                ROS_INFO_ONCE("RelativeDistanceController %d exploitation (velocity) tsum=%f scal=%f", droneNumber_, best_sum, velocity_scaling_);
                set_point_marker[0] = odometry_.position[0] + best_movement(0) * velocity_scaling_;
                set_point_marker[1] = odometry_.position[1] + best_movement(1) * velocity_scaling_;
                set_point_marker[2] = odometry_.position[2] + best_movement(2) * velocity_scaling_;
                tempEnv << exploration_info << "," << best_movement(0) * velocity_scaling_ << "," << best_movement(1) * velocity_scaling_ << "," << best_movement(2) * velocity_scaling_ << "," << (float)best_sum << "," << history_cnt_ << ",";
            }
            else
            {
                set_point.pose.position.x = odometry_.position[0] + best_movement(0);
                set_point.pose.position.y = odometry_.position[1] + best_movement(1);
                set_point.pose.position.z = odometry_.position[2] + best_movement(2)*1.5; // TODO: proper scaling
                ROS_INFO_ONCE("RelativeDistanceController %d exploitation tsum=%f", droneNumber_, best_sum);
                set_point_marker[0] = set_point.pose.position.x;
                set_point_marker[1] = set_point.pose.position.y;
                set_point_marker[2] = set_point.pose.position.z;
                tempEnv << exploration_info << "," << best_movement(0) << "," << best_movement(1) << "," << best_movement(2)*1.5 << "," << (float)best_sum << "," << history_cnt_ << ",";
            }
            tempCost << best_sum << "," << min_coehesion_term << "," << min_separation_term << "," << min_target_term << "," << min_calm_term << "," << min_height_term << ",";
            tempCost << still_sum << "," << still_coehesion_term << "," << still_separation_term << "," << still_target_term << "," << still_calm_term << "," << still_height_term << ",";
        }

        //tempEnv << unit_vectors_age_[0] << "," << unit_vectors_age_[1] << "," << unit_vectors_age_[2] << ",";
        tempEnv << neighbourhood_cnt << ",";

        //tempCost << unit_vectors_[0][0] << "," << unit_vectors_[0][1] << "," << unit_vectors_[0][2] << "," << unit_vectors_age_[0] << ",";
        //tempCost << unit_vectors_[1][0] << "," << unit_vectors_[1][1] << "," << unit_vectors_[1][2] << "," << unit_vectors_age_[1] << ",";
        //tempCost << unit_vectors_[2][0] << "," << unit_vectors_[2][1] << "," << unit_vectors_[2][2] << "," << unit_vectors_age_[2] << ",";
    }
    else if(enable_swarm_ & SWARM_SPC_CYCLIC)
    {
        ROS_INFO_ONCE("RelativeDistanceController %d starting swarm mode: SWARM_SPC_CYCLIC", droneNumber_);

        float window_len = 3.0;
        float window_time = std::fmod(odometry_.timeStampSec + odometry_.timeStampNsec/(double)1000000000, (double)droneCount_ * (double)window_len) - ((double)droneNumber_ * (double)window_len);

        if(window_time < 0)
            cyclic_current_phase_ = CYCLIC_PHASE_REST;
        else if(window_time > window_len)
            cyclic_current_phase_ = CYCLIC_PHASE_REST;
        else if(window_time > 0.0 && cyclic_current_phase_ == CYCLIC_PHASE_REST)
        {
            cyclic_current_phase_ = CYCLIC_PHASE_IDENTIFY_A;
            cyclic_odometry_history0_ = odometry_; // save absolute position, but use relative position only for calculation
            for (size_t i = 0; i < droneCount_; i++)
                cyclic_distances_history0_[i] = distances_[droneNumber_][i];
            cyclic_distances_history0_[droneNumber_] = beacons_[droneNumber_][0]; // quickfix to use own drone index for beacon0
        }
        else if(window_time > 0.4 && cyclic_current_phase_ == CYCLIC_PHASE_IDENTIFY_A)
        {
            cyclic_current_phase_ = CYCLIC_PHASE_IDENTIFY_B;
            cyclic_odometry_history1_ = odometry_; // save absolute position, but use relative position only for calculation
            for (size_t i = 0; i < droneCount_; i++)
                cyclic_distances_history1_[i] = distances_[droneNumber_][i];
            cyclic_distances_history1_[droneNumber_] = beacons_[droneNumber_][0]; // quickfix to use own drone index for beacon0
        }
        else if(window_time > 0.8 && cyclic_current_phase_ == CYCLIC_PHASE_IDENTIFY_B)
        {
            cyclic_current_phase_ = CYCLIC_PHASE_IDENTIFY_C;
            cyclic_odometry_history2_ = odometry_; // save absolute position, but use relative position only for calculation
            for (size_t i = 0; i < droneCount_; i++)
                cyclic_distances_history2_[i] = distances_[droneNumber_][i];
            cyclic_distances_history2_[droneNumber_] = beacons_[droneNumber_][0]; // quickfix to use own drone index for beacon0
        }
        else if(window_time > 1.4 && cyclic_current_phase_ == CYCLIC_PHASE_IDENTIFY_C)
        {
            cyclic_current_phase_ = CYCLIC_PHASE_CONTROL;
            cyclic_odometry_history3_ = odometry_; // save absolute position, but use relative position only for calculation
            for (size_t i = 0; i < droneCount_; i++)
                cyclic_distances_history3_[i] = distances_[droneNumber_][i];
            cyclic_distances_history3_[droneNumber_] = beacons_[droneNumber_][0]; // quickfix to use own drone index for beacon0

            Vector3f positions_try_a[N_DRONES_MAX];
            Vector3f positions_try_b[N_DRONES_MAX];
            Vector3f positions_try_c[N_DRONES_MAX];
            Vector3f positions_try_d[N_DRONES_MAX];
            float quality_try_a[N_DRONES_MAX];
            float quality_try_b[N_DRONES_MAX];
            float quality_try_c[N_DRONES_MAX];
            float quality_try_d[N_DRONES_MAX];

            TrilaterationCalculation( &cyclic_odometry_history0_,
                                      &cyclic_odometry_history1_,
                                      &cyclic_odometry_history2_,
                                      &cyclic_odometry_history3_,
                                      cyclic_distances_history0_,
                                      cyclic_distances_history1_,
                                      cyclic_distances_history2_,
                                      cyclic_distances_history3_,
                                      &cyclic_odometry_history3_,
                                      positions_try_a,
                                      quality_try_a);

            TrilaterationCalculation( &cyclic_odometry_history1_,
                                      &cyclic_odometry_history2_,
                                      &cyclic_odometry_history3_,
                                      &cyclic_odometry_history0_,
                                      cyclic_distances_history1_,
                                      cyclic_distances_history2_,
                                      cyclic_distances_history3_,
                                      cyclic_distances_history0_,
                                      &cyclic_odometry_history3_,
                                      positions_try_b,
                                      quality_try_b);

            TrilaterationCalculation( &cyclic_odometry_history2_,
                                      &cyclic_odometry_history3_,
                                      &cyclic_odometry_history0_,
                                      &cyclic_odometry_history1_,
                                      cyclic_distances_history2_,
                                      cyclic_distances_history3_,
                                      cyclic_distances_history0_,
                                      cyclic_distances_history1_,
                                      &cyclic_odometry_history3_,
                                      positions_try_c,
                                      quality_try_c);

            TrilaterationCalculation( &cyclic_odometry_history3_,
                                      &cyclic_odometry_history2_,
                                      &cyclic_odometry_history1_,
                                      &cyclic_odometry_history0_,
                                      cyclic_distances_history3_,
                                      cyclic_distances_history2_,
                                      cyclic_distances_history1_,
                                      cyclic_distances_history0_,
                                      &cyclic_odometry_history3_,
                                      positions_try_d,
                                      quality_try_d);

            for (size_t i = 0; i < droneCount_; i++)
            {
                //if(i == droneNumber_) // skip for own quadcopter
                //    continue;
                // quickfix: do not skip, as this contains the distance to beacon0

                float best_quality = quality_try_a[i];
                cyclic_positions_est_[i] = positions_try_a[i];

                if(quality_try_b[i] < best_quality)
                {
                    best_quality = quality_try_b[i];
                    cyclic_positions_est_[i] = positions_try_b[i];
                }
                if(quality_try_c[i] < best_quality)
                {
                    best_quality = quality_try_c[i];
                    cyclic_positions_est_[i] = positions_try_c[i];
                }
                if(quality_try_d[i] < best_quality)
                {
                    best_quality = quality_try_d[i];
                    cyclic_positions_est_[i] = positions_try_d[i];
                }
                ROS_INFO("(%d) cyclic_positions_est_[%d] %s (quality: %f)", droneNumber_, (int)i, VectorToString(cyclic_positions_est_[i]).c_str(), best_quality);
            }
            // ROS_INFO("############################");
        }

        if(cyclic_current_phase_ == CYCLIC_PHASE_IDENTIFY_A)
        {
            set_point.pose.position.x = 0.0; // * velocity_scaling_;
            set_point.pose.position.y = 0.0;
            set_point.pose.position.z = 1.0;
            set_point_marker[0] = odometry_.position[0];
            set_point_marker[1] = odometry_.position[1];
            set_point_marker[2] = odometry_.position[2] + 0.5;
            exploration_info = 1;
        }
        else if(cyclic_current_phase_ == CYCLIC_PHASE_IDENTIFY_B)
        {
            set_point.pose.position.x = 0.0;
            set_point.pose.position.y = 1.0; // * velocity_scaling_;
            set_point.pose.position.z = 0.0;
            set_point_marker[0] = odometry_.position[0];
            set_point_marker[1] = odometry_.position[1] + 0.5;
            set_point_marker[2] = odometry_.position[2];
            exploration_info = 2;
        }
        else if(cyclic_current_phase_ == CYCLIC_PHASE_IDENTIFY_C)
        {
            set_point.pose.position.x = 1.0;
            set_point.pose.position.y = 0.0; // * velocity_scaling_;
            set_point.pose.position.z = 0.0;
            set_point_marker[0] = odometry_.position[0] + 0.5;
            set_point_marker[1] = odometry_.position[1];
            set_point_marker[2] = odometry_.position[2];
            exploration_info = 3;
        }
        else if(cyclic_current_phase_ == CYCLIC_PHASE_CONTROL)
        {
            Vector3f best_movement;
            float best_sum = FLT_MAX;
            float min_coehesion_term;
            float min_separation_term;
            float min_target_term;
            float min_calm_term;
            float min_height_term;

            float still_sum = FLT_MAX;
            float still_coehesion_term;
            float still_separation_term;
            float still_target_term;
            float still_calm_term;
            float still_height_term;

            int neighbourhood_cnt = droneCount_ - 1;

            for(int ai = 0; ai <= n_move_max_; ai ++) // iterate over all possible next actions in x-, y- and z-dimension; and over length n_move_max_
            {
                for(int xi = -1; xi <= 1; xi ++)
                {
                    for(int yi = -1; yi <= 1; yi ++)
                    {
                        for(int zi = -1; zi <= 1; zi ++)
                        {
                            Vector3f potential_movement = {(float)xi, (float)yi, (float)zi}; // 3-dimensional vector of movement
                            float potential_movement_norm = potential_movement.norm();
                            if(xi != 0 || yi != 0 || zi != 0)
                                potential_movement = (potential_movement / potential_movement_norm) * eps_move_ * ai; // normalize movement vector to get length 1 and then scale by desired length

                            Vector3f potential_position;
                            potential_position[0] = odometry_.position[0] + potential_movement[0];
                            potential_position[1] = odometry_.position[1] + potential_movement[1];
                            potential_position[2] = odometry_.position[2] + potential_movement[2];
                            // odometry_ would actually be replaced by relative state-estimation.

                            float cohesion_sum = 0;
                            float separation_sum = 0;
                            float total_sum = 0;

                            for (size_t i = 0; i < droneCount_; i++) // iterate over all quadcopters
                            {
                                if(i == droneNumber_) // skip for own quadcopter
                                    continue;

                                float dist = (potential_position - cyclic_positions_est_[i]).norm();

                                if(true) // separation and cohesion only calculated for quadcopters in neighbourhood
                                {
                                    cohesion_sum += pow(dist, 2);
                                    separation_sum += 1.0/pow(fmax(0.000001, dist - 2*drone_radius_ - extra_separation_distance_), 2);
                                }
                                ROS_INFO_ONCE("dr.%d (%2d/%2d/%2d|%2d) i=%d, dist=%f, cohesion_sum=%f, separation_sum=%f", droneNumber_, xi, yi, zi, ai, (int)i, dist, cohesion_sum, separation_sum);
                            }

                            float dist_beacon[N_BEACONS_MAX], dist_gt_beacon[N_BEACONS_MAX];
                            dist_beacon[0] = (potential_position - cyclic_positions_est_[droneNumber_]).norm();
                            // quickfix: cyclic_positions_est_[droneNumber_] contains the distance to beacon0

                            float target_sum;
                            target_sum = fabs(dist_beacon[current_target_]);

                            float height_diff = swarm_elevation_ - (elevation_filtered_[droneNumber_] + potential_movement[2]);
                            float height_sum = height_diff*height_diff;

                            float coehesion_term = spc_cohesion_weight_ * cohesion_sum / ((float)neighbourhood_cnt);
                            float separation_term = spc_separation_weight_ * separation_sum / ((float)neighbourhood_cnt);
                            float target_term = spc_target_weight_ * target_sum;
                            float calm_term = spc_calm_weight_ * potential_movement.norm();
                            float height_term = 0;

                            if(enable_swarm_ & SWARM_SPC_DISTANCES_ELEV)
                                height_term += spc_height_weight_ * height_sum;

                            if(neighbourhood_cnt != 0) // no neighbours means there was a division by 0
                                total_sum = coehesion_term + separation_term + target_term + calm_term + height_term;
                            else
                                total_sum = target_term + calm_term + height_term;

                            ROS_INFO_ONCE("dr.%d (%2d/%2d/%2d|%2d) coh=%7.1f sep=%7.1f tar=%7.1f calm=%7.1f total=%7.1f len=%f", droneNumber_, xi, yi, zi, ai, coehesion_term, separation_term, target_term, calm_term, total_sum, potential_movement.norm());
                            //ROS_INFO("dr.%d (%2d/%2d/%2d|%2d) tar=%f", droneNumber_, xi, yi, zi, ai, target_sum);

                            if(total_sum < best_sum)
                            {
                                best_sum = total_sum;

                                best_movement = potential_movement; // save movement vector

                                min_coehesion_term = coehesion_term;
                                min_separation_term = separation_term;
                                min_target_term = target_term;
                                min_calm_term = calm_term;
                                min_height_term = height_term;
                            }

                            if(xi == 0 && yi == 0 && zi == 0)
                            {
                                still_sum = total_sum;

                                still_coehesion_term = coehesion_term;
                                still_separation_term = separation_term;
                                still_target_term = target_term;
                                still_calm_term = calm_term;
                                still_height_term = height_term;
                            }

                        }
                    }
                }
            }

            ROS_INFO_ONCE("drone%d coh=%7.1f sep=%7.1f tar=%7.1f calm=%7.1f total=%7.1f", droneNumber_, min_coehesion_term, min_separation_term, min_target_term, min_calm_term, best_sum);

            if(inner_controller_ == 3) // velocity controller
            {
                set_point.pose.position.x = best_movement(0) * velocity_scaling_;
                set_point.pose.position.y = best_movement(1) * velocity_scaling_;
                set_point.pose.position.z = best_movement(2) * velocity_scaling_;
                ROS_INFO_ONCE("RelativeDistanceController %d exploitation (velocity) tsum=%f scal=%f", droneNumber_, best_sum, velocity_scaling_);
                set_point_marker[0] = odometry_.position[0] + best_movement(0) * velocity_scaling_;
                set_point_marker[1] = odometry_.position[1] + best_movement(1) * velocity_scaling_;
                set_point_marker[2] = odometry_.position[2] + best_movement(2) * velocity_scaling_;
                tempEnv << exploration_info << "," << best_movement(0) * velocity_scaling_ << "," << best_movement(1) * velocity_scaling_ << "," << best_movement(2) * velocity_scaling_ << "," << (float)best_sum << "," << history_cnt_ << ",";
            }
            else
                ROS_FATAL("inner_controller_ != 3, not velocity controller");
        }
        else
        {
            exploration_info = 0;
            set_point_marker[0] = 0;
            set_point_marker[1] = 0;
            set_point_marker[2] = -1000;
        }
    }

    if(enable_swarm_ != SWARM_DISABLED || target_swarm_.position_W[2] > 0.01) // do not enable drone until proper target point received
    {
        setpoint_pub_.publish(set_point);
        ROS_INFO_ONCE("RelativeDistanceController %d set_point x=%f y=%f z=%f", droneNumber_, set_point.pose.position.x, set_point.pose.position.y, set_point.pose.position.z);
    }

    if(dataStoring_active_) // save data for log files
    {
        tempDistance << "\n";
        listDistance_.push_back(tempDistance.str());
        tempEnv << "\n";
        listEnv_.push_back(tempEnv.str());
        tempState << "\n";
        listState_.push_back(tempState.str());
        tempCost << "\n";
        listCost_.push_back(tempCost.str());
        tempVectors << "\n";
        listVectors_.push_back(tempVectors.str());
    }

    if(!isSim_) return; // we are not in simulation, so we should not try to move gazebo markers

    // move gazebo markers
    geometry_msgs::Point pr2_position_red;
    geometry_msgs::Point pr2_position_blue;
    pr2_position_red.x = set_point_marker[0];
    pr2_position_red.y = set_point_marker[1];
    pr2_position_red.z = set_point_marker[2];
    if(enable_swarm_ == SWARM_DISABLED) // hovering at fixed position, use both markers
    {
        pr2_position_blue = pr2_position_red;
    }
    else if(exploration_info == 0) // exploitation phase, use blue marker
    {
        pr2_position_blue = pr2_position_red;
        pr2_position_red.z -= 1000;
    }
    else // exploration phase, use blue marker
    {
        pr2_position_blue = pr2_position_red;
        pr2_position_blue.z -= 1000;
    }

    geometry_msgs::Quaternion pr2_orientation;
    geometry_msgs::Pose pr2_pose;
    gazebo_msgs::ModelState pr2_modelstate;
    gazebo_msgs::SetModelState srv;
    pr2_orientation.x = 0.0;
    pr2_orientation.y = 0.0;
    pr2_orientation.z = 0.0;
    pr2_orientation.w = 1.0;
    pr2_pose.position = pr2_position_red;
    pr2_pose.orientation = pr2_orientation;
    pr2_modelstate.model_name = (std::string) "marker_red_crazyflie2_" + std::to_string(droneNumber_);
    pr2_modelstate.pose = pr2_pose;
    srv.request.model_state = pr2_modelstate;
    if(!gazebo_client_.call(srv))
        ROS_ERROR("Failed to move red marker! Error msg:%s",srv.response.status_message.c_str());
    pr2_pose.position = pr2_position_blue;
    pr2_modelstate.model_name = (std::string) "marker_blue_crazyflie2_" + std::to_string(droneNumber_);
    pr2_modelstate.pose = pr2_pose;
    srv.request.model_state = pr2_modelstate;
    if(!gazebo_client_.call(srv))
        ROS_ERROR("Failed to move blue marker! Error msg:%s",srv.response.status_message.c_str());
}

void RelativeDistanceController::DistancesCallback(const std_msgs::Float32MultiArray& distances_msg) {
    ROS_INFO_ONCE("DistancesCallback got first distances message.");

/*
    // publish distances message (old version, this contains all distances for every drone to every)
    for (size_t i = 0; i < droneCount_; i++)
    {
        for (size_t j = 0; j < droneCount_; j++)
        {
            distances_[i][j] = distances_msg.data[i*droneCount_ + j];
            distances_filtered_[i][j] = distances_filtered_[i][j]*(1.0-distance_iir_filter_) + distances_[i][j]*(distance_iir_filter_); // IIR lowpass filter for distance measurements
            ROS_INFO_ONCE("DistancesCallback drone#%d -> drone#%d: distance=%f.", (int)i, (int)j, distances_[i][j]);
        }
    }
*/
    // publish distances message (new version, this only contains the distance from the own drone to all others)
    for (size_t j = 0; j < droneCount_; j++)
    {
        distances_[droneNumber_][j] = distances_msg.data[j];
        if(noise_suppression_ == 1)
        {
            if(distances_[droneNumber_][j] < distance_max_filter_ && distances_[droneNumber_][j] > distance_min_filter_) // reject completely incorrect measurements
                distances_filtered_[droneNumber_][j] = distances_filtered_[droneNumber_][j]*(1.0-distance_iir_filter_) + distances_[droneNumber_][j]*(distance_iir_filter_); // IIR lowpass filter for distance measurements
        }
        else if(noise_suppression_ == 0)
            distances_filtered_[droneNumber_][j] = distances_[droneNumber_][j];

        ROS_INFO_ONCE("DistancesCallback drone#%d -> drone#%d: distance=%f filtered=%f.", (int)droneNumber_, (int)j, distances_[droneNumber_][j], distances_filtered_[droneNumber_][j]);

        if(enable_swarm_ & SWARM_USE_ML)
        {
            torch::Tensor tensor = torch::tensor({{(float)(odometry_.position[0] - odometry_last_distances_measurement_.position[0]),
                                                   (float)(odometry_.position[1] - odometry_last_distances_measurement_.position[1]),
                                                   (float)(odometry_.position[2] - odometry_last_distances_measurement_.position[2]),
                                                   distances_filtered_[droneNumber_][j]}}, {torch::kFloat32});
            auto ret = distances_model_[j].forward({tensor, distances_hx_[j]}).toTuple();
            distances_hx_[j] = ret->elements()[1];
            torch::Tensor outputs = ret->elements()[0].toTensor();
            //ROS_INFO("DistancesCallback drone#%d -> drone#%d output=%f.", (int)droneNumber_, (int)j, outputs[0].item<float>());
        }
    }
    odometry_last_distances_measurement_ = odometry_;
}

void RelativeDistanceController::TrilaterationCalculation(EigenOdometry* history0,
                                                          EigenOdometry* history1,
                                                          EigenOdometry* history2,
                                                          EigenOdometry* history3,
                                                          float* distances_history0,
                                                          float* distances_history1,
                                                          float* distances_history2,
                                                          float* distances_history3,
                                                          EigenOdometry* current,
                                                          Vector3f* positions_result,
                                                          float* quality_result
                            )
{
    int error_occured = 0;
    Vector3f movement_a, movement_b, movement_c, movement_v, movement_w, movement_u;
    movement_a[0] = history0->position[0] - history3->position[0];
    movement_a[1] = history0->position[1] - history3->position[1];
    movement_a[2] = history0->position[2] - history3->position[2];
    movement_b[0] = history1->position[0] - history3->position[0];
    movement_b[1] = history1->position[1] - history3->position[1];
    movement_b[2] = history1->position[2] - history3->position[2];
    movement_c[0] = history2->position[0] - history3->position[0];
    movement_c[1] = history2->position[1] - history3->position[1];
    movement_c[2] = history2->position[2] - history3->position[2];
    movement_v[0] = history1->position[0] - history2->position[0];
    movement_v[1] = history1->position[1] - history2->position[1];
    movement_v[2] = history1->position[2] - history2->position[2];
    movement_w[0] = history0->position[0] - history2->position[0];
    movement_w[1] = history0->position[1] - history2->position[1];
    movement_w[2] = history0->position[2] - history2->position[2];
    movement_u[0] = history0->position[0] - history1->position[0];
    movement_u[1] = history0->position[1] - history1->position[1];
    movement_u[2] = history0->position[2] - history1->position[2];

    /*
    ROS_INFO("(#%d) movement_a x=%f y=%f z=%f", droneNumber_, movement_a[0], movement_a[1], movement_a[2]);
    ROS_INFO("(#%d) movement_b x=%f y=%f z=%f", droneNumber_, movement_b[0], movement_b[1], movement_b[2]);
    ROS_INFO("(#%d) movement_c x=%f y=%f z=%f", droneNumber_, movement_c[0], movement_c[1], movement_c[2]);
    */

    Vector3f c_1, c_2, c_3, c_4;
    c_1[0] = 0; c_1[1] = 0; c_1[2] = 0;
    c_2[0] = movement_c.norm(); c_2[1] = 0; c_2[2] = 0;
    c_3[0] = (pow(movement_b.norm(),2) - pow(movement_v.norm(),2) + pow(c_2[0], 2)) / (2.0*c_2[0]);
    if(pow(movement_b.norm(),2) - pow(c_3[0], 2) < 0)
    {
        ROS_ERROR("(#%d) ERROR: pow(movement_b.norm(),2) - pow(c_3[0], 2) < 0", droneNumber_);
        error_occured = 1;
        c_3[1] = 0.01;
    }
    else
        c_3[1] = sqrt(pow(movement_b.norm(),2) - pow(c_3[0], 2));
    c_3[2] = 0;

    /*
    ROS_INFO("(#%d) c_1 x=%f y=%f z=%f", droneNumber_, c_1[0], c_1[1], c_1[2]);
    ROS_INFO("(#%d) c_2 x=%f y=%f z=%f", droneNumber_, c_2[0], c_2[1], c_2[2]);
    ROS_INFO("(#%d) c_3 x=%f y=%f z=%f", droneNumber_, c_3[0], c_3[1], c_3[2]);
    */

    // determine rotation

    // // first rotation: align point c_2 to movement_c
    Vector3f a1 = c_2;
    Vector3f b1 = movement_c;
    a1 = a1 / a1.norm(); // divide by norm to get unit vector
    b1 = b1 / b1.norm(); // divide by norm to get unit vector
    // get axis of rotation by cross-product of a1, b1
    Vector3f axis1 = a1.cross(b1);
    Matrix3f rotation1;
    if (axis1.norm() < 0.000001) // a1 is already aligned with b1, use identity matrix for first rotation
        rotation1 = IdentityMatrix();
    else
        rotation1 = RotationMatrixFromAxisAngle(axis1 / axis1.norm(), acos(a1.dot(b1) / (a1.norm() * b1.norm()))); // get angle of rotation by scalar-product of a1, b1
    /*
    ROS_INFO("rotation1 %s", MatrixToString(rotation1).c_str());
    Vector3f c_2_tmp = rotation1 * c_2;
    ROS_INFO("c_2_tmp %s", VectorToString(c_2_tmp).c_str());
    ROS_INFO("movement_c %s", VectorToString(movement_c).c_str());
    */

    Vector3f h0 = {0, 0, 1};
    // // helper rotation: rotation axis to Z-axis
    Vector3f aa1 = rotation1 * a1;
    Vector3f axis0 = aa1.cross(h0);
    Matrix3f rotation01 = RotationMatrixFromAxisAngle(axis0 / axis0.norm(), acos(aa1.dot(h0) / (aa1.norm() * h0.norm()))); // get angle of rotation by scalar-product of aa1, h0
    Vector3f a_tmp = rotation01 * rotation1 * c_3;
    a_tmp[2] = 0;

    // // helper rotation: rotation axis to Z-axis
    axis0 = b1.cross(h0);
    Matrix3f rotation02 = RotationMatrixFromAxisAngle(axis0 / axis0.norm(), acos(b1.dot(h0) / (b1.norm() * h0.norm()))); // get angle of rotation by scalar-product of b1, h0
    Vector3f b_tmp = rotation02 * movement_b;
    b_tmp[2] = 0;

    // // second rotation: align point a2 to b2
    Vector3f a2 = c_3;
    Vector3f b2 = movement_b;
    a2 = a2 / a2.norm(); // divide by norm to get unit vector
    b2 = b2 / b2.norm(); // divide by norm to get unit vector
    // get axis as vector b1
    Vector3f axis2 = b1 / b1.norm();
    // get angle of rotation by scalar-product of a_tmp, b_tmp
    float angle2 = acos(a_tmp.dot(b_tmp) / (a_tmp.norm() * b_tmp.norm()));
    Matrix3f rotation2p = RotationMatrixFromAxisAngle(axis2, angle2);
    Matrix3f rotation2m = RotationMatrixFromAxisAngle(axis2, -angle2);

    Matrix3f rotation2;
    // determine in which direction to rotate
    float diff_p = (rotation2p * rotation1 * c_3 - movement_b).norm();
    float diff_m = (rotation2m * rotation1 * c_3 - movement_b).norm();
    if(diff_m < diff_p)
        rotation2 = rotation2m;
    else
        rotation2 = rotation2p;
    Matrix3f rotation = rotation2 * rotation1;

    Vector3f c_2_tmp = rotation * c_2;
    if((c_2_tmp - movement_c).norm() > 0.01)
    {
        ROS_ERROR("(#%d) ERROR: c_2_tmp - movement_c alignment error", droneNumber_);
        error_occured = 1;
    }

    Vector3f c_3_tmp = rotation * c_3;
    if((c_3_tmp - movement_b).norm() > 0.01)
    {
        ROS_ERROR("(#%d) ERROR: c_3_tmp - movement_b alignment error", droneNumber_);
        error_occured = 1;
    }

    /*
    Vector3f c_2_tmp = rotation * c_2;
    ROS_INFO("c_2_tmp %s", VectorToString(c_2_tmp).c_str());
    ROS_INFO("movement_c %s", VectorToString(movement_c).c_str());

    Vector3f c_3_tmp = rotation * c_3;
    ROS_INFO("c_3_tmp %s", VectorToString(c_3_tmp).c_str());
    ROS_INFO("movement_b %s", VectorToString(movement_b).c_str());
    */

    // check with c_4 (coresponds to history0_)
    c_4[0] = (pow(movement_a.norm(), 2) - pow(movement_w.norm(), 2) + pow(c_2[0], 2)) / (2.0*c_2[0]);
    c_4[1] = (pow(movement_a.norm(), 2) - pow(movement_u.norm(), 2) + pow(c_3[0], 2) + pow(c_3[1], 2) - 2*c_3[0]*c_4[0]) / (2.0*c_3[1]);
    c_4[2] = sqrt(pow(movement_a.norm(), 2) - pow(c_4[0], 2) - pow(c_4[1], 2));
    if(pow(movement_a.norm(), 2) - pow(c_4[0], 2) - pow(c_4[1], 2) < 0)
    {
        ROS_ERROR("(#%d) ERROR: pow(movement_a.norm(), 2) - pow(c_4[0], 2) - pow(c_4[1], 2) < 0", droneNumber_);
        error_occured = 1;
    }

    // check if it should be mirrored
    Vector3f c_4neg = c_4;
    c_4neg[2] = 0 - c_4neg[2];
    if((rotation * c_4neg - movement_a).norm() < (rotation * c_4 - movement_a).norm())
        c_4 = c_4neg;

    /*
    Vector3f c_4_tmp = rotation * c_4;
    ROS_INFO("c_4_tmp %s", VectorToString(c_4_tmp).c_str());
    ROS_INFO("movement_a %s", VectorToString(movement_a).c_str());
    ROS_INFO("------------------------------");
    */

    Vector3f c_4_tmp = rotation * c_4;
    if((c_4_tmp - movement_a).norm() > 0.01)
    {
        ROS_ERROR("(#%d) ERROR: c_4_tmp - movement_a alignment error", droneNumber_);
        error_occured = 1;
    }

    for (size_t i = 0; i < droneCount_; i++)
    {
        quality_result[i] = 0;
        if(error_occured)
            quality_result[i] += 5000;

        //if(i == droneNumber_) // skip for own quadcopter
        //    continue;
        // quickfix: do not skip, as this contains the distance to beacon0

        // calculate relative position of drone i w.r.t. to own drone
        float r_1 = distances_history3[i], r_2 = distances_history2[i], r_3 = distances_history1[i], r_4 = distances_history0[i];
        Vector3f other_drone_tmp;
        Vector3f other_drone_neg;
        Vector3f current_v;
        current_v[0] = current->position[0];
        current_v[1] = current->position[1];
        current_v[2] = current->position[2];

        other_drone_tmp[0] = (pow(r_1, 2) - pow(r_2, 2) + pow(c_2[0], 2)) / (2.0*c_2[0]);
        other_drone_tmp[1] = (pow(r_1, 2) - pow(r_3, 2) + pow(c_3[0], 2) + pow(c_3[1], 2) - 2*c_3[0]*other_drone_tmp[0]) / (2.0*c_3[1]);
        other_drone_tmp[2] = sqrt(pow(r_1, 2) - pow(other_drone_tmp[0], 2) - pow(other_drone_tmp[1], 2));
        if(pow(r_1, 2) - pow(other_drone_tmp[0], 2) - pow(other_drone_tmp[1], 2) < 0)
        {
            ROS_ERROR("(#%d) ERROR: pow(r_1, 2) - pow(other_drone_tmp[0], 2) - pow(other_drone_tmp[1], 2) < 0", droneNumber_);
            quality_result[i] += 1000;
        }

        other_drone_neg = other_drone_tmp; other_drone_neg[2] = 0 - other_drone_tmp[2];
        if(fabs((other_drone_neg - c_4).norm() - r_4) < fabs((other_drone_tmp - c_4).norm() - r_4)) // check if it should be mirrored
            other_drone_tmp = other_drone_neg;

        if((other_drone_tmp - c_1).norm() - r_1 > 0.01)
        {
            ROS_ERROR("(#%d;%d) ERROR: (other_drone_tmp - c_1).norm() - r_1 error", droneNumber_, (int)i);
            quality_result[i] += 1000;
        }
        if((other_drone_tmp - c_2).norm() - r_2 > 0.01)
        {
            ROS_ERROR("(#%d;%d) ERROR: (other_drone_tmp - c_2).norm() - r_2 error", droneNumber_, (int)i);
            quality_result[i] += 1000;
        }
        if((other_drone_tmp - c_3).norm() - r_3 > 0.01)
        {
            ROS_ERROR("(#%d;%d) ERROR: (other_drone_tmp - c_3).norm() - r_3 error", droneNumber_, (int)i);
            quality_result[i] += 1000;
        }

        Vector3f other_drone_rotated = rotation * other_drone_tmp;
        Vector3f other_drone_position = other_drone_rotated + current_v;

        /*
        ROS_INFO("distances: r_1 %f r_2 %f r_3 %f r_4 %f", r_1, r_2, r_3, r_4);
        ROS_INFO("(other_drone_tmp - c_1).norm %f c_2 %f c_3 %f c_4 %f", (other_drone_tmp - c_1).norm(), (other_drone_tmp - c_2).norm(), (other_drone_tmp - c_3).norm(), (other_drone_tmp - c_4).norm());
        ROS_INFO("other_drone_tmp %s", VectorToString(other_drone_tmp).c_str());
        ROS_INFO("other_drone_rotated %s", VectorToString(other_drone_rotated).c_str());
        */
        //ROS_INFO("other_drone_position %s", VectorToString(other_drone_position).c_str());
        //ROS_INFO("(other_drone_tmp - c_4).norm() - r_4 = %f", (other_drone_tmp - c_4).norm() - r_4);
        //ROS_INFO("positions_gt_[%d] %s", (int)i, VectorToString(positions_gt_[i]).c_str()); // ground-truth position of drones, only to be used for verification of estimation
        //ROS_INFO("error %s (norm: %f)", VectorToString(other_drone_position - positions_gt_[i]).c_str(), (other_drone_position - positions_gt_[i]).norm());
        /*
        other_drone_neg[2] = 0 - other_drone_tmp[2];
        Vector3f other_drone_rotated_neg = rotation * other_drone_neg;
        Vector3f other_drone_position_neg = other_drone_rotated_neg + current_v;

        ROS_INFO("other_drone_position_neg %s", VectorToString(other_drone_position_neg).c_str());
        ROS_INFO("(other_drone_neg - c_4).norm() - r_4 = %f", (other_drone_neg - c_4).norm() - r_4);
        ROS_INFO("positions_gt_[%d] %s", (int)i, VectorToString(positions_gt_[i]).c_str()); // ground-truth position of drones, only to be used for verification of estimation
        ROS_INFO("error %s (norm: %f)", VectorToString(other_drone_position_neg - positions_gt_[i]).c_str(), (other_drone_position_neg - positions_gt_[i]).norm());

        Vector3f r_1_vec = other_drone_tmp - c_1;
        Vector3f r_2_vec = other_drone_tmp - c_2;
        Vector3f r_3_vec = other_drone_tmp - c_3;
        float angle1_2 = acos(r_1_vec.dot(r_2_vec) / (r_1_vec.norm() * r_2_vec.norm()));
        float angle1_3 = acos(r_1_vec.dot(r_3_vec) / (r_1_vec.norm() * r_3_vec.norm()));
        float angle2_3 = acos(r_2_vec.dot(r_3_vec) / (r_2_vec.norm() * r_3_vec.norm()));

        ROS_INFO("angle: 1_2 %f 1_3 %f 2_3 %f (product: %f)", angle1_2, angle1_3, angle2_3, angle1_2*angle1_3*angle2_3);
        */

        positions_result[i] = other_drone_position;
        quality_result[i] += fabs((other_drone_tmp - c_4).norm() - r_4);

        if(isnan(quality_result[i]))
        {
            positions_result[i][0] = 0;
            positions_result[i][1] = 0;
            positions_result[i][2] = 1;
            quality_result[i] = 100000;
        }

        // ROS_INFO("quality_result[i] %f", quality_result[i]);
        // ROS_INFO("------------------------------");
    }
}



void RelativeDistanceController::ElevationCallback(const std_msgs::Float32MultiArray& elevation_msg) {
    ROS_INFO_ONCE("DistancesCallback got first elevation message.");
    // do not use this callback, as we get elevation in z coordinate of odometry anyways

    /*
    for (size_t i = 0; i < droneCount_; i++)
    {
        elevation_[i] = elevation_msg.data[i];
        elevation_filtered_[i] = elevation_filtered_[i]*(1.0-elevation_iir_filter_) + elevation_[i]*(elevation_iir_filter_); // IIR lowpass filter for distance measurements
        ROS_INFO_ONCE("DistancesCallback drone#%d: elevation=%f.", (int)i, elevation_[i]);
    }
    */
}

void RelativeDistanceController::BeaconsCallback(const std_msgs::Float32MultiArray& distances_msg) {
    ROS_INFO_ONCE("BeaconsCallback got first beacons message.");

/*
    // publish distances to beacons message (old version, this contains all distances for every drone to every beacon)
    for (size_t i = 0; i < droneCount_; i++)
    {
        for (size_t j = 0; j < beaconCount_; j++)
        {
            beacons_[i][j] = distances_msg.data[i*beaconCount_ + j];
            beacons_filtered_[i][j] = beacons_filtered_[i][j]*(1.0-distance_iir_filter_) + beacons_[i][j]*(distance_iir_filter_); // IIR lowpass filter for distance measurements
            ROS_INFO_ONCE("BeaconsCallback drone#%d -> beacon#%d: distance=%f.", (int)i, (int)j, beacons_[i][j]);
        }
    }
*/
    // publish distances to beacons message (new version, this only contains the distance from the own drone to all beacons)
    for (size_t j = 0; j < beaconCount_; j++)
    {
        beacons_[droneNumber_][j] = distances_msg.data[j];
        if(noise_suppression_ == 1)
        {
            if(beacons_[droneNumber_][j] < distance_max_filter_ && beacons_[droneNumber_][j] > distance_min_filter_) // reject completely incorrect measurements
                beacons_filtered_[droneNumber_][j] = beacons_filtered_[droneNumber_][j]*(1.0-distance_iir_filter_) + beacons_[droneNumber_][j]*(distance_iir_filter_); // IIR lowpass filter for distance measurements
        }
        else if(noise_suppression_ == 0)
            beacons_filtered_[droneNumber_][j] = beacons_[droneNumber_][j];
        ROS_INFO_ONCE("BeaconsCallback drone#%d -> beacon#%d: distance=%f filtered=%f.", (int)droneNumber_, (int)j, beacons_[droneNumber_][j], beacons_filtered_[droneNumber_][j]);


        if(enable_swarm_ & SWARM_USE_ML)
        {
            torch::Tensor tensor = torch::tensor({{(float)(odometry_.position[0] - odometry_last_beacons_measurement_.position[0]),
                                                   (float)(odometry_.position[1] - odometry_last_beacons_measurement_.position[1]),
                                                   (float)(odometry_.position[2] - odometry_last_beacons_measurement_.position[2]),
                                                   beacons_filtered_[droneNumber_][j]}}, {torch::kFloat32});
            auto ret = beacons_model_[j].forward({tensor, beacons_hx_[j]}).toTuple();
            beacons_hx_[j] = ret->elements()[1];
            torch::Tensor outputs = ret->elements()[0].toTensor();
        }
    }
    odometry_last_beacons_measurement_ = odometry_;
}

void RelativeDistanceController::PositionsCallback(const std_msgs::Float32MultiArray& positions_msg) {
    ROS_INFO_ONCE("PositionsCallback got positions message.");

    for (size_t i = 0; i < droneCount_; i++)
    {
        // ground-truth positions of all drones, only to be used for verification of estimation
        positions_gt_[i][0]= positions_msg.data[i*3 + 0];
        positions_gt_[i][1]= positions_msg.data[i*3 + 1];
        positions_gt_[i][2]= positions_msg.data[i*3 + 2];

        ROS_INFO_ONCE("PositionsCallback (%d) drone#%d @ %s.", droneNumber_, (int)i, VectorToString(positions_gt_[i]).c_str());
    }
}

// This is only used for debugging. Absolute position of beacons are not available for controller!
void RelativeDistanceController::ModelstateCallback(const gazebo_msgs::ModelStatesConstPtr& modelstates_msg)
{
    ROS_INFO_ONCE("RelativeDistanceController got ModelstateCallback.");

    int b = 0;
    for (size_t i = 0; i < modelstates_msg->name.size(); i++) // iterate over models
    {
        for (size_t j = 0; j < beaconCount_; j++) // iterate over all beacons
        {
            if(modelstates_msg->name[i] == std::string("marker_green_beacon_").append(std::to_string(j)))
            {
                // save beacon positions
                beacon_gt_[b][0] = modelstates_msg->pose[i].position.x;
                beacon_gt_[b][1] = modelstates_msg->pose[i].position.y;
                beacon_gt_[b][2] = modelstates_msg->pose[i].position.z;
                ROS_INFO_ONCE("RelativeDistanceController model %d: %s (%d) at location %s", (int)i, modelstates_msg->name[i].c_str(), b, VectorToString(beacon_gt_[b]).c_str());
                b ++;
            }
        }
        if(modelstates_msg->name[i] == "jackal")
        {
            // save jackal position as beacon 1
            beacon_gt_[1][0] = modelstates_msg->pose[i].position.x;
            beacon_gt_[1][1] = modelstates_msg->pose[i].position.y;
            beacon_gt_[1][2] = modelstates_msg->pose[i].position.z;
            beacon_gt_[1][2] = 3.5; // TODO: this is just for testing!
            ROS_INFO_ONCE("RelativeDistanceController model %d: %s (%d) at location %s", (int)i, modelstates_msg->name[i].c_str(), 1, VectorToString(beacon_gt_[b]).c_str());
        }
    }
}

// This is only used for debugging. Absolute position of drones are not available for controller!
void DroneStateWithTime::PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
    ROS_INFO_ONCE("DroneStateWithTime got first pose message.");

    // received message containing other drone position information
    // odometry_ = EigenOdometry();
    odometry_.timeStampSec = pose_msg->header.stamp.sec;
    odometry_.timeStampNsec = pose_msg->header.stamp.nsec;
    odometry_.position[0] = pose_msg->pose.position.x;
    odometry_.position[1] = pose_msg->pose.position.y;
    odometry_.position[2] = pose_msg->pose.position.z;

    ROS_INFO_ONCE("DroneStateWithTime got pose message: x=%f y=%f z=%f (self:%d, other:%d)", odometry_.position[0], odometry_.position[1], odometry_.position[2], self_, other_);
}

float DroneStateWithTime::GetDistance_gt(EigenOdometry* odometry_gt) {
    if(self_ == other_)
        return 0;

    float distance_x = fabs(odometry_.position[0] - odometry_gt->position[0]);
    float distance_y = fabs(odometry_.position[1] - odometry_gt->position[1]);
    float distance_z = fabs(odometry_.position[2] - odometry_gt->position[2]);
    return sqrt(distance_x*distance_x + distance_y*distance_y + distance_z*distance_z);
}

float DroneStateWithTime::GetDistance_sim_gt(DroneStateWithTime* own_state, Vector3f potential_movement)
{
    float distance_x = fabs(odometry_.position[0] - (own_state->odometry_.position[0] + potential_movement[0]));
    float distance_y = fabs(odometry_.position[1] - (own_state->odometry_.position[1] + potential_movement[1]));
    float distance_z = fabs(odometry_.position[2] - (own_state->odometry_.position[2] + potential_movement[2]));
    return sqrt(distance_x*distance_x + distance_y*distance_y + distance_z*distance_z);
}


void DroneStateWithTime::SetId(int self, int other)
{
    self_ = self;
    other_ = other;
}


}

void rotors_control::linearLeastSquaresApproximation(float* dataset, size_t n_points, float* alpha0, float* alpha1)
{
    // ROS_INFO("linearLeastSquaresApproximation n_points=%d", (int)n_points);

    float x_mean = ((float)n_points+1)/2.0;

    float y_mean = 0;
    for (size_t i = 0; i < n_points; i++)
        y_mean += dataset[i];
    y_mean /= (float)n_points;

    float dividend = 0.0, divisor = 0.0;
    for (size_t i = 0; i < n_points; i++)
        dividend += ((float)i+1.0) * dataset[i];
    dividend -= ((int)n_points)*x_mean*y_mean;
    for (size_t i = 0; i < n_points; i++)
        divisor += ((float)i+1.0) * ((float)i+1.0);
    divisor -= ((float)n_points)*(x_mean*x_mean);

    float quotient = dividend / divisor;
    *alpha1 = quotient;
    *alpha0 = y_mean - quotient*x_mean;

    // ROS_INFO("linearLeastSquaresApproximation alpha0=%f alpha1=%f", *alpha0, *alpha1);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "relative_distance_controller_node");

    ros::NodeHandle nh2;

    rotors_control::RelativeDistanceController relative_distance_controller_node;

    ros::spin();

    return 0;
}
