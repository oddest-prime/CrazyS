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

#include "swarm_state_estimator.h"

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
// #include "rotors_control/crazyflie_complementary_filter.h"

namespace rotors_control {

SwarmStateEstimator::SwarmStateEstimator() {

    ROS_INFO_ONCE("Started SwarmStateEstimator");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

    // Topics subscribe
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &SwarmStateEstimator::OdometryCallback, this);
    distances_sub_ = nh.subscribe("/drone_distances", 1, &SwarmStateEstimator::DistancesCallback, this);
    elevation_sub_ = nh.subscribe("/drone_elevation", 1, &SwarmStateEstimator::ElevationCallback, this);

    // To publish the current pose for gazebo visual
    visual_cnt_ = 0;
    gazebo_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

SwarmStateEstimator::~SwarmStateEstimator(){}

void SwarmStateEstimator::InitializeParams() {
    ros::NodeHandle pnh("~");

    ROS_INFO_ONCE("[SwarmStateEstimator] InitializeParams");

    //Reading the parameters come from the launch file
    std::string dataStoringActive;
    int droneCount;
    int droneNumber;
    double dataStoringTime;

    if (pnh.getParam("droneCount", droneCount)){
       ROS_INFO("Got param 'droneCount': %d", droneCount);
       droneCount_ = droneCount;
    }
    else
       ROS_ERROR("Failed to get param 'droneCount'");

    if (pnh.getParam("droneNumber", droneNumber)){
        ROS_INFO("Got param 'droneNumber': %d", droneNumber);
        droneNumber_ = droneNumber;
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
    //timer_saveData = nh.createTimer(ros::Duration(dataStoringTime), &SwarmStateEstimator::CallbackSaveData, this, false, true);
}

void SwarmStateEstimator::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("SwarmStateEstimator got first odometry message.");

    // received message drone position information, store ground-truth pose
    odometry_gt_.timeStampSec = odometry_msg->header.stamp.sec;
    odometry_gt_.timeStampNsec = odometry_msg->header.stamp.nsec;
    odometry_gt_.position[0] = odometry_msg->pose.pose.position.x;
    odometry_gt_.position[1] = odometry_msg->pose.pose.position.y;
    odometry_gt_.position[2] = odometry_msg->pose.pose.position.z;
    ROS_INFO_ONCE("SwarmStateEstimator got odometry message: x=%f y=%f z=%f (%d)", odometry_gt_.position[0], odometry_gt_.position[1], odometry_gt_.position[2], droneNumber_);


/*  if(droneNumber_ == 0) // move red indicator sphere for testing purposes
    {
      geometry_msgs::Point pr2_position;
      pr2_position.x = odometry_msg->pose.pose.position.x + sin(visual_cnt_)* 0.1;
      pr2_position.y = odometry_msg->pose.pose.position.y + cos(visual_cnt_)* 0.1;
      pr2_position.z = odometry_msg->pose.pose.position.z + cos(visual_cnt_)* 0.1;
      geometry_msgs::Quaternion pr2_orientation;
      pr2_orientation.x = 0.0;
      pr2_orientation.y = 0.0;
      pr2_orientation.z = 0.0;
      pr2_orientation.w = 1.0;
      geometry_msgs::Pose pr2_pose;
      pr2_pose.position = pr2_position;
      pr2_pose.orientation = pr2_orientation;
      gazebo_msgs::ModelState pr2_modelstate;
      pr2_modelstate.model_name = (std::string) "marker_crazyflie2_" + std::to_string(droneNumber_);
      pr2_modelstate.pose = pr2_pose;
      gazebo_msgs::SetModelState srv;
      srv.request.model_state = pr2_modelstate;
      if(!gazebo_client_.call(srv))
        ROS_ERROR("Failed to move marker! Error msg:%s",srv.response.status_message.c_str());
      visual_cnt_ += 2*3.14159265358979323846 / 200;
    } */
}

void SwarmStateEstimator::ElevationCallback(const std_msgs::Float32MultiArray& elevation_msg) {
    ROS_INFO_ONCE("ElevationCallback got elevation message.");
    for (size_t i = 0; i < droneCount_; i++)
    {
        elevation_[i] = elevation_msg.data[i];
        ROS_INFO_ONCE("ElevationCallback drone#%d elevation=%f.", i, elevation_[i]);
    }
}

void SwarmStateEstimator::DistancesCallback(const std_msgs::Float32MultiArray& distances_msg) {
    ROS_INFO_ONCE("DistancesCallback got distances message.");

    for (size_t i = 0; i < droneCount_; i++)
    {
        odometry_estimate_[i].position[0] = 0;
        odometry_estimate_[i].position[1] = 0;
        odometry_estimate_[i].position[2] = 0;
    }

    for (size_t i = 0; i < droneCount_; i++)
    {
        for (size_t j = 0; j < droneCount_; j++)
        {
            distances_[i][j] = distances_msg.data[i*droneCount_ + j];
            ROS_INFO_ONCE("DistancesCallback drone#%d -> drone#%d: distance=%f.", i, j, distances_[i][j]);
        }
    }

    if(droneNumber_ == 2)
    {
        FindBestTriangle(distances_, best_triangle_);
        ROS_INFO_ONCE("DistancesCallback (%d) best_triangle_ = %d / %d / %d / %d", droneNumber_, best_triangle_[0], best_triangle_[1], best_triangle_[2], best_triangle_[3]);
        InferPositions(distances_, best_triangle_, odometry_estimate_);
        FindBestZset(distances_, odometry_estimate_, best_zset_);
        ROS_INFO("DistancesCallback (%d) best_zset_ = %d / %d / %d", droneNumber_, best_zset_[0], best_zset_[1], best_zset_[2]);
        CheckDistances(distances_, odometry_estimate_);

        EigenOdometry axis0;
        axis0.position[0] = 1;
        axis0.position[1] = 2;
        axis0.position[2] = 3;
        axis0 = axis0 / EuclideanNorm(&axis0);
        float angle0 = 0.3;
        auto rotation0 = RotationMatrixFromAxisAngle(&axis0, &angle0);
        ROS_INFO("RotationMatrixFromAxisAngle %s", toString(rotation0).c_str());

        EigenOdometry tmp0;
        axis0.position[0] = 7;
        axis0.position[1] = 8;
        axis0.position[2] = 9;
        auto tmp1 = rotation0 * tmp0;
        ROS_INFO("tmp0 %s", toString(tmp0).c_str());
        ROS_INFO("tmp1 %s", toString(tmp1).c_str());
    }

    if(droneNumber_ == 0) // move red indicator spheres to indicate determined positions
    {
        for (size_t i = 0; i < droneCount_; i++)
        {
            geometry_msgs::Point pr2_position;
            // pr2_position.x = odometry_gt_.position[0] + sin(visual_cnt_)* 0.1;
            // pr2_position.y = odometry_gt_.position[1] + cos(visual_cnt_)* 0.1;
            // pr2_position.z = odometry_gt_.position[2];
            pr2_position.x = odometry_gt_.position[0] + odometry_estimate_[i].position[0];
            pr2_position.y = odometry_gt_.position[1] + odometry_estimate_[i].position[1];
            pr2_position.z = odometry_gt_.position[2] + odometry_estimate_[i].position[2];
            geometry_msgs::Quaternion pr2_orientation;
            pr2_orientation.x = 0.0;
            pr2_orientation.y = 0.0;
            pr2_orientation.z = 0.0;
            pr2_orientation.w = 1.0;
            geometry_msgs::Pose pr2_pose;
            pr2_pose.position = pr2_position;
            pr2_pose.orientation = pr2_orientation;
            gazebo_msgs::ModelState pr2_modelstate;
            pr2_modelstate.model_name = (std::string) "marker_crazyflie2_" + std::to_string(i);
            pr2_modelstate.pose = pr2_pose;
            gazebo_msgs::SetModelState srv;
            srv.request.model_state = pr2_modelstate;
            if(!gazebo_client_.call(srv))
                ROS_ERROR("Failed to move marker! Error msg:%s",srv.response.status_message.c_str());
            visual_cnt_ += 2*3.14159265358979323846 / 200;
        }
    }
}

void SwarmStateEstimator::FindBestTriangle(float (*distances)[N_DRONES_MAX], int* triangle) {
    float diff_min = FLT_MAX;
    int diff_min_ind_i = droneNumber_;
    int diff_min_ind_a = -1;
    int diff_min_ind_b = -1;
    int z_max_ind = -1;

    for (size_t a = 0; a < droneCount_; a++)
    {
        if(a == droneNumber_)
            continue;
        for (size_t b = 0; b < a; b++)
        {
            if(b == droneNumber_)
                continue;
            float diff = (
                            pow(distances[droneNumber_][a] - distances[droneNumber_][b], 2) +
                            pow(distances[droneNumber_][a] - distances[a][b], 2) +
                            pow(distances[droneNumber_][b] - distances[a][b], 2)
                          ) / pow(distances[droneNumber_][a] + distances[droneNumber_][b] + distances[a][b], 2);
            if (diff < diff_min)
            {
                diff_min = diff;
                diff_min_ind_i = droneNumber_;
                diff_min_ind_a = a;
                diff_min_ind_b = b;
            }
            //_, _, z_max_ind = infer_positions_helper(nquads, distances, [i, a, b])
        }
    }
    triangle[0] = diff_min_ind_i;
    triangle[1] = diff_min_ind_a;
    triangle[2] = diff_min_ind_b;
    triangle[3] = z_max_ind; // TODO: determine actual best z-index drone
}

void SwarmStateEstimator::FindBestZset(float (*distances)[N_DRONES_MAX], EigenOdometry* positions, int* zset) {
    //find drone with the largest distance to own drone
    float maxdist = 0;
    int maxdist_ind = -1;
    for (size_t i = 0; i < droneCount_; i++)
    {
        if(distances[droneNumber_][i] > maxdist)
        {
            maxdist = distances[droneNumber_][i];
            maxdist_ind = i;
        }
    }

    // find drone with the largest projected distance to the vector (own drone, maxdist_ind)
    EigenOdometry md = UnitVector(&positions[maxdist_ind]);
    float maxproj = 0;
    int maxproj_ind = -1;
    for (size_t i = 0; i < droneCount_; i++)
    {
        if(i == maxdist_ind)
            continue;
        EigenOdometry dist_vect = positions[i] - md * DotProduct(&positions[i], &md);
        float dist = EuclideanNorm(&dist_vect);
        if(dist > maxproj)
        {
            maxproj = dist;
            maxproj_ind = i;
        }
    }

    // find third drone as tie-breaker (use drone with the largest projected distance to the vector (own drone, maxproj_ind))
    EigenOdometry mdd = UnitVector(&positions[maxproj_ind]);
    float tie_breaker = 0;
    int tie_breaker_ind = -1;
    for (size_t i = 0; i < droneCount_; i++)
    {
        if(i == maxdist_ind)
            continue;
        if(i == maxproj_ind)
            continue;
        EigenOdometry dist_vect = positions[i] - mdd * DotProduct(&positions[i], &mdd);
        float dist = EuclideanNorm(&dist_vect);
        if(dist > tie_breaker)
        {
            tie_breaker = dist;
            tie_breaker_ind = i;
        }
    }
    zset[0] = maxdist_ind;
    zset[1] = maxproj_ind;
    zset[2] = tie_breaker_ind;
}

void SwarmStateEstimator::InferPositions(float (*distances)[N_DRONES_MAX], int* triangle, EigenOdometry* positions) {
    int a = triangle[1];
    int b = triangle[2];
    int c = triangle[3];
    // int c = -1 # this saves the index of the fourth quadcopter to tie-break the roots (it is optional, if not given, the first quadcopter not in the triangle is used)

    float positions_quality[N_DRONES_MAX];

    positions[triangle[0]].position[0] = 0; // relative x-coordinate
    positions[triangle[0]].position[1] = 0; // relative y-coordinate
    positions[triangle[0]].position[2] = 0; // relative z-coordinate
    positions_quality[triangle[0]] = 3; // high quality of point (part of triangle)

    ROS_INFO_ONCE("InferPositions (%d) tr0_:%d at x:%f y:%f z:%f", droneNumber_, triangle[0], positions[triangle[0]].position[0], positions[triangle[0]].position[1], positions[triangle[0]].position[2]);

    positions[triangle[1]].position[0] = distances[triangle[0]][triangle[1]]; // relative x-coordinate
    positions[triangle[1]].position[1] = 0; // relative y-coordinate
    positions[triangle[1]].position[2] = 0; // relative z-coordinate
    positions_quality[triangle[1]] = 3; // high quality of point (part of triangle)

    ROS_INFO_ONCE("InferPositions (%d) tr1_:%d at x:%f y:%f z:%f", droneNumber_, triangle[1], positions[triangle[1]].position[0], positions[triangle[1]].position[1], positions[triangle[1]].position[2]);

    // https://en.wikipedia.org/wiki/True-range_multilateration#Two_Cartesian_dimensions,_two_measured_slant_ranges_(Trilateration)
    positions[triangle[2]].position[0] = ( // relative x-coordinate
                                            pow(distances[triangle[0]][triangle[2]], 2) -
                                            pow(distances[triangle[1]][triangle[2]], 2) +
                                            pow(distances[triangle[0]][triangle[1]], 2)
                                          ) / (2*distances[triangle[0]][triangle[1]]);
    float t1 = (pow(distances[triangle[0]][triangle[2]], 2) - pow(positions[triangle[2]].position[0], 2));
    if (t1 < 0) // should not be the case, but it can happen because noisy measurements: sum of two edges of the triangle is shorter than the third edge
        positions[triangle[2]].position[1] = 0.001; // relative y-coordinate
    else
        positions[triangle[2]].position[1] = sqrt(pow(distances[triangle[0]][triangle[2]], 2) - pow(positions[triangle[2]].position[0], 2)); // relative y-coordinate
    positions[triangle[2]].position[2] = 0; // relative z-coordinate
    positions_quality[triangle[2]] = 3; // high quality of point (part of triangle)
    ROS_INFO_ONCE("InferPositions (%d) tr2_:%d at x:%f y:%f z:%f", droneNumber_, triangle[2], positions[triangle[2]].position[0], positions[triangle[2]].position[1], positions[triangle[2]].position[2]);

    float z_max = 0;
    int z_max_ind = -1;

    std::list<int> quadrange;
    if(triangle[3] > 0)
        quadrange.push_back(triangle[3]);
    for (size_t i = 0; i < droneCount_; i++)
        if(i != triangle[0] && i != triangle[1] && i != triangle[2])
            quadrange.push_back(i);

    // https://en.wikipedia.org/wiki/True-range_multilateration#Three_Cartesian_dimensions,_three_measured_slant_ranges
    for(auto num = quadrange.begin(); num != quadrange.end(); num++)
    {
        int i = *num;

        positions[i].position[0] = ( // relative x-coordinate
                                      pow(distances[triangle[0]][i], 2) -
                                      pow(distances[triangle[1]][i], 2) +
                                      pow(distances[triangle[0]][triangle[1]], 2)
                                    ) / (2 * distances[triangle[0]][triangle[1]]);
        positions[i].position[1] = (
                                      pow(distances[triangle[0]][i], 2) -
                                      pow(distances[triangle[2]][i], 2) +
                                      pow(distances[triangle[0]][triangle[2]], 2) -
                                      2*positions[triangle[2]].position[0]*positions[i].position[0]
                                    ) / (2*positions[triangle[2]].position[1]);
        float t1 = (pow(distances[triangle[0]][i], 2) - pow(positions[i].position[0], 2) - pow(positions[i].position[1], 2));
        if(t1 > 0)
            positions[i].position[2] = sqrt(t1);
        else
            positions[i].position[2] = 0;

        if(abs(positions[i].position[2]) > z_max)
        {
            z_max = abs(positions[i].position[2]);
            z_max_ind = i;
        }
        positions_quality[i] = positions[i].position[2]; // quality of point proportional to z elevation
        ROS_INFO_ONCE("InferPositions (%d) loop:%d at x:%f y:%f z:%f", droneNumber_, i, positions[i].position[0], positions[i].position[1], positions[i].position[2]);

        // use fourth drone (tie-breaker) to check if relative z-coordinate may be flipped
        if(c >= 0)
        {
            EigenOdometry pos_neg = positions[i];
            pos_neg.position[2] = 0 - positions[i].position[2];
            float dist_pos = Distance(&positions[c], &positions[i]);
            float dist_neg = Distance(&positions[c], &pos_neg);
            if(abs(dist_neg - distances[c][i]) < abs(dist_pos - distances[c][i]))
                positions[i].position[2] = pos_neg.position[2];
        }
        if(c < 0)
            c = i;
    }

    for (size_t i = 0; i < droneCount_; i++)
        ROS_INFO_ONCE("InferPositions (%d) drone:%d at x:%f y:%f z:%f", droneNumber_, i, positions[i].position[0], positions[i].position[1], positions[i].position[2]);

  //  return positions, positions_quality, z_max_ind
}

void InferRotationZ(EigenOdometry* positions, float* elevation, int* zset, Eigen::Matrix3f* rotation) {
    int mm0_ind = zset[0];
    int mm1_ind = zset[1];
    int mm2_ind = zset[2];

    float elev0 = elevation[mm0_ind];
    float elev1 = elevation[mm1_ind];
    float elev2 = elevation[mm2_ind];
    auto mm0 = positions[mm0_ind];
    auto mm1 = positions[mm1_ind];
    auto mm2 = positions[mm2_ind];

    float projection;
    float d = EuclideanNorm(&mm0);
    if(d > elev0)
        projection = sqrt(pow(d, 2) - pow(elev0, 2));
    else
        projection = 0;

    /*print("infer_rotation_matrix_z / elev0={}, d={}, pr={}".format(elev0, d, projection))
    print("----------------")
    print("mm0_ind      ", mm0_ind)
    print("mm1_ind      ", mm1_ind)
    print("elevations[0]", elevations[0])
    print("elev0 (mm0)  ", elev0)
    print("elev1 (mm1)  ", elev1)
    print("----------------") */

    // pre rotation: rotate only around z-axis, s.t. a0 points towards the direction of y-axis
    EigenOdometry a0 = mm0; a0.position[2] = 0;
    a0 = UnitVector(&a0);
    a0 = a0 * DotProduct(&mm0, &a0); // projection of mm0 onto z-plane
    EigenOdometry b0; b0.position[0] = 0; b0.position[1] = 1; b0.position[2] = 0;
    // get axis of rotation by cross-product of a0, b0
    EigenOdometry axis0 = CrossProduct(&a0, &b0);
    float angle0;
    Eigen::Matrix3f rotation0;
    //print("a0:{}, b0:{}, axis0:{}".format(a0, b0, axis0))
    if(EuclideanNorm(&axis0) < 0.0001) // a0 is already aligned with b0, use identity matrix for first rotation
    {
        //print("a0 is already aligned with b0, use identity matrix for pre rotation")
        rotation0 = IdentityMatrix();
    }
    else
    {
        axis0 = axis0 / EuclideanNorm(&axis0);
        // get angle of rotation by scalar-product of a0, b0
        angle0 = acos(DotProduct(&a0, &b0) / (EuclideanNorm(&a0) * EuclideanNorm(&b0)));
        rotation0 = RotationMatrixFromAxisAngle(&axis0, &angle0);
    }

    //positions0 = rotate_positions(positions, rotation0)
    auto tmp = positions[mm0_ind] * rotation0;
/*
    # ## first rotation: align point a1=mm0 to b1=(0, projection, elev0)
    a1 = positions0[mm0_ind]
    b1 = [0, projection, elev0]
    a1 = a1 / np.linalg.norm(a1) # divide by norm to get unit vector
    b1 = b1 / np.linalg.norm(b1) # divide by norm to get unit vector
    # get axis of rotation by cross-product of a1, b1
    axis1 = np.cross(a1, b1)
    print("a1:{}, b1:{}, axis1:{}".format(a1, b1, axis1))
    if abs(np.linalg.norm(axis1)) < 0.000001: # a1 is already aligned with b1, use identity matrix for first rotation
        #print("a1 is already aligned with b1, use identity matrix for first rotation")
        rotation1 = np.identity(3)
    else:
        axis1 = axis1 / np.linalg.norm(axis1)
        # get angle of rotation by scalar-product of a1, b1
        angle1 = np.arccos(np.dot(a1, b1) / (np.linalg.norm(a1) * np.linalg.norm(b1)))
        rotation1 = rotation_matrix_helper(axis1, angle1)
/*

    print("----------------")
#    print("rotation1:\n{}".format(rotation1))
#    print("after rotation0 and 1:\n{}".format(rotate_positions(positions, np.matmul(rotation1, rotation0))))

    rotation01 = np.matmul(rotation1, rotation0)

    # ########################################

    # ## helper rotation: rotation axis to Y-axis
    bb1 = b1
    h2 = np.array([0, 1, 0])
    axis2 = np.cross(bb1, h2)
    axis2 = axis2 / np.linalg.norm(axis2)
    # get angle of rotation by scalar-product of bb1, h2
    angle2 = np.arccos(np.dot(bb1, h2) / (np.linalg.norm(bb1) * np.linalg.norm(h2)))
    if axis2[0] < 0:
        axis2 = -axis2
        angle2 = -angle2
    rotation2 = rotation_matrix_helper(axis2, angle2)
    mm0_tmp = np.matmul(np.matmul(rotation2, rotation01), mm0)
    mm1_tmp = np.matmul(np.matmul(rotation2, rotation01), mm1)
    print("angle2: {}, axis2: {} (must be x-axis)".format(angle2, axis2)) # must be x-axis
    print("mm0_tmp: {} (must only have y-value != 0)".format(mm0_tmp))
    print("mm1_tmp: {}".format(mm1_tmp))
    print("----------------")

    h = elev1 / np.sin(-angle2)
    b = np.tan(-angle2) * (h - mm1_tmp[1])
    l = np.sqrt(pow(mm1_tmp[2], 2) + pow(mm1_tmp[0], 2))
    print("h: {}, b: {}, l: {}".format(h, b, l))
    x2 = np.sqrt(pow(mm1_tmp[2], 2) + pow(mm1_tmp[0], 2) - pow(b, 2))
    print("h: {}, b: {}, x2: {},".format(h, b, x2))
    print("----------------")

    # ## second rotation: align point x
    #print("a2:{}, b2:{}".format(a2, b2))
    # get axis as vector b1
    axis3 = b1
    axis3 = axis3 / np.linalg.norm(axis3)
    #axis2 = np.array([0, 1, 0])
    # get angle of rotation
    print("np.arctan2(mm1_tmp[2], mm1_tmp[0]) = ", np.arctan2(mm1_tmp[2], mm1_tmp[0]))
    print("np.arctan2(b, x2) = ", np.arctan2(b, x2))
    print("np.arctan2(b, -x2) = ", np.arctan2(b, -x2))

    atanp = np.arctan2(b, x2)
    atanm = np.arctan2(b, -x2)

    angle3p = np.arctan2(mm1_tmp[2], mm1_tmp[0]) - atanp
    angle3m = np.arctan2(mm1_tmp[2], mm1_tmp[0]) - atanm
    print("angle3p: {}, angle3m: {}, axis3: {}".format(angle3p, angle3m, axis3))
    rotation3pp = rotation_matrix_helper(axis3, angle3p)
    rotation3mp = rotation_matrix_helper(axis3, -angle3p)
    rotation3pm = rotation_matrix_helper(axis3, angle3m)
    rotation3mm = rotation_matrix_helper(axis3, -angle3m)

    # determine direction of rotation
    mm1_tmp3pp = np.matmul(np.matmul(rotation3pp, rotation01), positions[mm1_ind])
    mm1_tmp3mp = np.matmul(np.matmul(rotation3mp, rotation01), positions[mm1_ind])
    mm1_tmp3pm = np.matmul(np.matmul(rotation3pm, rotation01), positions[mm1_ind])
    mm1_tmp3mm = np.matmul(np.matmul(rotation3mm, rotation01), positions[mm1_ind])
    mm2_tmp3pp = np.matmul(np.matmul(rotation3pp, rotation01), positions[mm2_ind])
    mm2_tmp3mp = np.matmul(np.matmul(rotation3mp, rotation01), positions[mm2_ind])
    mm2_tmp3pm = np.matmul(np.matmul(rotation3pm, rotation01), positions[mm2_ind])
    mm2_tmp3mm = np.matmul(np.matmul(rotation3mm, rotation01), positions[mm2_ind])
    print("mm1_tmp3pp:{}, mm2_tmp3pp:{}".format(abs(mm1_tmp3pp[2] - elev1), abs(mm2_tmp3pp[2] - elev2)))
    print("mm1_tmp3pm:{}, mm2_tmp3pm:{}".format(abs(mm1_tmp3pm[2] - elev1), abs(mm2_tmp3pm[2] - elev2)))
    print("mm1_tmp3mp:{}, mm2_tmp3mp:{}".format(abs(mm1_tmp3mp[2] - elev1), abs(mm2_tmp3mp[2] - elev2)))
    print("mm1_tmp3mm:{}, mm2_tmp3mm:{}".format(abs(mm1_tmp3mm[2] - elev1), abs(mm2_tmp3mm[2] - elev2)))
    # use rotation version with smallest error for tie-breaker drone
    tmp_err = abs(mm2_tmp3pp[2] - elev2)
    rotation3 = rotation3pp
    if abs(mm2_tmp3pm[2] - elev2) < tmp_err:
        tmp_err = abs(mm2_tmp3pm[2] - elev2)
        rotation3 = rotation3pm
    if abs(mm2_tmp3mp[2] - elev2) < tmp_err:
        tmp_err = abs(mm2_tmp3mp[2] - elev2)
        rotation3 = rotation3mp
    if abs(mm2_tmp3mm[2] - elev2) < tmp_err:
        tmp_err = abs(mm2_tmp3mm[2] - elev2)
        rotation3 = rotation3mm

    rotation = np.matmul(rotation3, rotation01)
    # Decomposing rotation matrix https://nghiaho.com/?page_id=846
    phi_x = np.arctan2(rotation[2][1], rotation[2][2])
    phi_y = np.arctan2(-rotation[2][0], np.sqrt(pow(rotation[2][1], 2) - pow(rotation[2][2], 2)))
    phi_z = np.arctan2(rotation[1][0], rotation[0][0])

    # ## last rotation: compensate rotation along z-axis from first rotation, s.t. coresponding rotation does not rotate along z-axis
    rotation_z = np.array([
            [np.cos(-phi_z), -np.sin(-phi_z), 0],
            [np.sin(-phi_z),  np.cos(-phi_z), 0],
            [             0,               0, 1]
                        ])

    rotation_total = np.matmul(rotation_z, rotation)
    rotation = rotation_total
    # Decomposing rotation matrix https://nghiaho.com/?page_id=846
    phi_x = np.arctan2(rotation[2][1], rotation[2][2])
    phi_y = np.arctan2(-rotation[2][0], np.sqrt(pow(rotation[2][1], 2) - pow(rotation[2][2], 2)))
    phi_z = np.arctan2(rotation[1][0], rotation[0][0])

    rotated = rotate_positions(positions, rotation_total)

    for i in range(0, nquads):
        print("mm0_ind:{}, mm1_ind:{}, i:{}, elevation:{}, z:{}".format(mm0_ind, mm1_ind, i, elevations[i], rotated[i][2]))

    print("phi_x:{}".format(phi_x))
    print("phi_y:{}".format(phi_y))
    print("phi_z:{}".format(phi_z))

    return rotation_total
    */
}

void SwarmStateEstimator::CheckDistances(float (*distances)[N_DRONES_MAX], EigenOdometry* positions) {
    float eps = 0.01;

    for (size_t i = 0; i < droneCount_; i++)
        for (size_t j = 0; j < droneCount_; j++)
        {
            float dist = Distance(&positions[i], &positions[j]);
            if(abs(distances[i][j] - dist) > eps)
                ROS_INFO("CheckDistances (%d) distance missmatch at i:%d j:%d dist:%f positions:%f", droneNumber_, i, j, distances[i][j], dist);
            else
                ROS_INFO_ONCE("CheckDistances (%d) distance ok at        i:%d j:%d dist:%f positions:%f", droneNumber_, i, j, distances[i][j], dist);
        }
}

float Distance(EigenOdometry* a, EigenOdometry* b) {
    return sqrt(pow(a->position[0] - b->position[0], 2) +
                pow(a->position[1] - b->position[1], 2) +
                pow(a->position[2] - b->position[2], 2));
}

float EuclideanNorm(EigenOdometry* a) {
    return sqrt(pow(a->position[0], 2) +
                pow(a->position[1], 2) +
                pow(a->position[2], 2));
}

EigenOdometry UnitVector(EigenOdometry* a) {
    EigenOdometry b = *a / EuclideanNorm(a);
    return b;
}

EigenOdometry CrossProduct(EigenOdometry* a, EigenOdometry* b)
{
    EigenOdometry r;
    r.position[0] = a->position[1]*b->position[2] - a->position[2]*b->position[1];
    r.position[1] = a->position[2]*b->position[0] - a->position[0]*b->position[2];
    r.position[2] = a->position[0]*b->position[1] - a->position[1]*b->position[0];
    return r;
}

float DotProduct(EigenOdometry* a, EigenOdometry* b)
{
    return a->position[0]*b->position[0] + a->position[1]*b->position[1] + a->position[2]*b->position[2];
}

EigenOdometry operator-(const EigenOdometry& a, const EigenOdometry& b)
{
    EigenOdometry r;
    r.position[0] = a.position[0] - b.position[0];
    r.position[1] = a.position[1] - b.position[1];
    r.position[2] = a.position[2] - b.position[2];
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

EigenOdometry operator*(const Eigen::Matrix3f& b, const EigenOdometry& a)
{
//  tmp1 = rotation0 * tmp0;
    EigenOdometry r;
    ROS_INFO("my operator* b-a");
    r.position[0] = a.position[0] * b(0,0) + a.position[1] * b(0,1) + a.position[2] * b(0,2);
    r.position[1] = a.position[0] * b(1,0) + a.position[1] * b(1,1) + a.position[2] * b(1,2);
    r.position[2] = a.position[0] * b(2,0) + a.position[1] * b(2,1) + a.position[2] * b(2,2);
    return r;
}

EigenOdometry operator*(const EigenOdometry& a, const Eigen::Matrix3f& b)
{
    EigenOdometry r;
    ROS_INFO("my operator* a-b");
    r.position[0] = a.position[0] * b(0,0) + a.position[1] * b(0,1) + a.position[2] * b(0,2);
    r.position[1] = a.position[0] * b(1,0) + a.position[1] * b(1,1) + a.position[2] * b(1,2);
    r.position[2] = a.position[0] * b(2,0) + a.position[1] * b(2,1) + a.position[2] * b(2,2);
    return r;
}

Eigen::Matrix3f RotationMatrixFromAxisAngle(EigenOdometry* axis, float* angle)
{
    float n1 = axis->position[0];
    float n2 = axis->position[1];
    float n3 = axis->position[2];

    float c1 = cos(*angle);
    float s1 = sin(*angle);
    // https://de.wikipedia.org/wiki/Drehmatrix
    Eigen::Matrix3f rotation;
      /*
            {n1*n1*(1-c1) + c1,    n1*n2*(1-c1) - n3*s1, n1*n3*(1-c1) + n2*s1},
            {n2*n1*(1-c1) + n3*s1, n2*n2*(1-c1) + c1,    n2*n3*(1-c1) - n1*s1},
            {n3*n1*(1-c1) - n2*s1, n3*n2*(1-c1) + n1*s1, n3*n3*(1-c1) + c1}
          }; */
    rotation(0,0) = n1*n1*(1-c1) + c1;
    rotation(0,1) = n1*n2*(1-c1) - n3*s1;
    rotation(0,2) = n1*n3*(1-c1) + n2*s1;

    rotation(1,0) = n2*n1*(1-c1) + n3*s1;
    rotation(1,1) = n2*n2*(1-c1) + c1;
    rotation(1,2) = n2*n3*(1-c1) - n1*s1;

    rotation(2,0) = n3*n1*(1-c1) - n2*s1;
    rotation(2,1) = n3*n2*(1-c1) + n1*s1;
    rotation(2,2) = n3*n3*(1-c1) + c1;
    return rotation;
}

Eigen::Matrix3f IdentityMatrix()
{
    Eigen::Matrix3f rotation;

    rotation(0,0) = 0;
    rotation(0,1) = 1;
    rotation(0,2) = 0;

    rotation(1,0) = 0;
    rotation(1,1) = 1;
    rotation(1,2) = 0;

    rotation(2,0) = 0;
    rotation(2,1) = 0;
    rotation(2,2) = 1;
    return rotation;
}

std::string toString(const Eigen::Matrix3f& a)
{
    std::stringstream ss;
    ss << "[[ " << a(0,0) << ", " << a(0,1) << ", " << a(0,2) << " ]" << std::endl;
    ss << " [ " << a(1,0) << ", " << a(1,1) << ", " << a(1,2) << " ]" << std::endl;
    ss << " [ " << a(2,0) << ", " << a(2,1) << ", " << a(2,2) << " ]]" << std::endl;
    return ss.str();
}

std::string toString(const EigenOdometry& a)
{
    std::stringstream ss;
    ss << "[ " << a.position[0] << ", " << a.position[1] << ", " << a.position[3] << " ]" << std::endl;
    return ss.str();
}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "swarm_state_estimator_node");

    ros::NodeHandle nh2;

    rotors_control::SwarmStateEstimator swarm_state_estimator_node;

    ros::spin();

    return 0;
}
