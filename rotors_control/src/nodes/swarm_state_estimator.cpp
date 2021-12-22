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
        ROS_INFO_ONCE("ElevationCallback drone#%d elevation=%f.", (int)i, elevation_[i]);
    }
}

void SwarmStateEstimator::DistancesCallback(const std_msgs::Float32MultiArray& distances_msg) {
    ROS_INFO_ONCE("DistancesCallback got distances message.");

    for (size_t i = 0; i < droneCount_; i++)
    {
        odometry_estimate_[i][0] = 0;
        odometry_estimate_[i][1] = 0;
        odometry_estimate_[i][2] = 0;
    }

    for (size_t i = 0; i < droneCount_; i++)
    {
        for (size_t j = 0; j < droneCount_; j++)
        {
            distances_[i][j] = distances_msg.data[i*droneCount_ + j];
            ROS_INFO_ONCE("DistancesCallback drone#%d -> drone#%d: distance=%f.", (int)i, (int)j, distances_[i][j]);
        }
    }

    if(droneNumber_ == 0)
    {
        Matrix3f tmp;

        FindBestTriangle(distances_, best_triangle_);
        ROS_INFO_ONCE("DistancesCallback (%d) best_triangle_ = %d / %d / %d / %d", droneNumber_, best_triangle_[0], best_triangle_[1], best_triangle_[2], best_triangle_[3]);
        InferPositions(distances_, best_triangle_, odometry_estimate_);
        FindBestZset(distances_, odometry_estimate_, best_zset_);
        InferRotationZ(odometry_estimate_, elevation_, best_zset_, &tmp);
        ROS_INFO("DistancesCallback (%d) best_zset_ = %d / %d / %d", droneNumber_, best_zset_[0], best_zset_[1], best_zset_[2]);
        CheckDistances(distances_, odometry_estimate_);

/*
        Vector3f axis0 = {1,2,3};
        axis0 = axis0 / axis0.norm();
        float angle0 = 0.3;
        auto rotation0 = RotationMatrixFromAxisAngle(axis0, angle0);
        ROS_INFO("RotationMatrixFromAxisAngle %s", MatrixToString(rotation0).c_str());

        Vector3f tmp0 = {7,8,9};
        auto tmp1 = rotation0 * tmp0;
        ROS_INFO("tmp0 %s", VectorToString(tmp0).c_str());
        ROS_INFO("tmp1 %s", VectorToString(tmp1).c_str());
*/
    }

    if(droneNumber_ == 0) // move red indicator spheres to indicate determined positions
    {
        for (size_t i = 0; i < droneCount_; i++)
        {
            geometry_msgs::Point pr2_position;
            // pr2_position.x = odometry_gt_.position[0] + sin(visual_cnt_)* 0.1;
            // pr2_position.y = odometry_gt_.position[1] + cos(visual_cnt_)* 0.1;
            // pr2_position.z = odometry_gt_.position[2];
            pr2_position.x = odometry_gt_.position[0] + odometry_estimate_[i][0];
            pr2_position.y = odometry_gt_.position[1] + odometry_estimate_[i][1];
            pr2_position.z = odometry_gt_.position[2] + odometry_estimate_[i][2];
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

void SwarmStateEstimator::FindBestZset(float (*distances)[N_DRONES_MAX], Vector3f* positions, int* zset) {
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
    Vector3f md = positions[maxdist_ind] / positions[maxdist_ind].norm();
    float maxproj = 0;
    int maxproj_ind = -1;
    for (size_t i = 0; i < droneCount_; i++)
    {
        if(i == maxdist_ind)
            continue;
        float dist = (positions[i] - md * positions[i].dot(md)).norm();
        if(dist > maxproj)
        {
            maxproj = dist;
            maxproj_ind = i;
        }
    }

    // find third drone as tie-breaker (use drone with the largest projected distance to the vector (own drone, maxproj_ind))
    Vector3f mdd = positions[maxproj_ind] / positions[maxproj_ind].norm();
    float tie_breaker = 0;
    int tie_breaker_ind = -1;
    for (size_t i = 0; i < droneCount_; i++)
    {
        if(i == maxdist_ind)
            continue;
        if(i == maxproj_ind)
            continue;
        float dist = (positions[i] - mdd * positions[i].dot(mdd)).norm();
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

void SwarmStateEstimator::InferPositions(float (*distances)[N_DRONES_MAX], int* triangle, Vector3f* positions) {
    int a = triangle[1];
    int b = triangle[2];
    int c = triangle[3];
    // int c = -1 # this saves the index of the fourth quadcopter to tie-break the roots (it is optional, if not given, the first quadcopter not in the triangle is used)

    float positions_quality[N_DRONES_MAX];

    positions[triangle[0]][0] = 0; // relative x-coordinate
    positions[triangle[0]][1] = 0; // relative y-coordinate
    positions[triangle[0]][2] = 0; // relative z-coordinate
    positions_quality[triangle[0]] = 3; // high quality of point (part of triangle)

    ROS_INFO_ONCE("InferPositions (%d) tr0_:%d at x:%f y:%f z:%f", droneNumber_, triangle[0], positions[triangle[0]][0], positions[triangle[0]][1], positions[triangle[0]][2]);

    positions[triangle[1]][0] = distances[triangle[0]][triangle[1]]; // relative x-coordinate
    positions[triangle[1]][1] = 0; // relative y-coordinate
    positions[triangle[1]][2] = 0; // relative z-coordinate
    positions_quality[triangle[1]] = 3; // high quality of point (part of triangle)

    ROS_INFO_ONCE("InferPositions (%d) tr1_:%d at x:%f y:%f z:%f", droneNumber_, triangle[1], positions[triangle[1]][0], positions[triangle[1]][1], positions[triangle[1]][2]);

    // https://en.wikipedia.org/wiki/True-range_multilateration#Two_Cartesian_dimensions,_two_measured_slant_ranges_(Trilateration)
    positions[triangle[2]][0] = ( // relative x-coordinate
                                            pow(distances[triangle[0]][triangle[2]], 2) -
                                            pow(distances[triangle[1]][triangle[2]], 2) +
                                            pow(distances[triangle[0]][triangle[1]], 2)
                                          ) / (2*distances[triangle[0]][triangle[1]]);
    float t1 = (pow(distances[triangle[0]][triangle[2]], 2) - pow(positions[triangle[2]][0], 2));
    if (t1 < 0) // should not be the case, but it can happen because noisy measurements: sum of two edges of the triangle is shorter than the third edge
        positions[triangle[2]][1] = 0.001; // relative y-coordinate
    else
        positions[triangle[2]][1] = sqrt(pow(distances[triangle[0]][triangle[2]], 2) - pow(positions[triangle[2]][0], 2)); // relative y-coordinate
    positions[triangle[2]][2] = 0; // relative z-coordinate
    positions_quality[triangle[2]] = 3; // high quality of point (part of triangle)
    ROS_INFO_ONCE("InferPositions (%d) tr2_:%d at x:%f y:%f z:%f", droneNumber_, triangle[2], positions[triangle[2]][0], positions[triangle[2]][1], positions[triangle[2]][2]);

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

        positions[i][0] = ( // relative x-coordinate
                                      pow(distances[triangle[0]][i], 2) -
                                      pow(distances[triangle[1]][i], 2) +
                                      pow(distances[triangle[0]][triangle[1]], 2)
                                    ) / (2 * distances[triangle[0]][triangle[1]]);
        positions[i][1] = (
                                      pow(distances[triangle[0]][i], 2) -
                                      pow(distances[triangle[2]][i], 2) +
                                      pow(distances[triangle[0]][triangle[2]], 2) -
                                      2*positions[triangle[2]][0]*positions[i][0]
                                    ) / (2*positions[triangle[2]][1]);
        float t1 = (pow(distances[triangle[0]][i], 2) - pow(positions[i][0], 2) - pow(positions[i][1], 2));
        if(t1 > 0)
            positions[i][2] = sqrt(t1);
        else
            positions[i][2] = 0;

        if(abs(positions[i][2]) > z_max)
        {
            z_max = abs(positions[i][2]);
            z_max_ind = i;
        }
        positions_quality[i] = positions[i][2]; // quality of point proportional to z elevation
        ROS_INFO_ONCE("InferPositions (%d) loop:%d at x:%f y:%f z:%f", droneNumber_, (int)i, positions[i][0], positions[i][1], positions[i][2]);

        // use fourth drone (tie-breaker) to check if relative z-coordinate may be flipped
        if(c >= 0)
        {
            Vector3f pos_neg = positions[i];
            pos_neg[2] = 0 - positions[i][2];
            float dist_pos = (positions[c] - positions[i]).norm();
            float dist_neg = (positions[c] - pos_neg).norm();
            if(abs(dist_neg - distances[c][i]) < abs(dist_pos - distances[c][i]))
                positions[i][2] = pos_neg[2];
        }
        if(c < 0)
            c = i;
    }

    for (size_t i = 0; i < droneCount_; i++)
        ROS_INFO_ONCE("InferPositions (%d) drone:%d at x:%f y:%f z:%f", droneNumber_, (int)i, positions[i][0], positions[i][1], positions[i][2]);

  //  return positions, positions_quality, z_max_ind
}

void SwarmStateEstimator::InferRotationZ(Vector3f* positions, float* elevation, int* zset, Eigen::Matrix3f* rotation) {
    int mm0_ind = zset[0];
    int mm1_ind = zset[1];
    int mm2_ind = zset[2];

    float elev0 = elevation[mm0_ind] - elevation[droneNumber_];
    float elev1 = elevation[mm1_ind] - elevation[droneNumber_];
    float elev2 = elevation[mm2_ind] - elevation[droneNumber_];
    auto mm0 = positions[mm0_ind];
    auto mm1 = positions[mm1_ind];
    auto mm2 = positions[mm2_ind];

    float projection;
    float d = mm0.norm();
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

    // ///////////// pre rotation: rotate only around z-axis, s.t. a0 points towards the direction of y-axis
    Vector3f a0 = mm0; a0[2] = 0;
    a0 = a0 / a0.norm(); // divide by norm to get unit vector
    a0 = a0 * mm0.dot(a0); // projection of mm0 onto z-plane
    Vector3f b0 = {0, 1, 0};
    // get axis of rotation by cross-product of a0, b0
    Vector3f axis0 = a0.cross(b0);
    float angle0;
    Matrix3f rotation0;
    //print("a0:{}, b0:{}, axis0:{}".format(a0, b0, axis0))
    if(axis0.norm() < 0.0001) // a0 is already aligned with b0, use identity matrix for first rotation
    {
        //print("a0 is already aligned with b0, use identity matrix for pre rotation")
        rotation0 = IdentityMatrix();
    }
    else
    {
        axis0 = axis0 / axis0.norm(); // divide by norm to get unit vector
        angle0 = acos(a0.dot(b0) / (a0.norm() * b0.norm())); // get angle of rotation by scalar-product of a0, b0
        rotation0 = RotationMatrixFromAxisAngle(axis0, angle0);
    }

    //RotatePositions
    //positions0 = RotatePositions(positions, rotation0)
    //Vector3f tmp = positions[mm0_ind] * rotation0;

    //result = rotation * positions[i];

    // ///////////// first rotation: align point a1=mm0 to b1=(0, projection, elev0)
    Vector3f a1 = rotation0 * positions[mm0_ind];
    Vector3f b1 = {0, projection, elev0};
    a1 = a1 / a1.norm(); // divide by norm to get unit vector
    b1 = b1 / b1.norm(); // divide by norm to get unit vector
    Vector3f axis1 = a1.cross(b1); // get axis of rotation by cross-product of a1, b1
    float angle1;
    Matrix3f rotation1;
    //print("a1:{}, b1:{}, axis1:{}".format(a1, b1, axis1))
    if(axis1.norm() < 0.0001) // a1 is already aligned with b1, use identity matrix for first rotation
        rotation1 = IdentityMatrix();
    else
    {
        axis1 = axis1 / axis1.norm(); // divide by norm to get unit vector
        angle1 = acos(a1.dot(b1) / (a1.norm() * b1.norm())); // get angle of rotation by scalar-product of a0, b0
        rotation1 = RotationMatrixFromAxisAngle(axis1, angle1);
      }

    Matrix3f rotation01 = rotation1 * rotation0;

    // ///////////// helper rotation: rotation axis to Y-axis
    Vector3f bb1 = b1;
    Vector3f h2 = {0, 1, 0};
    Vector3f axis2 = bb1.cross(h2);
    axis2 = axis2 / axis2.norm(); // divide by norm to get unit vector
    float angle2 = acos(bb1.dot(h2) / (bb1.norm() * h2.norm())); // get angle of rotation by scalar-product of bb1, h2
    if(axis2[0] < 0)
    {
        axis2 = -axis2;
        angle2 = -angle2;
    }
    Matrix3f rotation2 = RotationMatrixFromAxisAngle(axis2, angle2);
    Vector3f mm0_tmp = (rotation2 * rotation01) * mm0;
    Vector3f mm1_tmp = (rotation2 * rotation01) * mm1;

    float h = elev1 / sin(-angle2);
    float b = tan(-angle2) * (h - mm1_tmp[1]);
    float l = sqrt(pow(mm1_tmp[2], 2) + pow(mm1_tmp[0], 2));
    float x2 = sqrt(pow(mm1_tmp[2], 2) + pow(mm1_tmp[0], 2) - pow(b, 2));

    // /////////////  second rotation: align point x
    Vector3f axis3 = b1;
    axis3 = axis3 / axis3.norm(); // divide by norm to get unit vector
    // get angle of rotation
    float atanp = atan2(b, x2);
    float atanm = atan2(b, -x2);

    // TODO: check if less options are feasable
    float angle3p = atan2(mm1_tmp[2], mm1_tmp[0]) - atanp;
    float angle3m = atan2(mm1_tmp[2], mm1_tmp[0]) - atanm;
    Matrix3f rotation3pp = RotationMatrixFromAxisAngle(axis3, angle3p);
    Matrix3f rotation3mp = RotationMatrixFromAxisAngle(axis3, -angle3p);
    Matrix3f rotation3pm = RotationMatrixFromAxisAngle(axis3, angle3m);
    Matrix3f rotation3mm = RotationMatrixFromAxisAngle(axis3, -angle3m);

    // determine direction of rotation
    Vector3f mm2_tmp3pp = (rotation3pp * rotation01) * positions[mm2_ind];
    Vector3f mm2_tmp3mp = (rotation3mp * rotation01) * positions[mm2_ind];
    Vector3f mm2_tmp3pm = (rotation3pm * rotation01) * positions[mm2_ind];
    Vector3f mm2_tmp3mm = (rotation3mm * rotation01) * positions[mm2_ind];
    // use rotation version with smallest error for tie-breaker drone
    float tmp_err = abs(mm2_tmp3pp[2] - elev2);
    Matrix3f rotation3 = rotation3pp;
    if(abs(mm2_tmp3pm[2] - elev2) < tmp_err)
    {
        tmp_err = abs(mm2_tmp3pm[2] - elev2);
        rotation3 = rotation3pm;
    }
    if(abs(mm2_tmp3mp[2] - elev2) < tmp_err)
    {
        tmp_err = abs(mm2_tmp3mp[2] - elev2);
        rotation3 = rotation3mp;
    }
    if(abs(mm2_tmp3mm[2] - elev2) < tmp_err)
    {
        tmp_err = abs(mm2_tmp3mm[2] - elev2);
        rotation3 = rotation3mm;
    }

    Matrix3f rotation9 = rotation3 * rotation01;

    for (size_t i = 0; i < droneCount_; i++)
       positions[i] = rotation9 * positions[i];

/*
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

void SwarmStateEstimator::CheckDistances(float (*distances)[N_DRONES_MAX], Vector3f* positions) {
    float eps = 0.01;

    for (size_t i = 0; i < droneCount_; i++)
        for (size_t j = 0; j < droneCount_; j++)
        {
            float dist = (positions[i] - positions[j]).norm();
            if(abs(distances[i][j] - dist) > eps)
                ROS_INFO("CheckDistances (%d) distance missmatch at i:%d j:%d dist:%f positions:%f", droneNumber_, (int)i, (int)j, distances[i][j], dist);
            else
                ROS_INFO_ONCE("CheckDistances (%d) distance ok at        i:%d j:%d dist:%f positions:%f", droneNumber_, (int)i, (int)j, distances[i][j], dist);
        }
}

void SwarmStateEstimator::RotatePositions(Vector3f* positions, Eigen::Matrix3f* rotation, Vector3f* result)
{
   for (size_t i = 0; i < droneCount_; i++)
      result[i] = (*rotation) * positions[i];
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

Eigen::Matrix3f RotationMatrixFromAxisAngle(const Eigen::Vector3f& axis, const float& angle)
{
    float n1 = axis[0];
    float n2 = axis[1];
    float n3 = axis[2];

    float c1 = cos(angle);
    float s1 = sin(angle);
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

std::string MatrixToString(const Eigen::Matrix3f& a)
{
    std::stringstream ss;
    /*
    ss << "[[ " << a(0,0) << ", " << a(0,1) << ", " << a(0,2) << " ]" << std::endl;
    ss << " [ " << a(1,0) << ", " << a(1,1) << ", " << a(1,2) << " ]" << std::endl;
    ss << " [ " << a(2,0) << ", " << a(2,1) << ", " << a(2,2) << " ]]" << std::endl;
*/
    return ss.str();
}

std::string VectorToString(const Eigen::Vector3f& a)
{
    std::stringstream ss;
    ss << "[ " << a[0] << ", " << a[1] << ", " << a[2] << " ]" << std::endl;
    return ss.str();
}

std::string toString(const EigenOdometry& a)
{
    std::stringstream ss;
    ss << "[ " << a.position[0] << ", " << a.position[1] << ", " << a.position[2] << " ]" << std::endl;
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
