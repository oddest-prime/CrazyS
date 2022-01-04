/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef INCLUDE_ROTORS_CONTROL_EIGEN_H_
#define INCLUDE_ROTORS_CONTROL_EIGEN_H_

#include <assert.h>
#include <Eigen/Eigen>
#include "rotors_control/common.h"

namespace rotors_control {
  using namespace Eigen;

  float Distance(EigenOdometry* a, EigenOdometry* b);
  float EuclideanNorm(EigenOdometry* a);
  EigenOdometry CrossProduct(EigenOdometry* a, EigenOdometry* b);
  float DotProduct(EigenOdometry* a, EigenOdometry* b);

  EigenOdometry operator-(const EigenOdometry& a, const EigenOdometry& b);
  EigenOdometry operator+(const EigenOdometry& a, const EigenOdometry& b);
  EigenOdometry operator*(const EigenOdometry& a, const float& b);
  EigenOdometry operator/(const EigenOdometry& a, const float& b);
  EigenOdometry operator*(const Eigen::Matrix3f& b, const EigenOdometry& a);
  EigenOdometry operator*(const EigenOdometry& a, const Eigen::Matrix3f& b);

  Eigen::Matrix3f RotationMatrixFromAxisAngle(const Eigen::Vector3f& axis, const float& angle);
  Eigen::Matrix3f IdentityMatrix();
  std::string MatrixToString(const Eigen::Matrix3f& a);
  std::string VectorToString(const Eigen::Vector3f& a);
  std::string toString(const EigenOdometry& a);
}

#endif /* INCLUDE_ROTORS_CONTROL_EIGEN_H_ */
