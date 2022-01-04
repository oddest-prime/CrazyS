/*
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

#include "rotors_control/Eigen.h"

#include <math.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <inttypes.h>

namespace rotors_control{

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

      rotation(0,0) = 1;
      rotation(0,1) = 0;
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
      ss << "[ " << a[0] << ", " << a[1] << ", " << a[2] << " ]"; // << std::endl;
      return ss.str();
  }

  std::string toString(const EigenOdometry& a)
  {
      std::stringstream ss;
      ss << "[ " << a.position[0] << ", " << a.position[1] << ", " << a.position[2] << " ]" << std::endl;
      return ss.str();
  }

}
