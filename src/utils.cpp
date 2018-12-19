//
// Created by m1 on 18-1-31.
//

#include "utils.h"

float HEX2double(std::string str)
{
  float ret;
  int i=0;
  sscanf(str.c_str(),"%x",&i);
  ret=*((float *)&i);
  return ret;
}

Eigen::Vector3d quat2rpy(const Eigen::Quaterniond &q) {
  double roll, pitch, yaw;

  roll = atan2((2. * q.y() * q.z() + 2. * q.w() * q.x()),
               (2. * q.w() * q.w() + 2. * q.z() * q.z() - 1.));
  pitch = -asin(2. * q.x() * q.z() - 2. * q.w() * q.y());
  yaw = atan2((2. * q.x() * q.y() + 2. * q.w() * q.z()),
              (2. * q.w() * q.w() + 2. * q.x() * q.x() - 1.));
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Matrix3d rpy2DCM(const Eigen::Vector3d &rpy) {
  double DCM[9];
  DCM[0] = cos(rpy[1]) * cos(rpy[2]);
  DCM[3] = sin(rpy[0]) * sin(rpy[1]) * cos(rpy[2]) - cos(rpy[0]) * sin(rpy[2]);
  DCM[6] = cos(rpy[0]) * sin(rpy[1]) * cos(rpy[2]) + sin(rpy[0]) * sin(rpy[2]);
  DCM[1] = cos(rpy[1]) * sin(rpy[2]);
  DCM[4] = sin(rpy[0]) * sin(rpy[1]) * sin(rpy[2]) + cos(rpy[0]) * cos(rpy[2]);
  DCM[7] = cos(rpy[0]) * sin(rpy[1]) * sin(rpy[2]) - sin(rpy[0]) * cos(rpy[2]);
  DCM[2] = -sin(rpy[1]);
  DCM[5] = sin(rpy[0]) * cos(rpy[1]);
  DCM[8] = cos(rpy[0]) * cos(rpy[1]);
  Eigen::Matrix3d ret;
  ret << DCM[0], DCM[3], DCM[6],
          DCM[1], DCM[4], DCM[7],
          DCM[2], DCM[5], DCM[8];
  return ret;
}

std::vector<std::string> split(std::string str, std::string pattern) {
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;
  std::string::size_type size = str.size(), i;

  for (i = 0; i < size; i++) {
    pos = str.find(pattern, i);
    if (pos < size) {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}