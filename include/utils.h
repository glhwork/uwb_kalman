//
// Created by m1 on 18-1-31.
//

#ifndef MONITOR_UTILS_H
#define MONITOR_UTILS_H

#include <vector>
#include <Eigen/Geometry>

Eigen::Matrix3d rpy2DCM(const Eigen::Vector3d &rpy);
Eigen::Vector3d quat2rpy(const Eigen::Quaterniond &q);
std::vector<std::string> split(std::string str, std::string pattern);
float HEX2double(std::string str);

#endif //MONITOR_UTILS_H
