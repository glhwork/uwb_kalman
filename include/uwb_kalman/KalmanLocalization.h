#ifndef KALMANLOCALIZATION_H
#define KALMANLOCALIZATION_H

#include <string>
#include <vector>
#include <Eigen/Dense>

#include "yaml-cpp/yaml.h"
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "KalmanFilter.h"

namespace kalman {

enum State {
  Init,
  Failed,
  OnFilter,
};

struct IdRange {
  int id;
  double range;
};



class KalmanLocalization {
 public:
  KalmanLocalization(ros::NodeHandle n,
                     int uwb_side,
                     std::string file_address_1,
                     std::string file_address_2);
  KalmanLocalization() {}
  ~KalmanLocalization() {}
  
  // read the config file of anchors
  void ReadConfig(std::string file_address);
  void ReadAcc(std::string address);
  void ReadCov(std::string address);
  
  // initial the state
  void InitState(const sensor_msgs::Range& range);
  
  // compute the initial position
  void InitPos(const std::vector<IdRange>& info_vec);
  
  void GetPositionCallBack(const sensor_msgs::Range& range);
  
 private:
  KalmanFilter filter;
  State filter_state;
  std::vector<IdRange> info_vec;
  Eigen::Matrix<double, 4, 3> anchor_posi;
  
  Eigen::Vector3d acc;
  Eigen::Vector3d velo;
  Eigen::VectorXd p_cov; 
  double cov_ob;
  
  double pre_time;
  bool config_flag;
  ros::Publisher odom_pub;


}; // class KalmanLocalization

}; // namespace kalman

#endif

