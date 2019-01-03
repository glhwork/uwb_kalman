#ifndef KALMANLOCALIZATION_H
#define KALMANLOCALIZATION_H

#include <string>
#include <vector>
#include <Eigen/Dense>

#include "yaml-cpp/yaml.h"
#include "sensor_msgs/Range.h"
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
  KalmanLocalization(std::string file_address_1,
                     std::string file_address_2);
  ~KalmanLocalization() {}
  
  // read the config file of anchors
  void ReadConfig(std::string file_address);
  void ReadAcc(std::string address);
  
  // initial the state
  void InitState(const sensor_msgs::Range& range);
  
  // compute the initial position
  void InitPos(const std::vector<IdRange>& info_vec);
  
 private:
  KalmanFilter filter;
  State filter_state;
  Eigen::Matrix<double, 4, 3> anchor_posi;
  Eigen::Vector3d acc;
  Eigen::Vector3d velo;
  bool config_flag;


}; // class KalmanLocalization

}; // namespace kalman

#endif

