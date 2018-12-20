//
// Created by unmanned_tractor on 18-1-3.
//

#ifndef UWB_LOCALIZATION_UWBLOCALIZATION_H
#define UWB_LOCALIZATION_UWBLOCALIZATION_H

//_____________________________________________________________________________
//
// #includes
//_____________________________________________________________________________

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <sstream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#ifdef WIN32
#include <conio.h>
#else // linux
#include <time.h>
#endif
#include <vector>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <sensor_msgs/Range.h>

#include "uwb_test/rnSampleApp.h"
#include "uwb_test/rcmIf.h"
#include "uwb_test/rcm.h"
#include "uwb_test/rn.h"
#include "param.h"

#include "uwb_kalman/KalmanFilter.h"

#include "utils.h"

#define ON_LINE 1
#define EKF 1

typedef enum {
  INIT,
  ONFILTER,
  LOST,
} KFState;

struct InitState {
  InitState() : flag(false) {}  
  bool flag;
  Eigen::VectorXd state;
  Eigen::MatrixXd state_cov;
};

class uwblocalization {
 public:
  uwblocalization(ros::NodeHandle& nh);
  uwblocalization() {}
  ~uwblocalization();
  void Init();
  double CalZ(const double&);
 private:
  void OnRange(const sensor_msgs::RangeConstPtr& range);
  int ValidAnchors();
  int ValidObs();
  bool InitPos();
  void OdomPublish(const nav_msgs::Odometry& odom);

  kalman::KalmanFilter kf_;

  // uwbanchors_N_ : the quantity of uwb anchors, calculated by iterator
  int uwbanchors_N_;
  std::vector<Eigen::Vector3d> uwbanchors_;
  //pre time
  double pretime_;

  nav_msgs::Path odom_Path_;
  //
  std::vector<int> vuwbidqueue_;

  //pub and sub
  ros::Subscriber range_sub_;
  ros::Publisher uwb_pub_;
  ros::Publisher odom_path_pub_;

  //
  int observe_N_;
  std::vector<sensor_msgs::RangeConstPtr> observer_;

  //statistics
  double sqrerr_;
  int counts_;
  int preid_;

  int countofpreerr_;

  KFState state_;
  
  Eigen::MatrixXd obser_cov_;
  
  int num;
};


#endif //UWB_LOCALIZATION_UWBLOCALIZATION_H
