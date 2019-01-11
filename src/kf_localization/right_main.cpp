#include "uwb_kalman/KalmanLocalization.h"

using kalman::KalmanLocalization;
using kalman::KalmanFilter;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "kalman_main_a");
  ros::NodeHandle n;
  
  std::string comm_add = "/home/george/renov_ws/src/uwb_kalman/data";
  std::string address_1 = comm_add + "/anchor.yaml";
  std::string address_2 = comm_add + "/acceleration.yaml";
  
  KalmanLocalization localizer(n, 1, address_1, address_2);
  ros::Subscriber range_sub = n.subscribe("uwb_data_a", 100,
                                          &KalmanLocalization::GetPositionCallBack, 
                                          &localizer);
  ros::spin();
  return 0;
}

