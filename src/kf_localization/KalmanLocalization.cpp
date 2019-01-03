#include "uwb_kalman/KalmanLocalization.h"

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

namespace kalman {

KalmanLocalization::KalmanLocalization(std::string file_address_1,
                                       std::string file_address_2) {
  filter_state = Failed;
  config_flag = false;
  
  ReadConfig(file_address_1);
  ReadAcc(file_address_2);  
}

void KalmanLocalization::ReadConfig(std::string file_address) {

  YAML::Node config = YAML::LoadFile(file_address);
  //config.LoadFile(file_address);
  
  anchor_posi << config["101_x"].as<double>(), config["101_y"].as<double>(),
                 config["101_z"].as<double>(),
                 config["102_x"].as<double>(), config["102_y"].as<double>(),
                 config["102_z"].as<double>(),
                 config["103_x"].as<double>(), config["103_y"].as<double>(),
                 config["103_z"].as<double>(),
                 config["104_x"].as<double>(), config["104_y"].as<double>(),
                 config["104_z"].as<double>();
  
  config_flag = true;
  
}

void KalmanLocalization::ReadAcc(std::string address) {

  YAML::Node acc_info = YAML::LoadFile(address);
  acc << acc_info["ax"].as<double>(), 
         acc_info["ay"].as<double>(), 
         acc_info["az"].as<double>();
      
  velo << acc_info["vx"].as<double>(), 
          acc_info["vy"].as<double>(), 
          acc_info["vz"].as<double>();
  
}
  
void KalmanLocalization::InitState(const sensor_msgs::Range& range) {

  std::vector<IdRange> info_vec;
  IdRange info;
  info.id = std::stoi(range.header.frame_id);
  info.range = range.range;
  
  int count = 0;
  if (0 == info_vec.size()) {
    info_vec.push_back(info);
  } else {
    for (size_t i = 0; i < info_vec.size(); i++) {
      if (info_vec[i].id == info.id) {
        count++;
      }
    }
    if (count > 0) {
      info_vec.push_back(info);
    }
  }
  
  if (3 == info_vec.size() && Failed == filter_state) {
    InitPos(info_vec);
    info_vec.clear();
    filter_state = OnFilter;
  }
  
}
  
void KalmanLocalization::InitPos(const std::vector<IdRange>& info_vec) {
  
  Eigen::Vector3d location;
  size_t n, rows;
  int count = 0;

  // "n" refers to quantity of UWB stations
  // "rows" refers to the quantity of equations
  n = info_vec.size();
  rows = n * (n-1) / 2;
  Eigen::MatrixXd m(rows, 3);
  Eigen::VectorXd result(rows);

  for (size_t i = 0; i < n; i++) {

    double x1, x2, y1, y2, z1, z2;
    double d1, d2;
    
    int index_i = 101 - info_vec[i].id;
    x1 = anchor_posi(index_i,0);
    y1 = anchor_posi(index_i,1);
    z1 = anchor_posi(index_i,2);
    d1 = info_vec[i].range;

    for (size_t j = i+1; j < n; j++) {

      int index_j = 101 - info_vec[j].id;
      x2 = anchor_posi(index_j,0);
      y2 = anchor_posi(index_j,0);
      z2 = anchor_posi(index_j,0);
      d2 = info_vec[j].range;

      m(count, 0) = x1 - x2;
      m(count, 1) = y1 - y2;
      m(count, 2) = z1 - z2;
      result(count) = (pow(x1, 2) - pow(x2, 2) +
                       pow(y1, 2) - pow(y2, 2) +
                       pow(z1, 2) - pow(z2, 2) +
                       pow(d2, 2) - pow(d1, 2)) / 2;
      count++;
    }    
  }	

  location = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(result);
  
  Eigen::VectorXd init_state(6);
  init_state << location, velo;
  Eigen::MatrixXd init_cov = Eigen::MatrixXd::Identity(6,6);
  filter.StateInit(init_state, init_cov);
  
  std::cout << "position init success!!" << std::endl;
}

}; // namespace kalman
