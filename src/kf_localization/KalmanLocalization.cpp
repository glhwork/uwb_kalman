#include "uwb_kalman/KalmanLocalization.h"

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

namespace kalman {

KalmanLocalization::KalmanLocalization(ros::NodeHandle n,
                                       std::string file_address_1,
                                       std::string file_address_2) {
  filter_state = Init;
  config_flag = false;
  info_vec.clear();
  p_cov = Eigen::VectorXd::Zero(6);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
  
  ReadConfig(file_address_1);
  ReadAcc(file_address_2);
  ReadCov(file_address_2);
  
  filter.GetAnchorConfig(anchor_posi);
  
  // struct 'Acceleration' is defined in 'KalmanFilter.cpp'
  Acceleration a;
  a.x = acc(0);
  a.y = acc(1);
  a.z = acc(2);
  filter.GetAcceleration(a);  
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

void KalmanLocalization::ReadCov(std::string address) {
  YAML::Node cov_info = YAML::LoadFile(address);
  p_cov << cov_info["cov_x"].as<double>(),
           cov_info["cov_y"].as<double>(),
           cov_info["cov_z"].as<double>(),
           cov_info["cov_vx"].as<double>(),
           cov_info["cov_vy"].as<double>(),
           cov_info["cov_vz"].as<double>();
           
  cov_ob = cov_info["cov_ob"].as<double>();
}
  
void KalmanLocalization::InitState(const sensor_msgs::Range& range) {

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
    if (0 == count) {
      info_vec.push_back(info);
    }
  }
  
  if (3 == info_vec.size() && Init == filter_state) {
    InitPos(info_vec);
    info_vec.clear();
   // filter_state = OnFilter;
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
    
    int index_i = info_vec[i].id - 101;
    x1 = anchor_posi(index_i,0);
    y1 = anchor_posi(index_i,1);
    z1 = anchor_posi(index_i,2);
    d1 = info_vec[i].range;
    
    for (size_t j = i+1; j < n; j++) {

      int index_j = info_vec[j].id - 101;
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
  for (int i = 0; i < p_cov.rows(); i++) {
    init_cov(i,i) = p_cov(i);
  }
  
  filter.StateInit(init_state, init_cov);
  
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = init_state(0);
  odom.pose.pose.position.y = init_state(1);
  odom.pose.pose.position.z = init_state(2);
  
  odom_pub.publish(odom);
  
  std::cout << "Initial position is " << " [ x: " << init_state(0) 
                                      << " y: "   << init_state(1)
                                      << " z: "   << init_state(2) 
                                      << " ]"     << std::endl;
  
}

void KalmanLocalization::GetPosition(const sensor_msgs::Range& range) {
  
  int uwb_seq = std::stoi(range.header.frame_id) - 101;
  double snr = 20.0 * std::log10(range.max_range / range.min_range + 0.001);  
  if (snr < 10) {
    return;
  }
  if (Init == filter_state) {
    pre_time = range.header.stamp.toSec();
  }
  if (Init == filter_state) {
    InitState(range);
    if (OnFilter == filter_state) {
      std::cout << "Init successfully" << std::endl;
    }
    return;
  }
  
  double cur_time;
  if (OnFilter == filter_state) {
    cur_time = range.header.stamp.toSec();
    double delta_t = cur_time - pre_time;
    if (delta_t > 2.0) {
      std::cout << "long time!!!" << std::endl;
      filter_state = Init;
      return;
    }
    filter.StatePredict(delta_t);
    
    Eigen::MatrixXd obser_cov = Eigen::MatrixXd::Zero(1,1);
    Eigen::MatrixXd distance = Eigen::MatrixXd::Zero(1,1);
    obser_cov(0,0) = cov_ob;    
    distance(0,0) = range.range;
    
    Eigen::MatrixXd pre_err = filter.GetError(uwb_seq, distance);
    filter.StateUpdate(uwb_seq, obser_cov);
    pre_time = cur_time;
  }
  
  Eigen::VectorXd final_state = filter.GetState();
  Eigen::MatrixXd final_cov = filter.GetCov();
  
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = final_state(0);
  odom.pose.pose.position.y = final_state(1);
  odom.pose.pose.position.z = final_state(2);
  
  odom_pub.publish(odom);
  std::cout << "position is " << " [ x: " << final_state(0) 
                              << " y: "   << final_state(1)
                              << " z: "   << final_state(2) 
                              << " ]"     << std::endl;
  
}

}; // namespace kalman
