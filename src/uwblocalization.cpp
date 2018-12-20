//
// Created by unmanned_tractor on 18-1-3.
//
#include "uwblocalization.h"

typedef std::vector<std::string> StringVec;

#define UNMANNED 1
#define RAND(a,b) ((rand()%(b-a+1))+a-1)

uwblocalization::uwblocalization(ros::NodeHandle& nh) {
  state_ = INIT;
  observe_N_ = 3;
  sqrerr_ = 0.0;
  counts_ = 0;
  preid_  = -1;
  countofpreerr_ = 0;
  observer_.reserve(observe_N_);
  obser_cov_ = Eigen::MatrixXd::Zero(1,1);
  obser_cov_(0,0) = 1;
  num = 0;
  //n_.getParam("obser_cov_", obser_cov_(0,0));
  
  range_sub_ = nh.subscribe<sensor_msgs::Range>("/uwb_raw_data", 1, &uwblocalization::OnRange, this);
  uwb_pub_ = nh.advertise<nav_msgs::Odometry>("/uwb_localization_result_with_ekf", 10);
  odom_path_pub_ = nh.advertise<nav_msgs::Path>("/odom_path", 1);
  
  //ParameterReader param("/home/m1/Desktop/cat_tmp/src/uwb_localization/include/uwb_test/parameters.txt");
#if UNMANNED
  ParameterReader param("/home/george/renov_ws/src/uwb_kalman/uwbindoor.txt");
#else
  ParameterReader param("/home/george/Desktop/uwbindoor");
#endif

  if (0 == param.status_) {
    std::cerr << "parameter file does not exist." << std::endl;
    exit(0);
  }
  
  uwbanchors_N_ = std::atoi(param.getData("Neighbors").c_str());
  std::cout << "Neighbors = " << uwbanchors_N_ << std::endl;

  for (int i = 0; i < uwbanchors_N_; i++) {
    char nodeid[4];
    sprintf(nodeid, "%d", i + 101);
    StringVec v_anchorcoordinate = split(param.getData(nodeid),",");
    
   // PositionModel pm(atof(v_anchorcoordinate[0].c_str()),
    //                 atof(v_anchorcoordinate[1].c_str()),
   //                  atof(v_anchorcoordinate[2].c_str()));
    // v_pm_ : vector of PositionModel
   // v_pm_.push_back(pm);
    Eigen::Vector3d anchorcoordinate_tmp(atof(v_anchorcoordinate[0].c_str()),
                                         atof(v_anchorcoordinate[1].c_str()),
                                         atof(v_anchorcoordinate[2].c_str()));  
    // vector of coordinates of all the anchors
    uwbanchors_.emplace_back(anchorcoordinate_tmp);
  }
  Init();
  
}

uwblocalization::~uwblocalization() {
  ros::shutdown();
}

void uwblocalization::Init() {

  
  int rows = uwbanchors_.size();
  int cols = uwbanchors_[0].rows();
  Eigen::MatrixXd config = Eigen::MatrixXd::Identity(rows, cols);
  for (size_t i = 0; i < uwbanchors_.size(); i++) {
    for (int j = 0; j < uwbanchors_[0].rows(); j++) {
      config(i, j) = uwbanchors_[i](j);     	
    }  	
  }
  kf_.GetAnchorConfig(config);
  
  kalman::Acceleration acc;
  acc.x =
  acc.y =
  acc.z = 1;
  /*n_.getParam("acc_x", acc.x);
  n_.getParam("acc_y", acc.y);
  n_.getParam("acc_z", acc.z);*/
  kf_.GetAcceleration(acc);

}

/*
snr : signal noise ratio

*/

void uwblocalization::OnRange(const sensor_msgs::RangeConstPtr &uwb) {
  int uwbid = std::stoi(uwb->header.frame_id) - 101;
  double snr = 20.0 * std::log10(uwb->max_range / uwb->min_range + 0.001);
  
  // calculate the snr to determine whether this value should be droped
  std::cout << "<<========================================>>" << std::endl;
  std::cout << "round " << num << std::endl;
  num++;
  if(snr < 10.0) {
    //std::cout << uwb->header.frame_id <<  " low snr( " << snr << " ) !!!" << std::endl;
    return;
  }
  //std::cout << "id " << uwbid + 101 << " snr = " << snr << "; range = " << uwb->range << std::endl;

  vuwbidqueue_.push_back(uwbid);
  if (state_ == INIT) {
    pretime_ = uwb->header.stamp.toSec();
  }
  int validanchor = ValidAnchors();
  if (validanchor >= 3) {
    double curtime = uwb->header.stamp.toSec();
    double range = uwb->range;
//    std::cout << "anchor id: " << uwbid << ", range: " << range << std::endl;

    double deltatime = curtime - pretime_;
    if (fabs(deltatime) > 2.0) {
      deltatime = 0.0;
      state_ = LOST;
      //InitPos();
      Init();
    }
    std::cout << "<<before push============================>>" << std::endl;
    std::cout << "before push" << observer_.size() << std::endl;
    if (state_ != ONFILTER) {
      observer_.emplace_back(uwb);
      
      // ============================
      if (ValidObs() && InitPos()) {
        ;
      } else {
        std::cout << "Init failed!!!" << std::endl;
        return;
      }
    }
//    if (uwbid % 2 == 0) {
//      return;
//    }
   
    
    // x_pre = F * x + B * u + (wt = 0)
    // error_pre = H * x_pre - z
    kf_.StatePredict(deltatime);
    Eigen::Matrix<double, 1, 1> uwb_range;
    uwb_range(0, 0) = range;
    Eigen::MatrixXd error_pre = kf_.GetError(uwbid, uwb_range);

    //std::cout << "pre error: " << error_pre(0,0) << std::endl;
    if (state_ == ONFILTER && fabs(error_pre(0,0)) > 0.3) {
      std::cout << "large pre error !!!" << std::endl;

      if (++countofpreerr_ > 4) {
        state_ = LOST;
      }
      return;
    }
    if (uwbid == preid_) {
      std::cout << "uwbid == preid !!!" << std::endl;
      return;
    }

    //PosCov pc;
   // int coff = range < 50.0 ? int(range / 10.0) + 1 : 5;
    //coff =1;
    //double measerror = coff * 0.05;
   // pc(0,0)=pow(measerror, 2);
//    v_pm_[uwbid].setCovariance(pc);

    kf_.StateUpdate(uwbid, obser_cov_);
    Eigen::VectorXd x_kf = kf_.GetState();
    Eigen::MatrixXd p = kf_.GetCov();

    preid_ = uwbid;
    pretime_ = curtime;

//    printf("ekf :%f,%f; %f,%f;\n", x_kf[0], sqrt(p(0, 0)), x_kf[1], sqrt(p(1, 1)));
    printf("ekf: %f, %f, %f, %f, %f, %f\n", x_kf[0], x_kf[1], x_kf[2], x_kf[3], x_kf[4], x_kf[5]);
    countofpreerr_ = 0;

    Eigen::MatrixXd error = kf_.ComputeError(uwbid, uwb_range, x_kf);

    sqrerr_ += pow(error(0,0), 2);
    counts_ ++;
    double rms = sqrt(sqrerr_ / static_cast<double>(counts_));
    //std::cout << "anchor id: " << uwbid + 101 << ", range: " << range
    //          << "; error: " << error(0)
    //          << "; rms: " << rms
     //         << std::endl;
    if(rms > 0.5 || fabs(error(0)) > 1.0) {
      state_ = LOST;
    }

    if(state_ == ONFILTER && uwbid == 2 && fabs(error(0)) < 0.2 ) {
      nav_msgs::Odometry msg;
      msg.header.frame_id = "map";
      msg.header.stamp = uwb->header.stamp;
      msg.child_frame_id = "uwb";
      msg.pose.pose.position.x = x_kf[0];
      msg.pose.pose.position.y = x_kf[1];
      msg.pose.pose.position.z = 0.0;
      msg.pose.covariance[0] = p(0, 0);
      msg.pose.covariance[7] = p(1, 1);
      msg.pose.covariance[14] = p(2, 2);
      uwb_pub_.publish(msg);
      OdomPublish(msg);
    }
  }
}

int uwblocalization::ValidAnchors() {
  int ret = 0;
  int count = 0;
  for (int i = 0; i < uwbanchors_N_; i++) {
    if (vuwbidqueue_.size() <= i) {
      break;
    }
    count |= (1 << vuwbidqueue_[vuwbidqueue_.size() - 1 - i]);
  }
  for (ret =0; count; ++ret)
  {
    count &= (count -1) ; //每次消除最右边的1，当n为0结束
  }
  return ret;
}

int uwblocalization::ValidObs() {
  int ret = 1;
  if (observe_N_ < 3) {
    ret = 0;
    return ret;
  }
  std::cout << "size inf valid obs : " << observer_.size() << std::endl;
  if (observer_.size() == observe_N_) {
    int count = 0, count_id = 0;
    count |= (1 << (vuwbidqueue_[vuwbidqueue_.size() - 1 - 0]) + 1);
    double snr = 20.0 * std::log10(observer_[0]->max_range / observer_[0]->min_range);
    if (snr < 30.0) {
      //ret = 0;
    }
    for (int i = 1; i < observe_N_; i++) {
      double snr = 20.0 * std::log10(observer_[i]->max_range / observer_[i]->min_range);
      if (snr < 30.0) {
        //ret = 0;
      }
      count |= (1 << (vuwbidqueue_[vuwbidqueue_.size() - 1 - i]) + 1);
      double deltatime = observer_[i]->header.stamp.toSec() - observer_[i - 1]->header.stamp.toSec();
      if (deltatime > 0.15) {
        ret = 0;
        break;
      }
    }
    for (count_id =0; count; ++count_id)
    {
      count &= (count -1) ; //每次消除最右边的1，当n为0结束
    }
    if (count_id != observe_N_) {
      ret = 0;
    }
//    if (observer_[0]->header.frame_id == "101") {
//      ret = 0;
//    }
    if (ret == 0) {
      observer_.erase(observer_.begin());
    }
  } else {
    ret = 0;
  }

  return ret;
}

bool uwblocalization::InitPos() {

  std::cout << "Start computing the initial value of this filter" << std::endl;
  Eigen::Matrix<double, 7, 3> B;
  Eigen::Matrix<double, 7, 1> l, v;
  B.setZero();
  l.setZero();
  Eigen::Vector3d X0(0, 0, 0.1);
  Eigen::Vector3d x_lsq;
  x_lsq.setZero();
  int count = 0;
  // ========================
  std::cout << "size of observer : " << observer_.size() << std::endl;
 /* if (observer_.size() < 3)
  {
  	return false;
  }*/
  do {
    for (int i = 0; i < observe_N_; i++) {
      int index = std::stoi(observer_[i]->header.frame_id) - 101;
      Eigen::Vector3d anchor = uwbanchors_[index];
      Eigen::Vector3d t = anchor - X0;
      double dis = t.norm() - observer_[i]->range;
      t.normalize();
      B(i, 0) = t[0];
      B(i, 1) = t[1];
      B(i, 2) = t[2];
      l(i, 0) = dis;
    }
    Eigen::Matrix3d btmp = B.transpose() * B;
    Eigen::Vector3d ltmp = B.transpose() * l;
    x_lsq = (B.transpose() * B).inverse() * (B.transpose() * l);
    X0 += x_lsq;
    X0[2] = 0.1;
    count ++;
  } while(count < 100 && sqrt(x_lsq[0] * x_lsq[0] + x_lsq[1] * x_lsq[1]) > 0.5);
  std::cout << "finish while loop" << std::endl;
  v = B * x_lsq - l;
  double err = 0;
  for (int i = 0; i < uwbanchors_N_; i++) {
    err += pow(v[i], 2);
  }
  err /= static_cast<double>(uwbanchors_N_);
  if (err > 1.0 || x_lsq.hasNaN()) {
    observer_.erase(observer_.begin());
    return false;
  }
  Eigen::VectorXd x(6);
  x.setZero();
  x(0) = X0[0];
  x(1) = X0[1];
  x(2) = X0[2];
  x(3) = x(4) = x(5) = 0;
  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(6, 6);
  
  kf_.StateInit(x, p);
  observer_.clear();
  state_ = ONFILTER;
  return true;
}

double uwblocalization::CalZ(const double& y) {
  double a0, a1, b1, w;
  a0 = 2.466;
  a1 = -2.512;
  b1 = 0.07777;
  w = 0.06993;
  double z = a0 + a1 * cos(y * w) + b1 * sin(y * w) + 1.55;
  return z;
}

void uwblocalization::OdomPublish(const nav_msgs::Odometry &odom) {
  odom_Path_.header = odom.header;
  odom_Path_.header.frame_id = "/world";
  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header = odom.header;
  odom_pose.header.frame_id = "/world";
  odom_pose.pose = odom.pose.pose;
  odom_Path_.poses.push_back(odom_pose);
  odom_path_pub_.publish(odom_Path_);
}
