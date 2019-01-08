#include "uwb_kalman/KalmanFilter.h"

using kalman::KalmanFilter;
using kalman::KalmanInit;
using kalman::Acceleration;
using kalman::StateResult;

KalmanFilter::KalmanFilter() {

  state_cov = Eigen::MatrixXd::Identity(6, 6);
  state = Eigen::VectorXd(6);
  
  config_flag = false;
  acc_flag = false;
  init_flag = false;
 
}

void KalmanFilter::GetAnchorConfig(const Eigen::MatrixXd& config) {
  // anchor_config is a n-by-3 matrix
  // n is quantity of uwb anchors
  // 3 is (x, y, z) coordinates 
  anchor_config = config;
  std::cout << "get anchor config successfully!" << std::endl;
  config_flag = true;
}

void KalmanFilter::GetAcceleration(const Acceleration& acc) {
  a.x = acc.x;
  a.y = acc.y;
  a.z = acc.z;
  std::cout << "get acceleration successfully!" << std::endl;
  acc_flag = true;
}

Eigen::MatrixXd KalmanFilter::GetJacobian(const int uwb_seq,
 				  	  const Eigen::VectorXd& cur_state) {
  
  Eigen::Matrix<double, 1, 6> h = Eigen::MatrixXd::Zero(1, 6);
  if (config_flag) {
  
    double denomi;
    denomi = sqrt(pow((anchor_config(uwb_seq, 0) - cur_state(0)), 2) +
                  pow((anchor_config(uwb_seq, 1) - cur_state(1)), 2) +
                  pow((anchor_config(uwb_seq, 2) - cur_state(2)), 2));

    h(0, 0) = (anchor_config(uwb_seq, 0) - cur_state(0)) / denomi;
    h(0, 1) = (anchor_config(uwb_seq, 1) - cur_state(1)) / denomi;
    h(0, 2) = (anchor_config(uwb_seq, 2) - cur_state(2)) / denomi;
  
  } else {
    std::cerr << "i don't get the config of anchors!!!" << std::endl;
    exit(1);
  }
  
  return h;  
}

Eigen::MatrixXd KalmanFilter::GetNoiseCov(const double delta_t) {
  
  Eigen::MatrixXd q = Eigen::MatrixXd::Identity(6, 6);
  if (acc_flag) {
    Eigen::VectorXd wt(6);
    wt << 0.5 * pow(delta_t, 2) * a.x,
          0.5 * pow(delta_t, 2) * a.y,
          0.5 * pow(delta_t, 2) * a.z,
          delta_t * a.x,
          delta_t * a.y,
          delta_t * a.z;
    q = wt * wt.transpose();
  } else {
    std::cerr << "i don't get the acceleration!!!" << std::endl;
    exit(1);
  }
  
  return q;
}

void KalmanFilter::StateInit(const Eigen::VectorXd& init_state, 
                             const Eigen::MatrixXd& init_cov) {

  state = init_state;
  state_cov = init_cov;
  init_flag = true;
               
}

void KalmanFilter::StatePredict(const double delta_t) {
  
  if (init_flag) {
    // define the state transformation matrix 
    // between states at continuous moments
    int f_row = state_cov.rows();
    int f_col = state_cov.cols();
    Eigen::MatrixXd trans_m = Eigen::MatrixXd::Identity(f_row, f_col);
    trans_m(0,3) = 
    trans_m(1,4) = 
    trans_m(2,5) = delta_t;
  
    // state prediction
    state = trans_m * state;
    // covariance update
    Eigen::MatrixXd q = GetNoiseCov(delta_t);
    state_cov = trans_m * state_cov * trans_m.transpose() + q;  
  }
}

void KalmanFilter::StateUpdate(const int uwb_seq,
			       const Eigen::MatrixXd& obser_cov) {
  
  if (init_flag) {
    Eigen::VectorXd cur_state = state;
    Eigen::MatrixXd h = GetJacobian(uwb_seq, cur_state);
  
    // Kt = P * H^t * (H * P * H^t + Rt)^-1
    // k_t : kalman gain e.t. Kt
    Eigen::MatrixXd matrix_denomi = h * state_cov * h.transpose() + obser_cov;
    Eigen::MatrixXd k_t;
    k_t = state_cov * h.transpose() * matrix_denomi.inverse();

    // x = x + Kt(z - H * x)
    // P = (I - Kt * H) * P
    state = state + k_t * error;
    state_cov = state_cov - k_t * h * state_cov;
  }
  
}

Eigen::MatrixXd KalmanFilter::GetError(const int uwb_seq,
                                       const Eigen::MatrixXd& range) {
  Eigen::VectorXd cur_state = state;
  Eigen::MatrixXd h = GetJacobian(uwb_seq, cur_state);
  
  error = range - h * state;
  
  return error;  
}

Eigen::MatrixXd KalmanFilter::ComputeError(const int uwb_seq,
                                           const Eigen::MatrixXd& range,
                                           const Eigen::VectorXd& x) {

  Eigen::MatrixXd h = GetJacobian(uwb_seq, x);
  Eigen::MatrixXd err = range - h * x;
  
  return err;

}











