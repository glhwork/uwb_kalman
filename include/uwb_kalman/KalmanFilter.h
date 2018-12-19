#include <iostream>
#include <Eigen/Dense>

// state_cov : The covariance matrix of the state
// state : The updated state of this filter

namespace kalman {

struct KalmanInit {
  KalmanInit(int N) {
    init_state = Eigen::VectorXd(6);
    uwb_config = Eigen::MatrixXd::Identity(N, 3);
  }
  // call this function if restart the initialization process
  void Reset() {
    init_state.setZero();
  }
  Eigen::VectorXd init_state;
  Eigen::MatrixXd uwb_config;

}; // struct KalmanInit

// struct Acceleration defines a variable to contain the 
// acceleration of each direction e.t. noise of each direction
struct Acceleration {
  Acceleration() 
    : x(0.0), y(0.0), z(0.0) {}
  double x;
  double y;
  double z;
}; // struct Acceleration


// provide the state and its covariance in one single variable
struct StateResult {
  StateResult() {
    state_cov = Eigen::MatrixXd::Identity(6, 6);
    state = Eigen::VectorXd(6);
  }
  
  Eigen::MatrixXd state_cov;
  Eigen::VectorXd state;

}; // struct StateResult

class KalmanFilter {
 public:
  KalmanFilter();
  ~KalmanFilter() {}
  
  // Get the position configuration of anchors
  void GetAnchorConfig(const Eigen::MatrixXd& config);
  void GetAcceleration(const Acceleration& acc);
  
  // Jacobian e.t. H matrix
  // Transformation matrix between state and observation
  Eigen::MatrixXd GetJacobian(const int uwb_seq, 
                              const Eigen::VectorXd& cur_state);
  
  // NoiseCov e.t. Q matrix
  // The Covariance matrix of Gussian Noise of the state
  Eigen::MatrixXd GetNoiseCov(const double delta_t);
  
  // initial the state and covariance matrix of this filter
  void StateInit(const Eigen::VectorXd& init_state,
                 const Eigen::MatrixXd& init_cov);
                 
  // Prediction process of kalman filter
  void StatePredict(const double delta_t);
  
  // Updating process of kalman filter
  void StateUpdate(const int uwb_seq,
                   const Eigen::MatrixXd& obser_cov);
  
  // Computing the error in advance
  Eigen::MatrixXd GetError(const int uwb_seq,
                           const Eigen::MatrixXd& range);
  
  // Only used to computer the error after data fusion        
  Eigen::MatrixXd ComputeError(const int uwb_seq,
                               const Eigen::MatrixXd& range,
                               const Eigen::VectorXd& x);

  Eigen::VectorXd GetState() const { return state; }
  Eigen::MatrixXd GetCov() const { return state_cov; }
  
 private:
  Eigen::MatrixXd state_cov;
  Eigen::VectorXd state;
  Eigen::MatrixXd anchor_config;
  Acceleration a;
  Eigen::MatrixXd error;
  
  bool config_flag;
  bool acc_flag; 
  bool init_flag;

}; // class KalmanFilter

}; // namespace kalman
