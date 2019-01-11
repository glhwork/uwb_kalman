#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#define PI 3.14159265

struct Position {
  Position(double a, double b) : x(a), y(b) {}
  double x;
  double y;
};

class OdomFusion {
 public:
  OdomFusion(ros::NodeHandle n) {
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  }
  ~OdomFusion() {}
  void GetLeftInfo(const nav_msgs::Odometry& left_side);
  void GetRightInfo(const nav_msgs::Odometry& right_side);
  void Fusion();
  double Norm(Eigen::Vector2d& vec);
  
 private:
  nav_msgs::Odometry odom_left;
  nav_msgs::Odometry odom_right;
  ros::Publisher odom_pub;

};

void OdomFusion::GetLeftInfo(const nav_msgs::Odometry& left_side) {
  odom_left = left_side;
}

void OdomFusion::GetRightInfo(const nav_msgs::Odometry& right_side) {
  odom_right = right_side;
}

double OdomFusion::Norm(Eigen::Vector2d& vec) {
  double s = vec(0) * vec(0) + vec(1) * vec(1);
  return sqrt(s);
}

void OdomFusion::Fusion() {
  double t_left = odom_left.header.stamp.toSec();
  double t_right = odom_right.header.stamp.toSec();
  
  double t_dif = fabs(t_left - t_right);
  if (t_dif < 0.2) {
    Position p_l(odom_left.pose.pose.position.x,
                 odom_left.pose.pose.position.y); 
    Position p_r(odom_right.pose.pose.position.x,
                 odom_right.pose.pose.position.y);
    
    Eigen::Vector2d uwb_v((p_r.x - p_l.x), (p_r.y - p_l.y));
    Eigen::Vector2d x_axis(1, 0);
    double angle = acos(uwb_v.dot(x_axis) / (Norm(uwb_v)*Norm(x_axis)));
    double yaw;
    if (p_r.y < p_l.y) {
      yaw = -fabs(angle) + (PI / 2);
    } else {
      yaw = fabs(angle) + (PI / 2);
    }
    Eigen::Matrix2d T;
    T.Zero();
    T(0,0) = cos(yaw);
    T(0,1) = -sin(yaw);
    T(1,0) = sin(yaw);
    T(1,1) = cos(yaw);
    
    // v_r : the position of the right uwb w.r.t. vehicle frame
    Position v_r(-487.2, -584.4);
    
    Position final_p((p_r.x - v_r.x * T(0,0) - v_r.y * T(0,1)),
                     (p_r.y - v_r.x * T(1,0) - v_r.y * T(1,1)));
    
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = final_p.x;
    odom.pose.pose.position.y = final_p.y;
    odom.pose.pose.position.z = yaw;
    
    odom_pub.publish(odom);
    
  }
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "kalman_main");
  ros::NodeHandle n;
  
  OdomFusion od_fu(n);
  ros::Subscriber left_sub = n.subscribe("odom_left", 100, &OdomFusion::GetLeftInfo, &od_fu);
  ros::Subscriber right_sub = n.subscribe("odom_right", 100, &OdomFusion::GetRightInfo, &od_fu);
  
  


}
