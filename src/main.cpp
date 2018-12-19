//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________
#include <csignal>
#include "uwblocalization.h"

//bool k_IntSingnal = false;
//void my_func(int sign_no) {
//  if (sign_no == SIGINT) {
//    k_IntSingnal = true;
//  }
//}
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "uwb_localization");
  ros::NodeHandle n;
  //signal(SIGINT, my_func);
  uwblocalization uwblocater(n);
  while(ros::ok()){
    ros::spinOnce();
  }
  return 0;
}
