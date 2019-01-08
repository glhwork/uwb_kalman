//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef WIN32
#include <conio.h>
#else // linux
#include <time.h>
#endif
#include <chrono>
#include <vector>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "uwb_test/rnSampleApp.h"
#include "uwb_test/rcmIf.h"
#include "uwb_test/rcm.h"
#include "uwb_test/rn.h"
//custom msg
//#include <custom_msg/UWB.h>
#include <sensor_msgs/Range.h>
//yaml

#include <yaml-cpp/yaml.h>
#include <uwb_test/hostInterfaceRN.h>
#include <uwb_test/hostInterfaceRCM.h>



extern "C" {
std::vector<std::string> split(std::string str, std::string pattern);
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "uwb_driver");
  ros::NodeHandle n;
  ros::Publisher uwb_pub = n.advertise<sensor_msgs::Range>("uwb_raw_data", 1000);
  ros::Rate loop_rate(10);
  int infoMsgType;

  //char msgType[100];
  char ip[15]="192.168.0.100";
  int status;
  //int valid = 0;

  rcmMsg_FullRangeInfo rangeInfo;
  rnMsg_GetFullNeighborDatabaseConfirm ndbInfo;
  printf("step 1 Initialize the interface to the RCM.\n");
  // initialize the interface to the RCM
  if (rcmIfInit(rcmIfIp, ip) != OK)
  {
    printf("Initialization failed.\n");
    exit(0);
  }
  printf("step 2 Set sleep mode idle.\n");
  if (rcmSleepModeSet(RCRM_SLEEP_MODE_IDLE) != 0)
  {
    printf("Time out waiting for sleep mode set 1.\n");
    exit(0);
  }
  printf("step 3 Execute Built-In Test.\n");
  // execute Built-In Test - verify that radio is healthy
  if (rcmBit(&status) != 0)
  {
    printf("Time out waiting for BIT.\n");
    
    exit(0);
  }
  if (status != OK)
  {
    printf("Built-in test failed - status %d.\n", status);
    exit(0);
  }
  printf("step 4 Set sleep mode active.\n");
  if (rcmSleepModeSet(RCRM_SLEEP_MODE_ACTIVE) != 0)
  {
    printf("Time out waiting for sleep mode set 2.\n");
    exit(0);
  }
  printf("Init done.\n");
  while(ros::ok()){
    infoMsgType=rcmInfoGet(&rangeInfo, &ndbInfo);
    //std::cout << "type is : " << infoMsgType << std::endl;
   // std::cout << "RANGEINFO is : " << RANGEINFO << std::endl;
    if(infoMsgType==RANGEINFO)
    {
      double snr = 20.0 * std::log10(rangeInfo.vPeak / rangeInfo.noise == 0 ? 0.001 : rangeInfo.noise);
      //printf("rangeInfo: id %d range %u status %d status %d snr %f time %u\n",rangeInfo.responderId,rangeInfo.precisionRangeMm, rangeInfo.reqLEDFlags, rangeInfo.respLEDFlags,snr,rangeInfo.timestamp);

      /******
      if (snr < 40 || rangeInfo.reqLEDFlags > 9 || rangeInfo.respLEDFlags > 9) {
        std::cout << "singal noise ratio of " << std::to_string(rangeInfo.responderId) << " is " << snr << std::endl;
         
        continue;
      }
      ******/
      
      //printf("rangeInfo: id %d range %u status %d status %d snr %f\n",rangeInfo.responderId,rangeInfo.precisionRangeMm, rangeInfo.reqLEDFlags, rangeInfo.respLEDFlags,snr);
      sensor_msgs::Range msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::to_string(rangeInfo.responderId);
      msg.range = rangeInfo.precisionRangeMm * 0.001;
      msg.max_range = rangeInfo.vPeak;
      msg.min_range = rangeInfo.noise;
      uwb_pub.publish(msg);
      loop_rate.sleep();
     // std::cout << "i finish pub" << std::endl;
    }
  } // while (valid)
  // Flush any pending messages
  rcmIfFlush();

//  out << YAML::EndMap; //1
//  fout << out.c_str();
//  fout.close();

  // All done. Restore original configuration
  printf("\nAll done.\n");
  rcmIfClose();
  return 0;
}

std::vector<std::string> split(std::string str, std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;
  std::string::size_type size = str.size(),i;

  for (i = 0; i < size; i++)
  {
    pos = str.find(pattern, i);
    if (pos < size)
    {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}
}

