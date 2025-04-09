/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Author: Ronghai He
 * @Date: 2025-04-09 21:41:58
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-04-09 21:41:58
 * @FilePath: /src/kimera_distributed/src/DistanceProcessNode.cpp
 * @Version:
 * @Description:
 *
 */

#include <kimera_distributed/DistanceProcess.h>
#include <ros/ros.h>

using namespace kimera_distributed;

int main(int argc, char** argv) {
  ros::init(argc, argv, "distance_process_node");
  ros::NodeHandle nh;

  DistanceProcess dp(nh);

  ros::spin();

  return 0;
}
