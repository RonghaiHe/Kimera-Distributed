/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Author: Ronghai He
 * @Date: 2025-04-03 15:33:59
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-04-10 03:15:56
 * @FilePath: /src/kimera_distributed/include/kimera_distributed/DistanceProcess.h
 * @Version:
 * @Description:
 *
 */
#pragma once

#include <geometry_msgs/Pose.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_msgs/RequestGlobalPose.h>
#include <pose_graph_tools_msgs/UWBFrame.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>

#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include "kimera_distributed/SubmapAtlas.h"
#include "kimera_distributed/configs.h"
#include "kimera_distributed/utils.h"

namespace kimera_distributed {
class DistanceProcess {
 public:
  DistanceProcess(const ros::NodeHandle& n);
  ~DistanceProcess();

 private:
  DistanceProcessConfig config_;

  ros::NodeHandle nh_;

  ros::Subscriber distance_sub_;

  ros::Publisher pose_graph_dis_pub_;

  // parameters
  std::vector<std::vector<double>> t_uwb_body_;

  // ID of destination robot: queue of (timestamp, distances)
  std::vector<std::queue<std::pair<int64_t, std::vector<double>>>> team_distance_queue_;
  // ID: vector of (timestamp, global pose)
  std::vector<std::vector<std::pair<int64_t, geometry_msgs::Pose>>> team_global_poses_;
  std::vector<std::vector<uint64>> team_pose_ids_;

  // (my latest pose index, dst_robot latest pose index)
  std::vector<std::pair<size_t, size_t>> team_latest_poses_idx_;
  // latest relative pose from myself to dst_robot
  std::vector<gtsam::Pose3> team_latest_relative_poses_;

  std::mutex team_distance_queue_mutex_;

  std::unique_ptr<std::thread> process_thread_;

  void DistanceCallback(const pose_graph_tools_msgs::UWBFrameConstPtr& msg);

  /**
   * @brief: Request global poses from certain robots, add them in
   * team_global_poses_ and team_pose_ids_
   * @param robot_id
   * @return void
   */
  void requestGlobalPoses(const unsigned int robot_id);

  /*
   * @brief: Calculate relative pose between two robots using gtsam
   * @param distances: distances between two robots
   * @param relative_rotation_init: initial relative rotation
   * @param relative_translation_init: initial relative translation
   * @param relative_pose_opt: optimized relative pose
   * @return void
   */
  void distances2RelativePose(const std::vector<double>& distances,
                              gtsam::Rot3& relative_rotation_init,
                              gtsam::Point3& relative_translation_init,
                              gtsam::Pose3& relative_pose_opt);

  void runDistanceProcessLoop();
};
}  // namespace kimera_distributed