/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Author: Ronghai He
 * @Date: 2025-04-03 15:33:59
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-04-15 15:36:25
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
  std::vector<std::vector<uint64>> team_submap_ids_;
  std::vector<std::vector<geometry_msgs::Pose>> team_T_submap_kfs_;

  // (my latest pose index, dst_robot latest pose index)
  std::vector<std::pair<size_t, size_t>> team_latest_poses_idx_;
  // latest relative pose from myself to dst_robot
  std::vector<gtsam::Pose3> team_latest_relative_poses_;

  std::mutex team_distance_queue_mutex_;

  std::unique_ptr<std::thread> process_thread_;

  void DistanceCallback(const pose_graph_tools_msgs::UWBFrameConstPtr& msg);

  /**
   * @brief: Request global poses from certain robots, add them in
   * team_global_poses_ and team_submap_ids_
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
   * @param covariance: uncertainty of relative pose (TODO RonghaiHe)
   * @return true or false for whether success
   */
  bool distances2RelativePose(const std::vector<double>& distances,
                              gtsam::Rot3& relative_rotation_init,
                              gtsam::Point3& relative_translation_init,
                              gtsam::Pose3& relative_pose_opt);

  /**
   * @brief Processes a single distance measurement and computes relative pose
   * @param id Target robot ID
   * @param process_meas Measurement data (timestamp and distances)
   * @return Whether processing was successful
   */
  bool processSingleDistanceMeasurement(
      size_t id,
      const std::pair<int64_t, std::vector<double>>& process_meas);

  /**
   * @brief Calculate initial relative pose between robots
   * @param my_idx Index of this robot's pose
   * @param dst_idx Index of destination robot's pose
   * @param id Destination robot ID
   * @param[out] relative_rotation_init Output relative rotation
   * @param[out] relative_translation_init Output relative translation
   */
  void calculateInitialRelativePose(size_t my_idx,
                                    size_t dst_idx,
                                    size_t id,
                                    gtsam::Rot3& relative_rotation_init,
                                    gtsam::Point3& relative_translation_init);

  /**
   * @brief Perform pose interpolation for source robot i->r
   * @param my_idx Index of this robot's pose
   * @param delta_t_ri Time difference
   * @param Delta_t_i Total time span
   * @param oR_i_next Optional rotation
   * @param[out] Delta_Ri_prev_r Output interpolated rotation
   * @param[out] Delta_ti_prev_r Output interpolated translation
   */
  void interpolateSourcePose(size_t my_idx,
                             int64_t delta_t_ri,
                             int64_t Delta_t_i,
                             const std::optional<gtsam::Rot3>& oR_i_next,
                             gtsam::Rot3& Delta_Ri_prev_r,
                             gtsam::Point3& Delta_ti_prev_r);

  /**
   * @brief Perform pose interpolation for destination robot r->j
   * @param dst_idx Index of destination robot's pose
   * @param id Destination robot ID
   * @param delta_t_rj Time difference
   * @param Delta_t_j Total time span
   * @param oR_j_next Optional rotation
   * @param[out] Delta_Rj_r_prev Output interpolated rotation
   * @param[out] Delta_tj_r_prev Output interpolated translation
   */
  void interpolateDestPose(size_t dst_idx,
                           size_t id,
                           int64_t delta_t_rj,
                           int64_t Delta_t_j,
                           const std::optional<gtsam::Rot3>& oR_j_next,
                           gtsam::Rot3& Delta_Rj_r_prev,
                           gtsam::Point3& Delta_tj_r_prev);

  /**
   * @brief Publish optimized relative pose as a pose graph edge
   * @param id Destination robot ID
   * @param my_idx Source robot pose index
   * @param dst_idx Destination robot pose index
   * @param use_idx_i Used source index
   * @param use_idx_j Used destination index
   * @param Delta_Rij Final rotation
   * @param Delta_tij Final translation
   */
  void publishPoseGraphEdge(size_t id,
                            size_t my_idx,
                            size_t dst_idx,
                            int32_t use_idx_i,
                            int32_t use_idx_j,
                            const gtsam::Rot3& Delta_Rij,
                            const gtsam::Point3& Delta_tij,
                            const gtsam::Pose3& Delta_Tij_r,
                            const std::vector<double>& distances);

  void runDistanceProcessLoop();
};
}  // namespace kimera_distributed