/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Author: Ronghai He
 * @Date: 2025-04-07 22:59:05
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-04-09 14:46:28
 * @FilePath: /src/kimera_distributed/src/DistanceProcess.cpp
 * @Version:
 * @Description:
 *
 */
#include "kimera_distributed/DistanceProcess.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_msgs/PoseGraphEdge.h>
#include <pose_graph_tools_msgs/PoseGraphNode.h>

#include <string>

#include "kimera_distributed/RelativeTransformationFactor.h"

namespace kimera_distributed {
DistanceProcess::DistanceProcess(const ros::NodeHandle& n) : nh_(n) {
  DistanceProcessConfig config;
  int my_id_int = -1;
  int num_robots_int = -1;
  ros::param::get("~robot_id", my_id_int);
  ros::param::get("~num_robots", num_robots_int);
  ros::param::get("~frame_id", config.frame_id_);
  assert(my_id_int >= 0);
  assert(num_robots_int > 0);
  config.my_id_ = my_id_int;
  config.num_robots_ = num_robots_int;

  for (size_t id = 0; id < config.num_robots_; id++) {
    std::string robot_name = "kimera" + std::to_string(id);
    ros::param::get("~robot" + std::to_string(id) + "_name", robot_name);
    config.robot_names_[id] = robot_name;
  }

  team_global_poses_.resize(config.num_robots_);
  team_pose_ids_.resize(config.num_robots_);
  team_latest_poses_idx_.resize(config.num_robots_, std::make_pair(0, 0));
  t_uwb_body_.resize(
      3, std::vector<double>(3, 0.0));  // Initialize t_uwb_body_ with appropriate size
  for (size_t id = 0; id < config.num_robots_; id++) {
    ros::param::get(config.robot_names_[config.my_id_] + "/kimera_vio_ros/t_body_uwb" +
                        std::to_string(id),
                    t_uwb_body_[id]);
  }

  std::string distance_topic = '/' + config.robot_names_[config.my_id_] + "/distance";
  distance_sub_ =
      nh_.subscribe(distance_topic, 100, &DistanceProcess::DistanceCallback, this);

  std::string pose_graph_dis_topic =
      "/" + config_.robot_names_[config_.my_id_] +
      "/kimera_distributed/pose_graph_distances_incremental";
  pose_graph_dis_pub_ =
      nh_.advertise<pose_graph_tools_msgs::PoseGraph>(pose_graph_dis_topic, 1000, true);
}

void DistanceProcess::DistanceCallback(const kimera_vio_ros::DisConstPtr& msg) {
  if (msg->src_robot_id != config.my_id_) {
    return;
  }
  int64_t timestamp = msg->stamp.toNSec();
  {
    lock_guard<mutex> lock(team_distance_queue_mutex_);
    team_distance_queue_[msg->dst_robot_id].insert(
        std::make_pair(timestamp, msg->distances));
  }
}

void DistanceProcess::requestGlobalPoses(const uint32 robot_id) {
  // Request global poses from certain robots
  RequestGlobalPose request_global_pose_self;
  request_global_pose_self.robot_id = robot_id;
  std::string service_name = "/" + config.robot_names_[robot_id] +
                             "/multi_relative_distances/request_global_pose";
  if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return;
  }
  if (!ros::service::call(service_name, request_global_pose_self)) {
    ROS_ERROR_STREAM("Failed to call ROS service " << service_name);
    return;
  }
  CHECK_EQ(request_global_pose_self.response.success, true);
  if (!request_global_pose_self.response.pose_ids.size()) {
    return;
  }
  CHECK_EQ(request_global_pose_self.response.pose_ids.size(),
           request_global_pose_self.response.path.size());
  // Initialize team_global_poses_ and team_pose_ids_ with appropriate size if they're
  // empty
  if (team_global_poses_.empty()) {
    team_global_poses_.resize(config.num_robots_);
    team_pose_ids_.resize(config.num_robots_);
  }
  team_global_poses_[robot_id].clear();
  team_pose_ids_[robot_id].clear();

  // Process and store the pose data for my robot
  for (size_t i = 0; i < request_global_pose_self.response.pose_ids.size(); i++) {
    uint32_t pose_id = request_global_pose_self.response.pose_ids[i];
    const geometry_msgs::PoseStamped& pose_stamped =
        request_global_pose_self.response.path.poses[i];

    int64_t timestamp = pose_stamped.header.stamp.toNSec();

    // Create a Pose object from geometry_msgs::Pose
    Pose global_pose;
    // Convert geometry_msgs::Pose to your Pose type
    // This depends on your Pose definition, but typically:
    global_pose.position.x = pose_stamped.pose.position.x;
    global_pose.position.y = pose_stamped.pose.position.y;
    global_pose.position.z = pose_stamped.pose.position.z;
    global_pose.orientation.w = pose_stamped.pose.orientation.w;
    global_pose.orientation.x = pose_stamped.pose.orientation.x;
    global_pose.orientation.y = pose_stamped.pose.orientation.y;
    global_pose.orientation.z = pose_stamped.pose.orientation.z;

    // Add to team_global_poses_ for my robot ID
    team_global_poses_[config.my_id_].emplace_back(timestamp, global_pose);

    // Add pose ID to team_pose_ids_ for my robot ID
    team_pose_ids_[config.my_id_].push_back(pose_id);
  }
}

void DistanceProcess::distances2RelativePose(const std::vector<double>& distances,
                                             gtsam::Rot3& relative_rotation_init,
                                             gtsam::Point3& relative_translation_init,
                                             gtsam::Pose3& relative_pose_opt) {
  // Convert distance measurements to point pairs and actual distances for optimization
  std::vector<gtsam::Point3> points_j;        // Points in robot A's frame
  std::vector<gtsam::Point3> points_i;        // Points in robot B's frame
  std::vector<double> measurement_distances;  // Actual distance measurements d_i

  // Extract anchor distance measurements from the input
  // This depends on your specific measurement format and system setup
  // Distances vector contains triplets of [dij0, dij1, dij2], **3** UWBs each robot:
  for (size_t i = 0; i < distances.size() / 3; i++) {
    for (int j = 0; j < 3; j++) {
      // Convert distances to points in the robot's frame
      points_j.push_back(
          gtsam::Point3(t_uwb_body_[j][0], t_uwb_body_[j][1], t_uwb_body_[j][2]));
      points_i.push_back(
          gtsam::Point3(t_uwb_body_[i][0], t_uwb_body_[i][1], t_uwb_body_[i][2]));
      measurement_distances.push_back(distances[i * 3 + j]);  // The measured distance
    }
  }

  // Create factor graph for optimization
  gtsam::NonlinearFactorGraph graph;

  // Create a noise model for the measurements
  // Adjust sigma according to your measurement uncertainty
  auto noise_model = gtsam::noiseModel::Isotropic::Sigma(points_j.size(), 0.0382);

  // Add the relative transformation factor with distance measurements
  gtsam::Key pose_key = gtsam::Symbol('x', 0);
  graph.add(RelativeTransformationFactor(
      pose_key, points_j, points_i, measurement_distances, noise_model));

  // Set initial estimate based on the provided initial values
  gtsam::Values initial_estimate;
  initial_estimate.insert(
      pose_key, gtsam::Pose3(relative_rotation_init, relative_translation_init));

  // Optimize using Levenberg-Marquardt
  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setMaxIterations(100);
  lm_params.setRelativeErrorTol(1e-5);
  lm_params.setAbsoluteErrorTol(1e-5);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, lm_params);
  gtsam::Values result = optimizer.optimize();

  // Extract the optimized pose
  relative_pose_opt = result.at<gtsam::Pose3>(pose_key);

  ROS_INFO("DistanceProcess: Optimization complete.");
  ROS_INFO_STREAM("Initial Relative Rotation: " << relative_rotation_init);
  ROS_INFO_STREAM("Initial Relative Translation: " << relative_translation_init);
  ROS_INFO_STREAM("Initial Relative Translation: " << relative_translation);
  ROS_INFO_STREAM(
      "Optimized Relative Rotation: " << relative_pose_opt.rotation().matrix);
  ROS_INFO_STREAM(
      "Optimized Relative Translation: " << relative_pose_opt.translation().vector);
}

void DistanceProcess::DistanceProcessLoop() {
  // No relative distence measurements
  if (team_distance_queue_.empty()) {
    return;
  }
  // No global poses
  if (team_global_poses_.empty()) {
    // request my own global pose
    requestGlobalPoses(config.my_id_);
  }
  lock_guard<mutex> lock(team_distance_queue_mutex_);
  for (auto& [id, distance_queue] : team_distance_queue_) {
    if (distance_queue.empty()) {
      continue;
    }
    if (team_global_poses_[id].empty()) {
      requestGlobalPoses(id);
      is_requested = true;
      if (team_global_poses_[id].empty()) {
        continue;
      }
    }
    while (!distance_queue.empty()) {
      auto process_meas = distance_queue.front();
      if (process_meas.first < team_global_poses_[id][0]->first ||
          process_meas.first < team_global_poses_[config.my_id_][0]->first) {
        // Too early, then pop out
        distance_queue.pop();
        continue;
      }
      // timestamp of relative measurement is larger than the latest global pose
      if (process_meas.first > team_global_poses_[id].back()->first ||
          process_meas.first > team_global_poses_[config.my_id_].back()->first) {
        if (!is_requested) {
          if (process_meas.first > team_global_poses_[id].back()->first) {
            requestGlobalPoses(id);
          }
          if (process_meas.first > team_global_poses_[config.my_id_].back()->first) {
            requestGlobalPoses(config.my_id_);
          }
          if (process_meas.first > team_global_poses_[id].back()->first ||
              process_meas.first > team_global_poses_[config.my_id_].back()->first) {
            break;
          }
        } else {
          break;
        }
      }
      // Process the distance measurement
      size_t my_idx = team_latest_poses_idx_[id].first;
      size_t dst_idx = team_latest_poses_idx_[id].second;
      if (team_global_poses_[config.my_id_][team_latest_poses_idx_[id].first]->first <
          process_meas.first) {
        auto it_i = std::lower_bound(
            team_global_poses_[config.my_id_].begin() +
                team_latest_poses_idx_[id].first,
            team_global_poses_[config.my_id_].end(),
            std::make_pair(process_meas.first, geometry_msgs::Pose()),
            [](const auto& a, const auto& b) { return a.first < b.first; });
        team_latest_poses_idx_[id].first =
            std::distance(team_global_poses_[config.my_id_].begin(), it_i);
      }
      if (team_global_poses_[id][team_latest_poses_idx_[id].second]->first <
          process_meas.first) {
        auto it_j = std::lower_bound(
            team_global_poses_[id].begin() + team_latest_poses_idx_[id].second,
            team_global_poses_[id].end(),
            std::make_pair(process_meas.first, geometry_msgs::Pose()),
            [](const auto& a, const auto& b) { return a.first < b; });
        team_latest_poses_idx_[id].second =
            std::distance(team_global_poses_[id].begin(), it_j);
      }

      // 2nd Calculate the relative pose
      gtsam::Rot3 relative_rotation_init;
      gtsam::Point3 relative_translation_init;
      gtsam::Pose3 relative_pose_opt;

      std::optional<gtsam::Rot3> oR_i_next, oR_j_next;

      if (my_idx == team_latest_poses_idx_[id].first &&
          dst_idx == team_latest_poses_idx_[id].second) {
        // The relative pose is already calculated
        relative_rotation_init = team_latest_relative_poses_[id].rotation();
        relative_translation_init = team_latest_relative_poses_[id].translation();
      } else {
        // Transform the quaternion to rotation matrix
        gtsam::Rot3 rotation_my_T =
            gtsam::Rot3::Quaternion(
                team_global_poses_[config.my_id_][my_idx].second.orientation.w,
                team_global_poses_[config.my_id_][my_idx].second.orientation.x,
                team_global_poses_[config.my_id_][my_idx].second.orientation.y,
                team_global_poses_[config.my_id_][my_idx].second.orientation.z)
                .transpose();
        gtsam::Rot3 rotation_dst gtsam::Rot3::Quaternion(
            team_global_poses_[id][dst_idx].second.orientation.w,
            team_global_poses_[id][dst_idx].second.orientation.x,
            team_global_poses_[id][dst_idx].second.orientation.y,
            team_global_poses_[id][dst_idx].second.orientation.z);

        relative_rotation_init = rotation_my_T * rotation_dst;
        relative_translation_init =
            rotation_my_T *
            (gtsam::Point3(team_global_poses_[id][dst_idx].position.x,
                           team_global_poses_[id][dst_idx].position.y,
                           team_global_poses_[id][dst_idx].position.z) -
             gtsam::Point3(team_global_poses_[config.my_id_][my_idx].position.x,
                           team_global_poses_[config.my_id_][my_idx].position.y,
                           team_global_poses_[config.my_id_][my_idx].position.z));
        team_latest_relative_poses_[id] =
            gtsam::Pose3(relative_rotation_init, relative_translation_init);
        oR_i_next.emplace(gtsam::Rot3(rotation_my_T.transpose()));
        oR_j_next.emplace(gtsam::Rot3(rotation_dst));
      }

      // 3rd Transform multiple relative distances into relative pose
      distances2RelativePose(process_meas.second,
                             relative_rotation_init,
                             relative_translation_init,
                             relative_pose_opt);
      // 4th: Calculate the relative pose aligned to the pose estimation timestamp based
      // on the linear interpolation
      gtsam::Rot3 Delta_Ri_prev_r = gtsam::Rot3::identity();
      gtsam::Rot3 Delta_Rj_prev_r = gtsam::Rot3::identity();
      gtsam::Point Delta_ti_prev_r = gtsam::Point3::Zero();
      gtsam::Point Delta_tj_prev_r = gtsam::Point3::Zero();

      int32 use_idx_i = -1, use_idx_j = -1;
      int64_t delta_t_ri =
          process_meas.first - team_global_poses_[config.my_id_][my_idx - 1].first;
      int64_t delta_t_rj =
          process_meas.first - team_global_poses_[id][dst_idx - 1].first;
      int64_t Delta_t_i = team_global_poses_[config.my_id_][my_idx].first -
                          team_global_poses_[id][my_idx - 1].first;
      int64_t Delta_t_j = team_global_poses_[id][dst_idx].first -
                          team_global_poses_[id][dst_idx - 1].first;

      // Judge if interpolation is needed
      // TODO (RonghaiHe) Use parameter rather than 0.005[s] 1 ns = 1e-9 s
      if (delta_t_ri < 2000000 || Delta_t_i - delta_t_ri < 2000000) {
        if (delta_t_ri < 2000000) {
          use_idx_i = my_idx - 1;
        } else {
          use_idx_i = my_idx;
        }
      }
      if (delta_t_rj < 2000000 || Delta_t_j - delta_t_rj < 2000000) {
        if (delta_t_rj < 2000000) {
          use_idx_j = dst_idx - 1;
        } else {
          use_idx_j = dst_idx;
        }
      }
      if (use_idx_i < 0) {
        // Need interpolation for i (my robot)
        // Calculate the relative pose between the previous two poses

        gtsam::Rot3 R_i_prev, R_j_prev;
        gtsam::Point3 t_i_next, t_i_prev, t_j_next, t_j_prev;
        t_i_next =
            gtsam::Point3(team_global_poses_[config.my_id_][my_idx].second.position.x,
                          team_global_poses_[config.my_id_][my_idx].second.position.y,
                          team_global_poses_[config.my_id_][my_idx].second.position.z);
        t_i_prev = gtsam::Point3(
            team_global_poses_[config.my_id_][my_idx - 1].second.position.x,
            team_global_poses_[config.my_id_][my_idx - 1].second.position.y,
            team_global_poses_[config.my_id_][my_idx - 1].second.position.z);
        R_i_prev_T = gtsam::Rot3::Quaternion(
            team_global_poses_[config.my_id_][my_idx - 1].second.orientation.w,
            team_global_poses_[config.my_id_][my_idx - 1].second.orientation.x,
            team_global_poses_[config.my_id_][my_idx - 1].second.orientation.y,
            team_global_poses_[config.my_id_][my_idx - 1].second.orientation.z);
        if (!oR_i_next.has_value()) {
          oR_i_next.emplace(gtsam::Rot3::Quaternion(
              team_global_poses_[config.my_id_][my_idx].second.orientation.w,
              team_global_poses_[config.my_id_][my_idx].second.orientation.x,
              team_global_poses_[config.my_id_][my_idx].second.orientation.y,
              team_global_poses_[config.my_id_][my_idx].second.orientation.z));
        }
        // Use Exponent map in gtsam to calculate the relative rotation
        Delta_Ri_prev_r = gtsam::Expmap(gtsam::Logmap(R_i_prev_T * oR_i_curr.value()) *
                                        (double)delta_t_ri / (double)Delta_t_i);
        Delta_ti_prev_r =
            R_i_prev_T * (t_i_next - t_i_prev) * (double)delta_t_ri / (double)Delta_t_i;
      }

      if (use_idx_j < 0) {
        // Need interpolation for j (dst robot)
        t_j_next = gtsam::Point3(team_global_poses_[id][dst_idx].second.position.x,
                                 team_global_poses_[id][dst_idx].second.position.y,
                                 team_global_poses_[id][dst_idx].second.position.z);

        t_j_prev = gtsam::Point3(team_global_poses_[id][dst_idx - 1].second.position.x,
                                 team_global_poses_[id][dst_idx - 1].second.position.y,
                                 team_global_poses_[id][dst_idx - 1].second.position.z);

        R_j_prev_T = gtsam::Rot3::Quaternion(
            team_global_poses_[id][dst_idx - 1].second.orientation.w,
            team_global_poses_[id][dst_idx - 1].second.orientation.x,
            team_global_poses_[id][dst_idx - 1].second.orientation.y,
            team_global_poses_[id][dst_idx - 1].second.orientation.z);

        if (!oR_j_next.has_value()) {
          oR_j_next.emplace(gtsam::Rot3::Quaternion(
              team_global_poses_[config.my_id_][j_idx].second.orientation.w,
              team_global_poses_[config.my_id_][j_idx].second.orientation.x,
              team_global_poses_[config.my_id_][j_idx].second.orientation.y,
              team_global_poses_[config.my_id_][j_idx].second.orientation.z));
        }
        Delta_Rj_r_prev = gtsam::Expmap(-gtsam::Logmap(R_j_prev_T * oR_j_curr.value()) *
                                        (double)delta_t_rj / (double)Delta_t_j);
        Delta_tj_prev_r = -Delta_Rj_r_prev * R_j_prev_T * (t_j_next - t_j_prev) *
                          (double)delta_t_rj / (double)Delta_t_j;
      }

      // whether iterpolation or not, use this formula to calculate
      gtsam::Rot3 Delta_Rij_r = relative_pose_opt.rotation();
      gtsam::Point3 Delta_tij_r = relative_pose_opt.translation();
      gtsam::Rot3 Delta_Rij = Delta_Ri_prev_r * Delta_Rij_r * Delta_Rj_r_prev;
      gtsam::Point3 Delta_tij = Delta_Ri_prev_r * Delta_Rij_r * Delta_tj_prev_r +
                                Delta_Ri_prev_r * Delta_tij_r + Delta_ti_prev_r;

      // 4th: publish the incremental pose graph about relative distances
      pose_graph_tools_msgs::PoseGraph pose_graph_dis;
      pose_graph_tools_msgs::PoseGraphEdge edge_dis;
      // pose_graph_tools_msgs::PoseGraphNode node_dis;  // no use now

      edge_dis.robot_from = config.my_id_;
      edge_dis.robot_to = id;
      // use_idx_i=-1 -> my_idx-1; use_idx_i=my_idx-1 -> my_idx-1
      edge_dis.key_from =
          team_pose_ids_[config.my_id_][my_idx - (use_idx_i < my_idx) ? 1 : 0];
      edge_dis.key_to = team_pose_ids_[id][dst_idx - (use_idx_j < dst_idx) ? 1 : 0];
      // TODO Use unique type like RELATIVEDISTANCE
      edge_dis.type = pose_graph_tools_msgs::PoseGraphEdge::LOOPCLOSE;
      edge_dis.pose = GtsamPoseToRos(
          gtsam::Pose3(Delta_Rij, Delta_tij));  // Convert to ROS Pose message
      pose_graph_dis.header.stamp = ros::Time::now();
      pose_graph_dis.edges.push_back(edge_dis);
      pose_graph_dis_pub_.publish(pose_graph_dis);

      distance_queue.pop();
    }

    // Process the distance measurement
  }
}

}  // namespace kimera_distributed