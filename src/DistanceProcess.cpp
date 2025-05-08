/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Author: Ronghai He
 * @Date: 2025-04-07 22:59:05
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-04-17 21:19:56
 * @FilePath: /src/kimera_distributed/src/DistanceProcess.cpp
 * @Version:
 * @Description:
 *
 */
#include "kimera_distributed/DistanceProcess.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_msgs/PoseGraphEdge.h>
#include <pose_graph_tools_msgs/PoseGraphNode.h>

#include <optional>
#include <string>

#include "kimera_distributed/RelativeTransformationFactor.h"

namespace kimera_distributed {
DistanceProcess::DistanceProcess(const ros::NodeHandle& n) : nh_(n) {
  int my_id_int = -1;
  int num_robots_int = -1;
  ros::param::get("~robot_id", my_id_int);
  ros::param::get("~num_robots", num_robots_int);
  ros::param::get("~frame_id", config_.frame_id_);
  if (my_id_int < 0) {
    if (num_robots_int <= 0) {
      ROS_ERROR(
          "Invalid number of robots: %d. The number of robots must be greater than 0.",
          num_robots_int);
      ros::shutdown();
      return;
    }
    throw std::runtime_error("Invalid robot ID. Initialization failed.");
  }
  if (num_robots_int <= 0) {
    ROS_ERROR("Invalid number of robots: %d. Number of robots must be positive.",
              num_robots_int);
    throw std::runtime_error("Invalid number of robots. Initialization failed.");
  }
  config_.my_id_ = my_id_int;
  config_.num_robots_ = num_robots_int;

  for (size_t id = 0; id < config_.num_robots_; id++) {
    std::string robot_name = "kimera" + std::to_string(id);
    ros::param::get("~robot" + std::to_string(id) + "_name", robot_name);
    config_.robot_names_[id] = robot_name;
  }

  team_global_poses_.resize(config_.num_robots_);
  team_submap_ids_.resize(config_.num_robots_);
  team_T_submap_kfs_.resize(config_.num_robots_);
  team_latest_poses_idx_.resize(config_.num_robots_, std::make_pair(0, 0));
  team_latest_relative_poses_.resize(config_.num_robots_);
  team_distance_queue_.resize(config_.num_robots_,
                              std::queue<std::pair<int64_t, std::vector<double>>>());

  t_uwb_body_.resize(config_.num_robots_,
                     std::vector<double>(3, 0.0));  // Correct initialization
  for (size_t id = 0; id < config_.num_robots_; id++) {
    for (size_t uid = 0; uid < 3; uid++) {
      ros::param::get(config_.robot_names_[config_.my_id_] +
                          "/kimera_vio_ros/kimera_vio_ros_node/t_body_uwb" +
                          std::to_string(id) + "_" + std::to_string(uid),
                      t_uwb_body_[id][uid]);
    }
  }

  std::string distance_topic = '/' + config_.robot_names_[config_.my_id_] +
                               "/kimera_vio_ros/kimera_vio_ros_node/dis";
  distance_sub_ =
      nh_.subscribe(distance_topic, 100, &DistanceProcess::DistanceCallback, this);

  std::string pose_graph_dis_topic =
      "/" + config_.robot_names_[config_.my_id_] +
      "/kimera_distributed/pose_graph_distances_incremental";
  pose_graph_dis_pub_ =
      nh_.advertise<pose_graph_tools_msgs::PoseGraph>(pose_graph_dis_topic, 1000, true);

  count_ok_ = 0;

  ros::param::get("~gt_file_path", config_.gt_file_path_);
  timestamp_gt_.resize(config_.num_robots_);
  pose_gt_.resize(config_.num_robots_);
  // Read GT file from outside
  if (config_.gt_file_path_.empty()) {
    ROS_WARN("No GT file path provided. Distance process will not read GT.");
  } else {
    readGTFile(config_.gt_file_path_);
  }

  process_thread_.reset(
      new std::thread(&DistanceProcess::runDistanceProcessLoop, this));
  ROS_INFO("Robot %zu started distance process thread.", config_.my_id_);
}

DistanceProcess::~DistanceProcess() {
  if (process_thread_) {
    process_thread_->join();
    process_thread_.reset();
  }
}

void DistanceProcess::readGTFile(const std::string& gt_file_path) {
  std::string gt_file_name;

  for (size_t i = 0; i < config_.num_robots_; i++) {
    gt_file_name =
        gt_file_path + "modified_" + config_.robot_names_[i] + "_gt_odom.tum";

    // Use gt_file_ to read GT file.
    gt_file_.open(gt_file_name);
    if (!gt_file_.is_open()) {
      ROS_ERROR("Failed to open GT file %s.", gt_file_name.c_str());
      return;
    }

    std::string line;
    // Format: #timestamp_kf x y z qx qy qz qw
    while (std::getline(gt_file_, line)) {
      if (line.empty() || line[0] == '#') {
        // Skip empty lines and comment lines
        continue;
      }

      std::istringstream iss(line);
      double timestamp, x, y, z, qx, qy, qz, qw;

      if (!(iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        ROS_WARN("Failed to parse ground truth line: %s", line.c_str());
        continue;
      }

      // Create a pose from the parsed ground truth data
      gtsam::Pose3 gt_pose =
          gtsam::Pose3(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(x, y, z));

      // Convert timestamp to uint64_t (nanoseconds)
      uint64_t timestamp_ns = static_cast<uint64_t>(timestamp * 1e9);

      // Initialize vectors for the first robot if they're empty
      if (timestamp_gt_.empty() || pose_gt_.empty()) {
        timestamp_gt_.resize(config_.num_robots_);
        pose_gt_.resize(config_.num_robots_);
      }

      // Store the ground truth timestamp and pose for the current robot
      timestamp_gt_[i].emplace_back(timestamp_ns);
      pose_gt_[i].emplace_back(gt_pose);
    }
    ROS_INFO("Finished loading ground truth data with %zu poses for robot %zu.",
             timestamp_gt_[i].size(),
             i);
    // Close the file before opening it for the next robot
    gt_file_.close();
  }
}

void DistanceProcess::requestGlobalPoses(const unsigned int robot_id) {
  // Request global poses from certain robots
  pose_graph_tools_msgs::RequestGlobalPose request_global_pose_self;
  request_global_pose_self.request.robot_id = robot_id;
  std::string service_name = "/" + config_.robot_names_[robot_id] +
                             "/distributed_loop_closure/request_global_pose";
  if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return;
  }
  if (!ros::service::call(service_name, request_global_pose_self)) {
    ROS_WARN_STREAM("Failed to call ROS service " << service_name);
    return;
  }
  // CHECK_EQ(request_global_pose_self.response.success, true);
  if (!request_global_pose_self.response.submap_ids.size()) {
    ROS_ERROR_STREAM("Obtaining response w/o global poses found from robot "
                     << robot_id);
    return;
  }
  ROS_INFO("Obtaining global poses from robot %u with %ld poses!",
           robot_id,
           request_global_pose_self.response.submap_ids.size());
  // CHECK_EQ(request_global_pose_self.response.submap_ids.size(),
  //  request_global_pose_self.response.path.size());

  // Initialize team_global_poses_ and team_submap_ids_ with appropriate size if they're
  // empty
  if (team_global_poses_.empty()) {
    team_global_poses_.resize(config_.num_robots_);
    team_submap_ids_.resize(config_.num_robots_);
    team_T_submap_kfs_.resize(config_.num_robots_);
  }
  team_global_poses_[robot_id].clear();
  team_submap_ids_[robot_id].clear();
  team_T_submap_kfs_[robot_id].clear();

  // Process and store the pose data for my robot
  for (size_t i = 0; i < request_global_pose_self.response.submap_ids.size(); i++) {
    uint32_t submap_id = request_global_pose_self.response.submap_ids[i];
    const geometry_msgs::Pose& T_submap_kf =
        request_global_pose_self.response.T_submap_kfs[i];
    const geometry_msgs::PoseStamped& pose_stamped =
        request_global_pose_self.response.path.poses[i];

    int64_t timestamp = pose_stamped.header.stamp.toNSec();

    // Create a Pose object from geometry_msgs::Pose
    // geometry_msgs::Pose global_pose;
    // // Convert geometry_msgs::Pose to your Pose type
    // // This depends on your Pose definition, but typically:
    // global_pose.position.x = pose_stamped.pose.position.x;
    // global_pose.position.y = pose_stamped.pose.position.y;
    // global_pose.position.z = pose_stamped.pose.position.z;
    // global_pose.orientation.w = pose_stamped.pose.orientation.w;
    // global_pose.orientation.x = pose_stamped.pose.orientation.x;
    // global_pose.orientation.y = pose_stamped.pose.orientation.y;
    // global_pose.orientation.z = pose_stamped.pose.orientation.z;

    // Add to team_global_poses_ for my robot ID
    team_global_poses_[robot_id].emplace_back(
        std::make_pair(timestamp, pose_stamped.pose));

    team_T_submap_kfs_[robot_id].emplace_back(T_submap_kf);

    // Add pose ID to team_submap_ids_ for my robot ID
    team_submap_ids_[robot_id].push_back(submap_id);
  }
}

bool DistanceProcess::distances2RelativePose(const std::vector<double>& distances,
                                             gtsam::Rot3& relative_rotation_init,
                                             gtsam::Point3& relative_translation_init,
                                             gtsam::Pose3& relative_pose_opt) {
  // ROS_INFO("In Distances2RelativePose");
  // Convert distance measurements to point pairs and actual distances for optimization
  std::vector<gtsam::Point3> points_j;        // Points in robot A's frame
  std::vector<gtsam::Point3> points_i;        // Points in robot B's frame
  std::vector<double> measurement_distances;  // Actual distance measurements d_i

  // TODO(RonghaiHe) Use CHECK
  if (distances.size() != 3 * config_.num_robots_) {
    ROS_ERROR(
        "Optimization failed: Insufficient measurements. At least 6 measurements are "
        "required, but only %ld were provided.",
        measurement_distances.size());
    return false;
  }

  // Extract anchor distance measurements from the input
  // This depends on your specific measurement format and system setup
  // Distances vector contains triplets of [dij0, dij1, dij2], **3** UWBs each robot:
  for (size_t i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (distances[i * 3 + j] < 0) {
        continue;
      }
      // Convert distances to points in the robot's frame
      points_j.push_back(
          gtsam::Point3(t_uwb_body_[j][0], t_uwb_body_[j][1], t_uwb_body_[j][2]));
      points_i.push_back(
          gtsam::Point3(t_uwb_body_[i][0], t_uwb_body_[i][1], t_uwb_body_[i][2]));
      measurement_distances.push_back(distances[i * 3 + j]);  // The measured distance

      auto temp = relative_rotation_init * points_j.back() + relative_translation_init -
                  points_i.back();
      double residual = temp.norm() - measurement_distances.back();
      ROS_INFO("Residual: %f", residual);
      // ros::Duration(2.0).sleep();
    }
  }

  // TODO (RonghaiHe) Use CHECK
  if (measurement_distances.size() < 6) {
    ROS_ERROR("Not enough measurements to optimize (Only %ld)",
              measurement_distances.size());
    return false;
  }

  // Create factor graph for optimization
  gtsam::NonlinearFactorGraph graph;

  // Create a base noise model with isotropic uncertainty
  auto base_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(measurement_distances.size(), 0.0382);
  // auto based_noise_model =
  // gtsam::noiseModel::Isotropic::Sigma(measurement_distances.size(), 0.0382);

  // Wrap the base noise model with a robust kernel (Huber)
  // The threshold parameter (1.345) determines when the robust loss function takes
  // effect
  auto noise_model = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise_model);

  // Addz the relative transformation factor with distance measurements
  gtsam::Key pose_key = gtsam::Symbol('x', 0);
  graph.add(RelativeTransformationFactor(
      pose_key, points_j, points_i, measurement_distances, noise_model));

  // Set initial estimate based on the provided initial values
  gtsam::Values initial_estimate;
  initial_estimate.insert(
      pose_key, gtsam::Pose3(relative_rotation_init, relative_translation_init));

  // Optimize using Levenberg-Marquardt
  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("TERMINATION");
  lm_params.setMaxIterations(100);
  lm_params.setRelativeErrorTol(1e-6);
  lm_params.setAbsoluteErrorTol(1e-6);

  // gtsam::GncParams<gtsam::LevenbergMarquardtParams> gnc_params(lm_params);
  // gnc_params.setMuStep(1.2);
  // gnc_params.setRelativeCostTol(1e-6);
  // gnc_params.setVerbosityGNC(
  //     gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::SUMMARY);
  // gnc_params.setLossType(gtsam::GncLossType::GM);
  // gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
  // gnc_optimizer(
  //     graph, initial_estimate, gnc_params);

  // gtsam::Values result = gnc_optimizer.optimize();

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, lm_params);
  gtsam::Values result = optimizer.optimize();

  // gtsam::Marginals marginals(graph, result,
  // gtsam::Marginals::Factorization::CHOLESKY); covariance =
  // marginals.marginalCovariance(0).fullMatrix();

  // Check if optimization was successful
  double initial_error = graph.error(initial_estimate);
  double final_error = graph.error(result);
  if (final_error > 0.1) {
    if (final_error <= 0.15) {
      ROS_WARN(
          "Optimization marginally improved but final error (%f) is still above the "
          "threshold (0.1).",
          final_error);
    } else {
      ROS_ERROR(
          "Fail to calculate the relative pose using estimated poses with final error: "
          "%f (initial error: %f).",
          final_error,
          initial_error);
    }
    // gtsam::Rot3 iden_R = gtsam::Rot3::Identity();
    gtsam::Point3 iden_t(
        measurement_distances[0] / 1.414, measurement_distances[0] / 1.414, 0);
    gtsam::Values initial_estimate2;
    initial_estimate2.insert(pose_key, gtsam::Pose3(relative_rotation_init, iden_t));
    // gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
    //     gnc_optimizer2(graph, initial_estimate2, gnc_params);
    // gtsam::Values result2 = gnc_optimizer2.optimize();
    gtsam::LevenbergMarquardtOptimizer optimizer2(graph, initial_estimate2, lm_params);
    gtsam::Values result2 = optimizer2.optimize();

    // gtsam::Marginals marginals(graph, result,
    // gtsam::Marginals::Factorization::CHOLESKY); covariance =
    // marginals.marginalCovariance(0).fullMatrix();

    // Check if optimization was successful
    double initial_error2 = graph.error(initial_estimate2);
    double final_error2 = graph.error(result2);
    bool optimization_improved2 = final_error2 < initial_error2;
    if (!optimization_improved2 || final_error2 > 0.1) {
      ROS_ERROR("Fail to calculate the relative pose using identity with %f",
                final_error);
      return false;
    } else {
      result = result2;
    }
  }
  // double error_reduction = initial_error > 0 ? (initial_error - final_error) /
  // final_error : 0;
  // if (error_reduction < 0.01) {
  //   return false;
  // }

  // Extract the optimized pose
  relative_pose_opt = result.at<gtsam::Pose3>(pose_key);

  ROS_INFO("Opt complete. init: %f vs final: %f", initial_error, final_error);
  // ROS_INFO_STREAM("Initial Relative Rotation: " << relative_rotation_init.matrix());
  // ROS_INFO_STREAM("After Relative Rotation: " <<
  // relative_pose_opt.rotation().matrix());

  // ROS_INFO_STREAM(
  //     "Initial Relative Translation: " << relative_translation_init.transpose());
  // ROS_INFO_STREAM(
  //     "After Relative Translation: " << relative_pose_opt.translation().transpose());

  return true;
}

void DistanceProcess::DistanceCallback(
    const pose_graph_tools_msgs::UWBFrameConstPtr& msg) {
  // Log incoming message data for debugging
  // ROS_INFO("Received UWB message - src_id: %zu, dst_id: %zu, distances: %zu",
  //          msg->src_robot_id,
  //          msg->dst_robot_id,
  //          msg->distances.size());

  if (msg->src_robot_id != config_.my_id_) {
    ROS_WARN("Ignoring UWB message from different source robot (expected %zu, got %d)",
             config_.my_id_,
             msg->src_robot_id);
    return;
  }

  // Validate destination robot ID
  if (msg->dst_robot_id >= config_.num_robots_) {
    ROS_ERROR("Invalid destination robot ID: %d (max allowed: %lu)",
              msg->dst_robot_id,
              config_.num_robots_ - 1);
    return;
  }
  int64_t timestamp = msg->stamp.toNSec();
  {
    std::lock_guard<std::mutex> lock(team_distance_queue_mutex_);
    // Convert Float32MultiArray to vector<double>
    std::vector<double> distances;
    distances.reserve(msg->distances.size());
    for (const auto& val : msg->distances) {
      distances.push_back(static_cast<double>(val));
    }
    team_distance_queue_[msg->dst_robot_id].emplace(
        std::make_pair(timestamp, distances));
    // ROS_INFO("Received distance from robot %zu", config_.my_id_);
  }
}

bool DistanceProcess::processSingleDistanceMeasurement(
    size_t id,
    const std::pair<int64_t, std::vector<double>>& process_meas) {
  // Find appropriate indices for poses
  size_t my_idx = team_latest_poses_idx_[id].first;
  size_t dst_idx = team_latest_poses_idx_[id].second;
  if (team_global_poses_[config_.my_id_][my_idx].first > process_meas.first) {
    my_idx = 0;
  }
  auto it_i =
      std::lower_bound(team_global_poses_[config_.my_id_].begin() + my_idx,
                       team_global_poses_[config_.my_id_].end(),
                       std::make_pair(process_meas.first, geometry_msgs::Pose()),
                       [](const auto& a, const auto& b) { return a.first < b.first; });
  team_latest_poses_idx_[id].first =
      std::distance(team_global_poses_[config_.my_id_].begin(), it_i);
  if (team_global_poses_[id][dst_idx].first > process_meas.first) {
    dst_idx = 0;
  }
  auto it_j =
      std::lower_bound(team_global_poses_[id].begin() + dst_idx,
                       team_global_poses_[id].end(),
                       std::make_pair(process_meas.first, geometry_msgs::Pose()),
                       [](const auto& a, const auto& b) { return a.first < b.first; });
  team_latest_poses_idx_[id].second =
      std::distance(team_global_poses_[id].begin(), it_j);

  // Calculate initial relative pose
  gtsam::Rot3 relative_rotation_init;
  gtsam::Point3 relative_translation_init;
  gtsam::Pose3 relative_pose_opt;

  std::optional<gtsam::Rot3> oR_i_next, oR_j_next;

  // if (my_idx == team_latest_poses_idx_[id].first &&
  //     dst_idx == team_latest_poses_idx_[id].second) {
  //   // The relative pose is already calculated
  //   relative_rotation_init = team_latest_relative_poses_[id].rotation();
  //   relative_translation_init = team_latest_relative_poses_[id].translation();
  // } else {
  my_idx = team_latest_poses_idx_[id].first;
  dst_idx = team_latest_poses_idx_[id].second;

  ROS_INFO("Meas time: %ld using robot %ld 's %ld and robot %ld 's %ld",
           process_meas.first,
           config_.my_id_,
           team_global_poses_[config_.my_id_][my_idx].first,
           id,
           team_global_poses_[id][dst_idx].first);
  ROS_INFO("Use robot %ld 's %ld (ID: %ld) and robot %ld 's %ld (ID: %ld)",
           config_.my_id_,
           team_latest_poses_idx_[id].first,
           team_submap_ids_[config_.my_id_][team_latest_poses_idx_[id].first],
           id,
           team_latest_poses_idx_[id].second,
           team_submap_ids_[id][team_latest_poses_idx_[id].second]);
  // ros::Duration(5).sleep();

  // Safety check to ensure indices are valid
  if (my_idx >= team_global_poses_[config_.my_id_].size() ||
      dst_idx >= team_global_poses_[id].size()) {
    ROS_ERROR("Invalid indices: my_idx=%zu (max=%zu), dst_idx=%zu (max=%zu)",
              my_idx,
              team_global_poses_[config_.my_id_].size() - 1,
              dst_idx,
              team_global_poses_[id].size() - 1);
    return false;  // Skip this iteration to prevent segmentation fault
  }

  // Calculate initial relative pose using output parameters
  bool is_succeed_init = calculateInitialRelativePose(process_meas.first,
                                                      my_idx,
                                                      dst_idx,
                                                      id,
                                                      relative_rotation_init,
                                                      relative_translation_init);

  if (!is_succeed_init) {
    return false;
  }

  // team_latest_relative_poses_[id] =
  // gtsam::Pose3(relative_rotation_init, relative_translation_init);

  // Create rotation objects only once and pass by reference
  const auto& my_pose = team_global_poses_[config_.my_id_][my_idx].second;
  const auto& dst_pose = team_global_poses_[id][dst_idx].second;

  // Convert quaternion to rotation for later use
  gtsam::Rot3 rotation_my_T = gtsam::Rot3::Quaternion(my_pose.orientation.w,
                                                      my_pose.orientation.x,
                                                      my_pose.orientation.y,
                                                      my_pose.orientation.z)
                                  .inverse();

  gtsam::Rot3 rotation_dst = gtsam::Rot3::Quaternion(dst_pose.orientation.w,
                                                     dst_pose.orientation.x,
                                                     dst_pose.orientation.y,
                                                     dst_pose.orientation.z);

  oR_i_next.emplace(rotation_my_T);
  oR_j_next.emplace(rotation_dst);

  // }
  // gtsam::Matrix covariance = Eigen::MatrixXd::Zero(6, 6);

  // Transform multiple relative distances into relative pose
  bool is_succceed = distances2RelativePose(process_meas.second,
                                            relative_rotation_init,
                                            relative_translation_init,
                                            relative_pose_opt);

  if (!is_succceed) {
    ROS_ERROR("Fail to calculate relative pose.");
    // gtsam::Rot3 iden_R = gtsam::Rot3::Identity();
    // gtsam::Point3 iden_t = gtsam::Point3::Zero();
    // bool is_succceed2 =
    //     distances2RelativePose(process_meas.second, iden_R, iden_t,
    //     relative_pose_opt);
    // if (!is_succceed2) {
    //   ROS_ERROR("Fail to calculate relative pose using identity.");
    //   return false;
    // }
    return false;
  }

  gtsam::Pose3 relative_pose_est =
      gtsam::Pose3(relative_rotation_init, relative_translation_init);
  gtsam::Pose3 verify_pose = relative_pose_est.inverse() * relative_pose_opt;

  ROS_INFO_STREAM("Verify_t: " << verify_pose.rotation().matrix());
  ROS_INFO_STREAM("Verify_t: " << verify_pose.translation().transpose());

  // Calculate the relative pose aligned to the pose estimation timestamp
  gtsam::Rot3 Delta_Ri_prev_r = gtsam::Rot3::Identity();
  gtsam::Rot3 Delta_Rj_r_prev = gtsam::Rot3::Identity();
  gtsam::Point3 Delta_ti_prev_r = gtsam::Point3::Zero();
  gtsam::Point3 Delta_tj_r_prev = gtsam::Point3::Zero();

  if (my_idx == 0 || dst_idx == 0) {
    ROS_ERROR("Invalid that the 1st pose is earlier than the measurement");
    return false;
  }

  int32_t use_idx_i = -1, use_idx_j = -1;
  int64_t delta_t_ri =
      process_meas.first - team_global_poses_[config_.my_id_][my_idx - 1].first;
  int64_t delta_t_rj = process_meas.first - team_global_poses_[id][dst_idx - 1].first;
  int64_t Delta_t_i = team_global_poses_[config_.my_id_][my_idx].first -
                      team_global_poses_[config_.my_id_][my_idx - 1].first;
  int64_t Delta_t_j =
      team_global_poses_[id][dst_idx].first - team_global_poses_[id][dst_idx - 1].first;

  // ROS_INFO("delta_t_ri: %ld, delta_t_rj: %ld, Delta_t_i: %ld, Delta_t_j: %ld",
  //          delta_t_ri,
  //          delta_t_rj,
  //          Delta_t_i,
  //          Delta_t_j);

  // Judge if interpolation is needed (threshold: 5ms = 5,000,000ns)
  const int64_t INTERP_THRESHOLD_NS = 5000000;

  if (delta_t_ri < INTERP_THRESHOLD_NS ||
      Delta_t_i - delta_t_ri < INTERP_THRESHOLD_NS) {
    if (delta_t_ri < INTERP_THRESHOLD_NS) {
      use_idx_i = my_idx - 1;
    } else {
      use_idx_i = my_idx;
    }
  }

  if (delta_t_rj < INTERP_THRESHOLD_NS ||
      Delta_t_j - delta_t_rj < INTERP_THRESHOLD_NS) {
    if (delta_t_rj < INTERP_THRESHOLD_NS) {
      use_idx_j = dst_idx - 1;
    } else {
      use_idx_j = dst_idx;
    }
  }

  if (use_idx_i < 0) {
    // Need interpolation for source robot using output parameters
    interpolateSourcePose(
        my_idx, delta_t_ri, Delta_t_i, oR_i_next, Delta_Ri_prev_r, Delta_ti_prev_r);
  }

  if (use_idx_j < 0) {
    // Need interpolation for destination robot
    interpolateDestPose(dst_idx,
                        id,
                        delta_t_rj,
                        Delta_t_j,
                        oR_j_next,
                        Delta_Rj_r_prev,
                        Delta_tj_r_prev);
  }

  // ROS_INFO("Use idx_i: %d, idx_j: %d", use_idx_i, use_idx_j);
  // ROS_INFO("delta_t_ri: %ld, delta_t_rj: %ld", delta_t_ri, delta_t_rj);
  // ROS_INFO("Delta_t_i: %ld, Delta_t_j: %ld", Delta_t_i, Delta_t_j);
  // ROS_INFO_STREAM("Delta_Ri_prev_r: " << Delta_Ri_prev_r.matrix());
  // ROS_INFO_STREAM("Delta_Rj_r_prev: " << Delta_Rj_r_prev.matrix());
  // ROS_INFO_STREAM("Delta_ti_prev_r: " << Delta_ti_prev_r.transpose());
  // ROS_INFO_STREAM("Delta_tj_prev_r: " << Delta_tj_r_prev.transpose());

  // Calculate final pose transformation
  gtsam::Rot3 Delta_Rij_r = relative_pose_opt.rotation();
  gtsam::Point3 Delta_tij_r = relative_pose_opt.translation();
  gtsam::Rot3 Delta_Rij = Delta_Ri_prev_r * Delta_Rij_r * Delta_Rj_r_prev;
  gtsam::Point3 Delta_tij = Delta_Ri_prev_r * Delta_Rij_r * Delta_tj_r_prev +
                            Delta_Ri_prev_r * Delta_tij_r + Delta_ti_prev_r;

  // ROS_INFO_STREAM("Before tf: " << Delta_Rij_r.matrix() << " "
  //                               << Delta_tij_r.transpose());
  // ROS_INFO_STREAM("Delta Ri" << Delta_Ri_prev_r.matrix());
  // ROS_INFO_STREAM("Delta Rj" << Delta_Rj_r_prev.matrix());
  // ROS_INFO_STREAM("Delta ti and tj" << Delta_ti_prev_r.transpose() << " "
  //                                   << Delta_tj_r_prev.transpose());
  // ROS_INFO_STREAM("delta t" << delta_t_ri << " " << delta_t_rj << " " << Delta_t_i
  //                           << " " << Delta_t_j);
  // ROS_INFO_STREAM("Final pose transformation: " << Delta_Rij.matrix() << " "
  //                                               << Delta_tij.transpose());
  // ros::Duration(10).sleep();
  // Publish the pose graph edge
  publishPoseGraphEdge(id,
                       my_idx,
                       dst_idx,
                       use_idx_i,
                       use_idx_j,
                       Delta_Rij,
                       Delta_tij,
                       relative_pose_opt,
                       process_meas.second);
  return true;
}

bool DistanceProcess::calculateInitialRelativePose(
    uint64_t ts_meas,
    size_t my_idx,
    size_t dst_idx,
    size_t id,
    gtsam::Rot3& relative_rotation_init,
    gtsam::Point3& relative_translation_init) {
  // Get references to the pose data to avoid repeated lookups
  // const auto& my_pose = team_global_poses_[config_.my_id_][my_idx].second;
  // const auto& dst_pose = team_global_poses_[id][dst_idx].second;

  // // Transform the quaternion to rotation matrix
  // gtsam::Rot3 rotation_my_T = gtsam::Rot3::Quaternion(my_pose.orientation.w,
  //                                                     my_pose.orientation.x,
  //                                                     my_pose.orientation.y,
  //                                                     my_pose.orientation.z)
  //                                 .inverse();

  // gtsam::Rot3 rotation_dst = gtsam::Rot3::Quaternion(dst_pose.orientation.w,
  //                                                    dst_pose.orientation.x,
  //                                                    dst_pose.orientation.y,
  //                                                    dst_pose.orientation.z);

  // relative_rotation_init = rotation_my_T * rotation_dst;

  // // Use direct reference to position data
  // gtsam::Point3 dst_position(
  //     dst_pose.position.x, dst_pose.position.y, dst_pose.position.z);

  // gtsam::Point3 my_position(my_pose.position.x, my_pose.position.y,
  // my_pose.position.z);

  // relative_translation_init = rotation_my_T * (dst_position - my_position);
  // return true;

  // const uint64_t ts_src = team_global_poses_[config_.my_id_][my_idx].first;
  // const uint64_t ts_dst = team_global_poses_[id][dst_idx].first;

  // uint64_t ts_src, ts_dst;
  auto it_src = std::lower_bound(timestamp_gt_[config_.my_id_].begin(),
                                 timestamp_gt_[config_.my_id_].end(),
                                 ts_meas);
  const uint64_t ts_next_src = *it_src;
  if (it_src == timestamp_gt_[config_.my_id_].begin()) {
    ROS_ERROR("Cannot find the pose of the robot %ld at %lu", config_.my_id_, ts_meas);
    return false;
  }
  size_t idx_src = std::distance(timestamp_gt_[config_.my_id_].begin(), it_src);
  const uint64_t ts_prev_src = timestamp_gt_[config_.my_id_][idx_src - 1];
  auto it_dst =
      std::lower_bound(timestamp_gt_[id].begin(), timestamp_gt_[id].end(), ts_meas);
  const uint64_t ts_next_dst = *it_dst;
  if (it_dst == timestamp_gt_[id].begin()) {
    ROS_ERROR("Cannot find the pose of the robot %ld at %lu", id, ts_meas);
    return false;
  }
  size_t idx_dst = std::distance(timestamp_gt_[id].begin(), it_dst);
  if (idx_dst == 0) {
    ROS_ERROR("Index out of bounds for robot %ld at %lu", id, ts_meas);
    return false;
  }
  const uint64_t ts_prev_dst = timestamp_gt_[id][idx_dst - 1];
  if (it_dst == timestamp_gt_[id].begin()) {
    ROS_ERROR("Cannot find the pose of the robot %ld at %lu", id, ts_next_dst);
    return false;
  }

  ROS_INFO("Use GT pose at %lu for robot %ld and %lu for robot %ld",
           ts_prev_src,
           config_.my_id_,
           ts_prev_dst,
           id);

  gtsam::Pose3 pose_src, pose_dst, pose_src_dst;

  const double ratio_src = (ts_meas - ts_prev_src) / (ts_next_src - ts_prev_src);
  interpolatePose(ratio_src,
                  pose_gt_[config_.my_id_][idx_src - 1],
                  pose_gt_[config_.my_id_][idx_src],
                  pose_src);

  const double ratio_dst = (ts_meas - ts_prev_dst) / (ts_next_dst - ts_prev_dst);
  interpolatePose(
      ratio_dst, pose_gt_[id][idx_dst - 1], pose_gt_[id][idx_dst], pose_dst);

  relative_rotation_init = pose_src.rotation().inverse() * pose_dst.rotation();
  relative_translation_init =
      pose_src.rotation().inverse() * (pose_dst.translation() - pose_src.translation());

  relative_rotation_init =
      relative_rotation_init * gtsam::Rot3::Expmap(gtsam::Vector3(0, 0, 0.01));
  relative_translation_init = relative_translation_init + gtsam::Vector3(0, 0, 0.01);
  // ROS_INFO_STREAM("Relative pose initialization: "
  //                 << relative_rotation_init.matrix() << std::endl
  //                 << relative_translation_init.transpose());
  ROS_INFO("Complete relative pose initialization.");
  return true;
}

void DistanceProcess::interpolatePose(double ratio_time,
                                      gtsam::Pose3& pose_prev,
                                      gtsam::Pose3& pose_next,
                                      gtsam::Pose3& pose_curr) {
  // Use Exponent map in gtsam to calculate the relative rotation
  gtsam::Rot3 R_curr =
      pose_prev.rotation() *
      gtsam::Rot3::Expmap(
          gtsam::Rot3::Logmap(pose_prev.rotation().inverse() * pose_next.rotation()) *
          ratio_time);

  gtsam::Point3 t_prev = pose_prev.translation();
  gtsam::Point3 t_next = pose_next.translation();
  gtsam::Point3 t_curr = t_prev + (t_next - t_prev) * ratio_time;

  // Set the interpolated pose
  pose_curr = gtsam::Pose3(R_curr, t_curr);
  // ROS_INFO_STREAM("Interpolating pose between " << pose_prev << " and " <<
  // pose_next); ROS_INFO_STREAM("Interpolated pose: " << pose_curr);
  // ros::Duration(5).sleep();
}

void DistanceProcess::interpolateSourcePose(size_t my_idx,
                                            int64_t delta_t_ri,
                                            int64_t Delta_t_i,
                                            const std::optional<gtsam::Rot3>& oR_i_next,
                                            gtsam::Rot3& Delta_Ri_prev_r,
                                            gtsam::Point3& Delta_ti_prev_r) {
  // Get references to avoid repeated lookups
  const auto& current_pose = team_global_poses_[config_.my_id_][my_idx].second;
  const auto& prev_pose = team_global_poses_[config_.my_id_][my_idx - 1].second;

  // Access position data directly by reference
  gtsam::Point3 t_i_next(
      current_pose.position.x, current_pose.position.y, current_pose.position.z);

  gtsam::Point3 t_i_prev(
      prev_pose.position.x, prev_pose.position.y, prev_pose.position.z);

  // Create quaternions once and reuse
  gtsam::Rot3 R_i_prev_T = gtsam::Rot3::Quaternion(prev_pose.orientation.w,
                                                   prev_pose.orientation.x,
                                                   prev_pose.orientation.y,
                                                   prev_pose.orientation.z)
                               .inverse();

  // Use the passed rotation if available, otherwise create it
  const gtsam::Rot3& R_i_next = gtsam::Rot3::Quaternion(current_pose.orientation.w,
                                                        current_pose.orientation.x,
                                                        current_pose.orientation.y,
                                                        current_pose.orientation.z);

  // Calculate interpolation ratio once
  if (Delta_t_i == 0) {
    ROS_ERROR("Delta_t_i is zero, cannot calculate interpolation ratio.");
    return;
  }
  double ratio = static_cast<double>(delta_t_ri) / static_cast<double>(Delta_t_i);

  // Use Exponent map in gtsam to calculate the relative rotation
  Delta_Ri_prev_r =
      gtsam::Rot3::Expmap(gtsam::Rot3::Logmap(R_i_prev_T * R_i_next) * ratio);

  ROS_INFO_STREAM("dR: " << R_i_prev_T * R_i_next);

  Delta_ti_prev_r = R_i_prev_T * (t_i_next - t_i_prev) * ratio;
}

void DistanceProcess::interpolateDestPose(size_t dst_idx,
                                          size_t id,
                                          int64_t delta_t_rj,
                                          int64_t Delta_t_j,
                                          const std::optional<gtsam::Rot3>& oR_j_next,
                                          gtsam::Rot3& Delta_Rj_r_prev,
                                          gtsam::Point3& Delta_tj_r_prev) {
  // Get references to avoid repeated lookups
  const auto& current_pose = team_global_poses_[id][dst_idx].second;
  const auto& prev_pose = team_global_poses_[id][dst_idx - 1].second;

  // Access position data directly by reference
  gtsam::Point3 t_j_next(
      current_pose.position.x, current_pose.position.y, current_pose.position.z);

  gtsam::Point3 t_j_prev(
      prev_pose.position.x, prev_pose.position.y, prev_pose.position.z);

  // Create quaternions once and reuse
  gtsam::Rot3 R_j_prev_T = gtsam::Rot3::Quaternion(prev_pose.orientation.w,
                                                   prev_pose.orientation.x,
                                                   prev_pose.orientation.y,
                                                   prev_pose.orientation.z)
                               .inverse();

  // Use the passed rotation if available, otherwise create it
  const gtsam::Rot3& R_j_next = gtsam::Rot3::Quaternion(current_pose.orientation.w,
                                                        current_pose.orientation.x,
                                                        current_pose.orientation.y,
                                                        current_pose.orientation.z);

  // Calculate interpolation ratio onDelta_Ri_prev_rce
  if (Delta_t_j == 0) {
    ROS_ERROR("Delta_t_j is zero, cannot calculate interpolation ratio.");
    return;
  }
  double ratio = static_cast<double>(delta_t_rj) / static_cast<double>(Delta_t_j);

  Delta_Rj_r_prev =
      gtsam::Rot3::Expmap(-gtsam::Rot3::Logmap(R_j_prev_T * R_j_next) * ratio);

  Delta_tj_r_prev = Delta_Rj_r_prev * R_j_prev_T * (t_j_prev - t_j_next) * ratio;
}

void DistanceProcess::publishPoseGraphEdge(size_t id,
                                           size_t my_idx,
                                           size_t dst_idx,
                                           int32_t use_idx_i,
                                           int32_t use_idx_j,
                                           const gtsam::Rot3& Delta_Rij,
                                           const gtsam::Point3& Delta_tij,
                                           const gtsam::Pose3& Delta_Tij_r,
                                           const std::vector<double>& distances) {
  pose_graph_tools_msgs::PoseGraph pose_graph_dis;
  pose_graph_tools_msgs::PoseGraphEdge edge_dis;

  edge_dis.robot_from = config_.my_id_;
  edge_dis.robot_to = id;

  // use_idx_i=-1 -> my_idx-1; use_idx_i=my_idx-1 -> my_idx-1
  size_t fin_idx_i = my_idx - ((use_idx_i < (int)my_idx) ? 1 : 0);
  size_t fin_idx_j = dst_idx - ((use_idx_j < (int)dst_idx) ? 1 : 0);
  edge_dis.key_from = team_submap_ids_[config_.my_id_][fin_idx_i];
  edge_dis.key_to = team_submap_ids_[id][fin_idx_j];

  edge_dis.type = pose_graph_tools_msgs::PoseGraphEdge::UWB;
  const gtsam::Pose3 T_ij = gtsam::Pose3(Delta_Rij, Delta_tij);
  const auto T_si_i = RosPoseToGtsam(team_T_submap_kfs_[config_.my_id_][fin_idx_i]);
  const auto T_si_j = RosPoseToGtsam(team_T_submap_kfs_[id][fin_idx_j]);
  const auto T_si_sj = T_si_i * T_ij * (T_si_j.inverse());

  edge_dis.pose = GtsamPoseToRos(T_si_sj);  // Convert to ROS Pose message
  edge_dis.number_edge_dis = count_ok_;

  ros::Time time_from_nsec(team_global_poses_[config_.my_id_][fin_idx_i].first / 1e9);
  edge_dis.stamp_from = time_from_nsec;
  ros::Time time_to_nsec(team_global_poses_[id][fin_idx_j].first / 1e9);
  edge_dis.stamp_to = time_to_nsec;
  edge_dis.pose_kf = GtsamPoseToRos(T_ij);
  edge_dis.pose_dis = GtsamPoseToRos(Delta_Tij_r);
  edge_dis.distances.assign(9, -1.0f);
  for (int i = 0; i < 9; i++) {
    edge_dis.distances[i] = distances[i];
  }

  pose_graph_dis.header.stamp = ros::Time::now();
  pose_graph_dis.edges.push_back(edge_dis);
  pose_graph_dis_pub_.publish(pose_graph_dis);

  ROS_INFO("Publish distance between %zu and %zu", config_.my_id_, id);
}

void DistanceProcess::runDistanceProcessLoop() {
  std::vector<std::queue<std::pair<int64_t, std::vector<double>>>>
      team_distance_queue_process(
          config_.num_robots_, std::queue<std::pair<int64_t, std::vector<double>>>());
  // uint count_fail = 0;

  while (ros::ok()) {
    // Check if there are measurements to process
    bool all_meas_empty = true;
    for (size_t i = 0; i < config_.num_robots_; i++) {
      if (i != config_.my_id_ && (!team_distance_queue_[i].empty() ||
                                  !team_distance_queue_process[i].empty())) {
        all_meas_empty = false;
        break;
      }
    }

    if (all_meas_empty) {
      ROS_INFO("All distance queues are empty, sleeping for 5 seconds...");
      ros::Duration(5.0).sleep();
      continue;
    }

    // Check if we have our own global poses
    if (team_global_poses_[config_.my_id_].empty()) {
      ROS_INFO("Without my global poses, requesting at first");
      requestGlobalPoses(config_.my_id_);
      if (team_global_poses_[config_.my_id_].empty()) {
        ROS_INFO("Still w/o my global poses, sleeping for 5 seconds...");
        ros::Duration(5.0).sleep();
        continue;
      }
    }

    // Transfer measurements from main queue to processing queue
    {
      std::lock_guard<std::mutex> lock(team_distance_queue_mutex_);
      for (size_t i = 0; i < config_.num_robots_; i++) {
        if (i != config_.my_id_) {
          while (!team_distance_queue_[i].empty()) {
            team_distance_queue_process[i].push(team_distance_queue_[i].front());
            team_distance_queue_[i].pop();
          }
          team_distance_queue_[i] =
              std::queue<std::pair<int64_t, std::vector<double>>>();
        }
      }
    }

    // ROS_INFO("Start Processing ...");

    // Process measurements for each robot
    for (size_t id = 0; id < config_.num_robots_; id++) {
      // ROS_INFO("Processing Robot %ld", id);
      if (id == config_.my_id_) {
        // ROS_INFO("Skip Processing Robot %ld, myself", id);
        continue;
      }

      auto& distance_queue = team_distance_queue_process[id];
      if (distance_queue.empty()) {
        ROS_INFO("No distance data from robot %lu", id);
        continue;
      }

      // Request global poses if needed
      bool is_requested = false;
      if (team_global_poses_[id].empty()) {
        requestGlobalPoses(id);
        is_requested = true;
        if (team_global_poses_[id].empty()) {
          ROS_INFO("Still no global pose data from robot %lu", id);
          continue;
        }
      }
      uint dis_size = distance_queue.size();
      uint processed_times = 0;
      const uint MAX_PROCESSED_TIMES = 1;
      // Process each measurement in the queue
      while (!distance_queue.empty()) {
        // ROS_INFO("Processing distance data from robot %lu", id);
        auto process_meas = distance_queue.front();

        if (!dis_size) {
          if (processed_times > MAX_PROCESSED_TIMES) {
            // Drop it for having processed for 2 times
            ROS_ERROR(
                "Neglect distance data about robot %lu with %lu measurements, waiting "
                "for the next time",
                id,
                distance_queue.size());
            team_latest_poses_idx_[id] = std::make_pair(0, 0);
            break;
          }
          // It means that this distance was processed but fail to use it
          // request the global pose again to update the relative pose
          requestGlobalPoses(config_.my_id_);
          requestGlobalPoses(id);
          is_requested = true;

          // Update the related information
          ++processed_times;
          dis_size = distance_queue.size();
          team_latest_poses_idx_[id] = std::make_pair(0, 0);
        }

        // Skip measurements that are too early
        if (process_meas.first < team_global_poses_[id][0].first ||
            process_meas.first < team_global_poses_[config_.my_id_][0].first) {
          ROS_WARN("Too early, then pop out");
          distance_queue.pop();
          continue;
        }

        // Request more recent poses if needed
        if (process_meas.first > team_global_poses_[id].back().first ||
            process_meas.first > team_global_poses_[config_.my_id_].back().first) {
          if (!is_requested) {
            if (process_meas.first > team_global_poses_[id].back().first) {
              requestGlobalPoses(id);
            }
            if (process_meas.first > team_global_poses_[config_.my_id_].back().first) {
              requestGlobalPoses(config_.my_id_);
            }
            if (process_meas.first > team_global_poses_[id].back().first ||
                process_meas.first > team_global_poses_[config_.my_id_].back().first) {
              ROS_WARN(
                  "Have requested, still don't match the time for %ld and %ld vs %ld",
                  team_global_poses_[id].back().first,
                  team_global_poses_[config_.my_id_].back().first,
                  process_meas.first);
              break;
            }
          } else {
            ROS_WARN("Have requested global poses but still don't match the time");
            break;
          }
        }

        ROS_INFO("Have requested global poses and match the time");

        // Process the measurement
        bool success = processSingleDistanceMeasurement(id, process_meas);
        if (!success) {
          // ROS_ERROR("Cannot use this measurement. Keep it for the next time");
          distance_queue.push(process_meas);
        } else {
          ++count_ok_;
        }
        --dis_size;
        distance_queue.pop();
      }
    }

    ros::Duration(5).sleep();
  }
}

}  // namespace kimera_distributed