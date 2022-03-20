/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#ifndef KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_SUBMAP_H_
#define KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_SUBMAP_H_

#include "kimera_distributed/Keyframe.h"
#include <gtsam/geometry/Pose3.h>
#include <unordered_map>
#include <unordered_set>
#include <memory>

namespace kimera_distributed {

class Submap {
 public:
  /**
   * @brief Constructor
   * @param id
   */
  Submap(int id);
  /**
   * @brief Get ID of this submap
   * @return
   */
  int id() const;
  /**
   * @brief Get number of keyframes in this submap
   * @return
   */
  int numKeyframes() const;
  /**
   * @brief Get pose of this submap in the odometry frame
   * @return
   */
  gtsam::Pose3 getPoseInOdomFrame() const;
  /**
   * @brief Set pose of this submap in the odometry frame
   * @param T_odom_submap
   */
  void setPoseInOdomFrame(const gtsam::Pose3 &T_odom_submap);
  /**
   * @brief Add a new keyframe to this submap
   * @param keyframe
   */
  void addKeyframe(const std::shared_ptr<Keyframe> &keyframe);
  /**
   * @brief Get the keyframe (nullptr if keyframe does not exist)
   * @param keyframe_id
   * @return
   */
  std::shared_ptr<Keyframe> getKeyframe(int keyframe_id) const;
  /**
   * @brief Get the set of Keyframe IDs in this submap
   * @return
   */
  std::unordered_set<int> getKeyframeIDs() const;

 private:
  const int id_;  // unique id of this submap
  std::unordered_map<int, std::shared_ptr<Keyframe>> keyframes_;  // keyframes that belong to this submap
  gtsam::Pose3 T_odom_submap_;  // the pose of this submap in the odometry frame (Kimera-VIO)
};

}

#endif //KIMERA_DISTRIBUTED_INCLUDE_KIMERA_DISTRIBUTED_SUBMAP_H_
