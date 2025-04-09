/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Author: Ronghai He
 * @Date: 2025-04-08 11:10:41
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-04-08 11:21:21
 * @FilePath:
 * /src/kimera_distributed/include/kimera_distributed/RelativeTransformationFactor.h
 * /src/kimera_distributed/include/kimera_distributed/RelativeTransformationFactor.h
 * @Version:
 * @Description:
 *
 */
/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Description: Header file for RelativeTransformationFactor to optimize
 * |R*a_i+t+b_i|^2-d_i^2
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <vector>

namespace kimera_distributed {

/**
 * @brief Factor implementing the cost function: sum_i (||R*a_i+t+b_i||^2 - d_i^2)
 *
 * This factor optimizes for a rotation matrix R and translation vector t
 * given sets of points a_i, b_i and distance measurements d_i.
 */
class RelativeTransformationFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  // Input points
  std::vector<gtsam::Point3> points_j_;
  std::vector<gtsam::Point3> points_i_;
  // Distance measurements
  std::vector<double> distances_;

 public:
  /**
   * @brief Constructor
   * @param key Key for the unknown Pose3 (rotation R and translation t)
   * @param points_j Vector of a_i points
   * @param points_i Vector of b_i points
   * @param distances Vector of distance measurements d_i
   * @param model Noise model
   */
  RelativeTransformationFactor(gtsam::Key key,
                               const std::vector<gtsam::Point3>& points_j,
                               const std::vector<gtsam::Point3>& points_i,
                               const std::vector<double>& distances,
                               const gtsam::SharedNoiseModel& model);

  // Evaluate error for the cost function
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const override;

  // Clone method required by gtsam
  gtsam::NonlinearFactor::shared_ptr clone() const override;

};  // class RelativeTransformationFactor

}  // namespace kimera_distributed