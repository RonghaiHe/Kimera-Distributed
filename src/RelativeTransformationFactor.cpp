/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Author: Ronghai He
 * @Date: 2025-04-10 00:44:12
 * @LastEditors: RonghaiHe hrhkjys@qq.com
 * @LastEditTime: 2025-04-15 20:11:47
 * @FilePath: /src/kimera_distributed/src/RelativeTransformationFactor.cpp
 * @Version:
 * @Description:
 *
 */
/*
 * Copyright © 2025, Sun Yat-sen University, Guangzhou, Guangdong, 510275, All Rights
 * Reserved
 * @Description: Implementation of RelativeTransformationFactor for optimization of
 * (|R*a_i+t+b_i|-d_i)^2
 */

#include "kimera_distributed/RelativeTransformationFactor.h"

namespace kimera_distributed {

RelativeTransformationFactor::RelativeTransformationFactor(
    gtsam::Key key,
    const std::vector<gtsam::Point3>& points_j,
    const std::vector<gtsam::Point3>& points_i,
    const std::vector<double>& distances,
    const gtsam::SharedNoiseModel& model)
    : NoiseModelFactor1(model, key),
      points_j_(points_j),
      points_i_(points_i),
      distances_(distances) {
  // Ensure vectors have the same size
  if (points_j_.size() != points_i_.size() || points_j_.size() != distances_.size()) {
    throw std::runtime_error(
        "Point and distance vector sizes must match in RelativeTransformationFactor");
  }
}

// gtsam::Vector RelativeTransformationFactor::evaluateError(
//     const gtsam::Pose3& pose,
//     boost::optional<gtsam::Matrix&> H) const {
//   // Extract rotation and translation from pose
//   gtsam::Rot3 R = pose.rotation();
//   gtsam::Point3 t = pose.translation();

//   // Create a scalar error (sum of squared errors)
//   gtsam::Vector error(1);
//   error[0] = 0.0;

//   // If Jacobian is requested, initialize it
//   if (H) {
//     *H = gtsam::Matrix::Zero(1, 6);  // 6 DoF for Pose3, 1 scalar error (sum)
//   }

//   // Calculate errors for each point pair and sum them
//   for (size_t i = 0; i < points_j_.size(); i++) {
//     // Calculate R*a_i + t + b_i
//     gtsam::Point3 rotated_j = R.rotate(points_j_[i]);
//     gtsam::Point3 error_vector = -rotated_j + t + points_i_[i];

//     // Calculate |R*a_i+t+b_i|^2 - d_i^2
//     double calculated_norm = std::sqrt(error_vector.dot(error_vector));
//     double point_error = calculated_norm - distances_[i];

//     // Add to the cumulative error
//     error[0] += point_error * point_error;

//     // Calculate Jacobian if requested
//     if (H) {
//       // Avoid division by zero
//       if (calculated_norm > 1e-10) {
//         gtsam::Point3 direction = error_vector / calculated_norm;

//         // Note: chain rule gives 2(|x| - d) * d(|x|)/dθ
//         double scalar = 2 * point_error;

//         // First calculate derivative of R*a_i with respect to rotation
//         gtsam::Matrix3 d_rotated_j_d_R =
//             gtsam::skewSymmetric(points_j_[i].x(), points_j_[i].y(),
//             points_j_[i].z());
//         gtsam::Matrix3 d_R_da = R.matrix() * d_rotated_j_d_R;

//         // Derivatives
//         gtsam::Matrix13 rotation_jacobian = scalar * direction.transpose() * d_R_da;
//         gtsam::Matrix13 translation_jacobian = scalar * direction.transpose();

//         // Accumulate this point's Jacobian into the full Jacobian
//         H->block<1, 3>(0, 0) += rotation_jacobian;
//         H->block<1, 3>(0, 3) += translation_jacobian;
//       }
//     }
//   }

//   return error;
// }

gtsam::Vector RelativeTransformationFactor::evaluateError(
    const gtsam::Pose3& pose,
    boost::optional<gtsam::Matrix&> H) const {
  // Extract rotation and translation from pose
  gtsam::Rot3 R = pose.rotation();
  gtsam::Point3 t = pose.translation();

  // Prepare error vector (scalar error for each point pair)
  gtsam::Vector errors(points_j_.size());

  // If Jacobian is requested, initialize it
  if (H) {
    *H = gtsam::Matrix::Zero(points_j_.size(),
                             6);  // 6 DoF for Pose3, 1 scalar error per point pair
  }

  // Calculate errors for each point pair and sum them
  for (size_t i = 0; i < points_j_.size(); i++) {
    // Calculate R*a_i + t + b_i
    gtsam::Point3 rotated_j = R.rotate(points_j_[i]);
    gtsam::Point3 error_vector = -rotated_j - t + points_i_[i];

    // Calculate |R*a_i+t+b_i|^2 - d_i^2
    double calculated_norm = std::sqrt(error_vector.dot(error_vector));
    double point_error = calculated_norm - distances_[i];

    // Add to the cumulative error
    errors[i] = point_error * point_error;

    // Calculate Jacobian if requested
    if (H) {
      gtsam::Matrix16 point_jacobian;
      // Avoid division by zero
      if (calculated_norm > 1e-10) {
        gtsam::Point3 direction = error_vector / calculated_norm;

        // Note: chain rule gives 2(|x| - d) * d(|x|)/dθ
        double scalar = 2 * point_error;

        // First calculate derivative of R*a_i with respect to rotation
        gtsam::Matrix3 d_rotated_j_d_R =
            gtsam::skewSymmetric(points_j_[i].x(), points_j_[i].y(), points_j_[i].z());
        gtsam::Matrix3 d_R_da = R.matrix() * d_rotated_j_d_R;

        // Derivatives
        point_jacobian.block<1, 3>(0, 0) = scalar * direction.transpose() * d_R_da;
        point_jacobian.block<1, 3>(0, 3) = -scalar * direction.transpose();

        // Insert this point's Jacobian into the full Jacobian
        H->row(i) = point_jacobian;
      }
    }
  }

  return errors;
}

gtsam::NonlinearFactor::shared_ptr RelativeTransformationFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new RelativeTransformationFactor(*this)));
}

}  // namespace kimera_distributed