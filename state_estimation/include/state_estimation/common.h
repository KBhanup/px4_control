#pragma once

// Eigen
#include <eigen3/Eigen/Dense>

namespace px4_ctrl {

struct eskf_state {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond attitude;
  Eigen::Vector3d disturbances;
  Eigen::Vector3d random_walk_bias;
  Eigen::Vector3d marker_position;
  Eigen::Quaterniond marker_orientation;
};

/**
 * @brief Clips a value between bounds
 * @param value The value to be clipped
 * @param l_bound Lower bound
 * @param u_bound Upper bound
 */
template <class num>
static inline num clipValue(const num &value, const num &l_bound,
                            const num &u_bound) {
  num clipped_value = value < l_bound ? l_bound : value;
  clipped_value = clipped_value > u_bound ? u_bound : clipped_value;

  return clipped_value;
}

/**
 * @brief Returns a skew symmetric matrix from a vector
 * @param v Vector3 to turn to skew symmetric
 * @return Skew symmetric matrix
 */
static inline Eigen::Matrix3d toSkew(const Eigen::Vector3d v) {
  Eigen::Matrix3d s;
  s << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return s;
}

/**
 * @brief Gets the rotation matrix from the Euler angles (Z-Y-X)
 * @param yaw Yaw angle
 * @param pitch Pitch angle
 * @param roll Roll angle
 * @returns Rotation matrix
 */
static inline Eigen::Matrix3d eulerToRotMat(const double &yaw,
                                            const double &pitch,
                                            const double &roll) {
  Eigen::Matrix3d Rot_mat;
  Rot_mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  return Rot_mat;
}

/**
 * @brief Gets the error state derivative of the state
 * @param state The current state
 * @param state_size The size of the state
 * @param error_state_size The size of the error state
 * @returns The error state derivative of the state
 */
static inline Eigen::MatrixXd getXddx(const eskf_state state,
                                      const int state_size,
                                      const int error_state_size) {
  Eigen::MatrixXd Xddx = Eigen::MatrixXd::Zero(state_size, error_state_size);

  // Position
  Xddx.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

  // Velocity
  Xddx.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

  // Attitude
  Xddx(6, 6) = -0.5 * state.attitude.x();
  Xddx(6, 7) = -0.5 * state.attitude.y();
  Xddx(6, 8) = -0.5 * state.attitude.z();

  Xddx(7, 6) = 0.5 * state.attitude.w();
  Xddx(7, 7) = -0.5 * state.attitude.z();
  Xddx(7, 8) = 0.5 * state.attitude.y();

  Xddx(8, 6) = 0.5 * state.attitude.z();
  Xddx(8, 7) = 0.5 * state.attitude.w();
  Xddx(8, 8) = -0.5 * state.attitude.x();

  Xddx(9, 6) = -0.5 * state.attitude.y();
  Xddx(9, 7) = 0.5 * state.attitude.x();
  Xddx(9, 8) = 0.5 * state.attitude.w();

  // Disturbances
  Xddx.block(10, 9, 3, 3) = Eigen::Matrix3d::Identity();

  // Biases
  Xddx.block(13, 12, 3, 3) = Eigen::Matrix3d::Identity();

  // Marker position
  Xddx.block(16, 15, 3, 3) = Eigen::Matrix3d::Identity();

  // Marker Orientation
  Xddx(19, 18) = -0.5 * state.marker_orientation.x();
  Xddx(19, 19) = -0.5 * state.marker_orientation.y();
  Xddx(19, 20) = -0.5 * state.marker_orientation.z();

  Xddx(20, 18) = 0.5 * state.marker_orientation.w();
  Xddx(20, 19) = -0.5 * state.marker_orientation.z();
  Xddx(20, 20) = 0.5 * state.marker_orientation.y();

  Xddx(21, 18) = 0.5 * state.marker_orientation.z();
  Xddx(21, 19) = 0.5 * state.marker_orientation.w();
  Xddx(21, 20) = -0.5 * state.marker_orientation.x();

  Xddx(22, 18) = -0.5 * state.marker_orientation.y();
  Xddx(22, 19) = 0.5 * state.marker_orientation.x();
  Xddx(22, 20) = 0.5 * state.marker_orientation.w();

  return Xddx;
}

/**
 * @brief Gets the derivative of a rotation matrix with respect to qw
 * @param q The rotation quaternion
 * @returns The derivative of R with respect to qw
 */
static inline Eigen::Matrix3d dRdqw(const Eigen::Quaterniond q) {
  Eigen::Matrix3d dR;
  dR << q.w(), -q.z(), q.y(), q.z(), q.w(), -q.x(), -q.y(), q.x(), q.w();
  return 2 * dR;
}

/**
 * @brief Gets the derivative of a rotation matrix with respect to qx
 * @param q The rotation quaternion
 * @returns The derivative of R with respect to qx
 */
static inline Eigen::Matrix3d dRdqx(const Eigen::Quaterniond q) {
  Eigen::Matrix3d dR;
  dR << q.x(), q.y(), q.z(), q.y(), -q.x(), -q.w(), q.z(), q.w(), -q.x();
  return 2 * dR;
}

/**
 * @brief Gets the derivative of a rotation matrix with respect to qy
 * @param q The rotation quaternion
 * @returns The derivative of R with respect to qy
 */
static inline Eigen::Matrix3d dRdqy(const Eigen::Quaterniond q) {
  Eigen::Matrix3d dR;
  dR << -q.y(), q.x(), q.w(), q.x(), q.y(), q.z(), -q.w(), q.z(), -q.y();
  return 2 * dR;
}

/**
 * @brief Gets the derivative of a rotation matrix with respect to qz
 * @param q The rotation quaternion
 * @returns The derivative of R with respect to qz
 */
static inline Eigen::Matrix3d dRdqz(const Eigen::Quaterniond q) {
  Eigen::Matrix3d dR;
  dR << -q.z(), -q.w(), q.x(), q.w(), -q.z(), q.y(), q.x(), q.y(), q.z();
  return 2 * dR;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to pw
 * @param q The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to pw
 */
static inline Eigen::Vector4d dpqdpw(const Eigen::Quaterniond q) {
  Eigen::Vector4d dpq;
  dpq << q.w(), q.x(), q.y(), q.z();
  return dpq;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to px
 * @param q The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to px
 */
static inline Eigen::Vector4d dpqdpx(const Eigen::Quaterniond q) {
  Eigen::Vector4d dpq;
  dpq << -q.x(), q.w(), -q.z(), q.y();
  return dpq;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to py
 * @param q The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to py
 */
static inline Eigen::Vector4d dpqdpy(const Eigen::Quaterniond q) {
  Eigen::Vector4d dpq;
  dpq << -q.y(), q.z(), q.w(), -q.x();
  return dpq;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to pz
 * @param q The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to pz
 */
static inline Eigen::Vector4d dpqdpz(const Eigen::Quaterniond q) {
  Eigen::Vector4d dpq;
  dpq << -q.z(), -q.y(), q.x(), q.w();
  return dpq;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to qw
 * @param p The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to qw
 */
static inline Eigen::Vector4d dpqdqw(const Eigen::Quaterniond p) {
  Eigen::Vector4d dpq;
  dpq << p.w(), p.x(), p.y(), p.z();
  return dpq;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to qx
 * @param p The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to qx
 */
static inline Eigen::Vector4d dpqdqx(const Eigen::Quaterniond p) {
  Eigen::Vector4d dpq;
  dpq << -p.x(), p.w(), p.z(), -p.y();
  return dpq;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to qy
 * @param p The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to qy
 */
static inline Eigen::Vector4d dpqdqy(const Eigen::Quaterniond p) {
  Eigen::Vector4d dpq;
  dpq << -p.y(), -p.z(), p.w(), p.x();
  return dpq;
}

/**
 * @brief Gets the derivative of the quaternion product pq with respect to qz
 * @param p The rotation quaternion
 * @returns The derivative of the quaternion product pq with respect to qz
 */
static inline Eigen::Vector4d dpqdqz(const Eigen::Quaterniond p) {
  Eigen::Vector4d dpq;
  dpq << -p.z(), p.y(), -p.x(), p.w();
  return dpq;
}

}  // namespace px4_ctrl
