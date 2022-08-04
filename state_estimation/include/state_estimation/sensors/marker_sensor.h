#pragma once

#include "state_estimation/sensors/base_sensor.h"

namespace px4_ctrl {
/**
 * @brief Sensor class for Marker sensor. The measurement is the relative
 * position and orientation of a stationary marker in the body frame
 */
class MarkerSensor : public BaseSensor {
 public:
  MarkerSensor() {}
  MarkerSensor(Eigen::MatrixXd R_mat, const int &N) {
    R_mat_nom = R_mat;
    R_mat_cur = R_mat;
    res_size = N;
    res_full = false;
    cyclic_index = 0;
    res_Mat = Eigen::MatrixXd::Zero(R_mat.rows(), N);
  }
  ~MarkerSensor() {}

  /**
   * @brief Returns the H matrix and expected sensor measurement for the
   * predicted state
   * @param state Predicted state
   * @param H_mat Sensor's H matrix
   * @param y_pred Sensor's expected measurement at the predicted state
   */
  virtual void correctionData(const eskf_state &state, Eigen::MatrixXd &H_mat,
                              Eigen::MatrixXd &y_expected) override {
    H_mat = getHmat(state);
    y_expected = getYExpected(state);
  }

  Eigen::MatrixXd getCurrentR() const { return R_mat_cur; }

 private:
  static const int measurement_size = 7;
  static const int state_size = 23;
  static const int error_state_size = 21;

  /**
   * @brief Calculates the H matrix using the predicted state
   * @param state Predicted state
   * @returns The H matrix at the provided state
   */
  Eigen::MatrixXd getHmat(const eskf_state &state) {
    // Quaternion derivatives of R transpose
    Eigen::Matrix3d R_w = dRdqw(state.attitude).transpose();
    Eigen::Matrix3d R_x = dRdqx(state.attitude).transpose();
    Eigen::Matrix3d R_y = dRdqy(state.attitude).transpose();
    Eigen::Matrix3d R_z = dRdqz(state.attitude).transpose();

    // Construct H matrix
    Eigen::Matrix<double, measurement_size, state_size> H_mat;
    H_mat.setZero();

    // Drone Position
    H_mat.block(0, 0, 3, 3) = -state.attitude.toRotationMatrix().transpose();

    // Drone Attitude
    H_mat.block(0, 6, 3, 1) = R_w * (state.marker_position - state.position);
    H_mat.block(0, 7, 3, 1) = R_x * (state.marker_position - state.position);
    H_mat.block(0, 8, 3, 1) = R_y * (state.marker_position - state.position);
    H_mat.block(0, 9, 3, 1) = R_z * (state.marker_position - state.position);

    H_mat.block(3, 6, 4, 1) = dpqdpw(state.marker_orientation);
    H_mat.block(3, 7, 4, 1) = -dpqdpx(state.marker_orientation);
    H_mat.block(3, 8, 4, 1) = -dpqdpy(state.marker_orientation);
    H_mat.block(3, 9, 4, 1) = -dpqdpz(state.marker_orientation);

    // Marker Position
    H_mat.block(0, 16, 3, 3) = state.attitude.toRotationMatrix().transpose();

    // Marker Orientation
    H_mat.block(3, 19, 4, 1) = dpqdqw(state.attitude.inverse());
    H_mat.block(3, 20, 4, 1) = dpqdqx(state.attitude.inverse());
    H_mat.block(3, 21, 4, 1) = dpqdqy(state.attitude.inverse());
    H_mat.block(3, 22, 4, 1) = dpqdqz(state.attitude.inverse());

    // Error state derivative of the state
    Eigen::MatrixXd Xddx = getXddx(state, state_size, error_state_size);

    return H_mat * Xddx;
  }

  /**
   * @brief Calculates the expected sensor measurement using the predicted state
   * @param state Predicted state
   * @returns The expected measurement at the provided state
   */
  Eigen::MatrixXd getYExpected(const eskf_state &state) {
    Eigen::Quaterniond q_meas =
        state.attitude.inverse() * state.marker_orientation;
    Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();

    Eigen::Vector3d p_meas =
        R_mat.transpose() * (state.marker_position - state.position);

    Eigen::Matrix<double, measurement_size, 1> y_exp;

    y_exp.segment(0, 3) = p_meas;
    y_exp(3) = q_meas.w();
    y_exp(4) = q_meas.x();
    y_exp(5) = q_meas.y();
    y_exp(6) = q_meas.z();

    return y_exp;
  }
};
}  // namespace px4_ctrl