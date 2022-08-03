#pragma once

#include "state_estimation/sensors/base_sensor.h"

namespace px4_ctrl {
/**
 * @brief Sensor class for odometry sensor. The measurement is the velocity in
 * the body frame
 */
class OdometrySensor : public BaseSensor {
 public:
  OdometrySensor() {}
  OdometrySensor(Eigen::MatrixXd R_mat, const int &N) {
    R_mat_nom = R_mat;
    R_mat_cur = R_mat;
    res_size = N;
    res_full = false;
    cyclic_index = 0;
    res_Mat = Eigen::MatrixXd::Zero(R_mat.rows(), N);
  }
  ~OdometrySensor() {}

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
  static const int measurement_size = 3;
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

    // Velocity
    H_mat.block(0, 3, 3, 3) = state.attitude.toRotationMatrix().transpose();
    H_mat.block(0, 6, 3, 1) = R_w * state.velocity;
    H_mat.block(0, 7, 3, 1) = R_x * state.velocity;
    H_mat.block(0, 8, 3, 1) = R_y * state.velocity;
    H_mat.block(0, 9, 3, 1) = R_z * state.velocity;

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
    Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();

    Eigen::Matrix<double, measurement_size, 1> y_exp;

    y_exp = R_mat.transpose() * state.velocity;

    return y_exp;
  }
};
}  // namespace px4_ctrl