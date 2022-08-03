#pragma once

// Eigen
#include <eigen3/Eigen/Dense>

// ROS
#include "state_estimation/common.h"
#include "tf/transform_datatypes.h"

namespace px4_ctrl {
class BaseSensor {
 public:
  BaseSensor() {}
  BaseSensor(const Eigen::MatrixXd &R_mat, const int &N) {
    R_mat_nom = R_mat;
    R_mat_cur = R_mat;
    res_size = N;
    res_full = false;
    cyclic_index = 0;
    res_Mat = Eigen::MatrixXd::Zero(R_mat.rows(), N);
  }
  ~BaseSensor() {}

  /**
   * @brief Returns the H matrix and expected sensor measurement for the
   * predicted state
   * @param state_pred Predicted state
   * @param H_mat Sensor's H matrix
   * @param y_pred Sensor's expected measurement at the predicted state
   */
  virtual void correctionData(const eskf_state &state_pred,
                              Eigen::MatrixXd &H_mat,
                              Eigen::MatrixXd &y_expected) {}

  /**
   * @brief Use Covariance Matching (Mehra 1972) to update the sensor covariance
   * matrix
   * @param innov Current innovation
   * @param Py Current HPH^T
   */
  void updateR(const Eigen::MatrixXd &innov, const Eigen::MatrixXd &Py) {
    res_Mat.block(0, cyclic_index, innov.rows(), innov.cols()) = innov;
    cyclic_index++;

    if (cyclic_index >= res_size) {
      cyclic_index = 0;
      res_full = true;
    }

    if (res_full) {
      Eigen::MatrixXd R_hat = (res_Mat * res_Mat.transpose()) / res_size - Py;

      for (size_t i = 0; i < R_hat.rows(); i++) {
        if (R_hat(i, i) > R_mat_nom(i, i))
          R_mat_cur(i, i) = R_hat(i, i);
        else
          R_mat_cur(i, i) = R_mat_nom(i, i);
      }
    }
  }

 protected:
  Eigen::MatrixXd R_mat_nom, R_mat_cur;
  Eigen::MatrixXd res_Mat;
  bool res_full;
  Eigen::Index cyclic_index;
  int res_size;
};
}  // namespace px4_ctrl