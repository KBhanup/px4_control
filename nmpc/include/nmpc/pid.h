#pragma once

#include <vector>

namespace px4_ctrl {
/**
 * @brief PID controller class
 */
class PIDController {
 public:
  PIDController() {}
  PIDController(double k_p, double k_i, double k_d,
                size_t integration_horizon) {
    kp = k_p;
    ki = k_i;
    kd = k_d;

    error_buffer_size = integration_horizon > 0 ? integration_horizon : 1;
    reset();
  }
  ~PIDController() {}

  /**
   * @brief Resets the controller
   */
  void reset() {
    previous_error.clear();
    previous_error_time.clear();
  }

  /**
   * @brief Returns the calculated control input
   * @param error Current error
   * @param error_time Error timestamp
   */
  double getControl(double error, double error_time) {
    // Proportional
    double ctrl = kp * error;

    if (previous_error.size() > 0) {
      // Integral
      if (ki > 0.0) {
        if (previous_error.size() > 1) {
          for (size_t i = 0; i < previous_error.size() - 1; i++) {
            ctrl += ki * previous_error[i] *
                    (previous_error_time[i + 1] - previous_error_time[i]);
          }
        }
        ctrl += ki * previous_error[previous_error.size() - 1] *
                (error_time - previous_error_time[previous_error.size() - 1]);
      }

      // Derivative
      if (kd > 0.0) {
        ctrl += kd * (error - previous_error[previous_error.size() - 1]) /
                (error_time - previous_error_time[previous_error.size() - 1]);
      }
    }

    // Update error buffers
    updateErrorBuffers(error, error_time);

    return ctrl;
  }

 private:
  /**
   * @brief Updates the error buffers
   * @param error Latest error
   * @param error_time Latest error time
   */
  void updateErrorBuffers(double error, double error_time) {
    previous_error.emplace_back(error);
    previous_error_time.emplace_back(error_time);

    if (previous_error.size() > error_buffer_size) {
      previous_error.erase(previous_error.begin());
      previous_error_time.erase(previous_error_time.begin());
    }
  }

  double kp, kd, ki;
  size_t error_buffer_size;
  std::vector<double> previous_error;
  std::vector<double> previous_error_time;
};
}  // namespace px4_ctrl