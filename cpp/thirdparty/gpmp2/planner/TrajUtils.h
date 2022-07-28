/**
 *  @file  TrajUtils.h
 *  @brief utils for trajectory optimization, include initialization and interpolation
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 11, 2015
 **/

#pragma once

// #include <cpp/thirdparty/gpmp2/config.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>


namespace incopt {
/* ************************************************************************** */
/**
 * @brief initial the trajectory in configuration space as a straight line
 * @param init_conf trajectory start configuration
 * @param end_conf  trajectory end configuration
 * @param total_step is how many intervals do you want in the traj
 * @return values, x(0) - x(total_step), v(0) - v(total_step)
 */
// gtsam::Values initArmTrajStraightLine(const gtsam::Vector& init_conf,
//     const gtsam::Vector& end_conf, size_t total_step);
gtsam::Values initArmTrajStraightLine(const Vector& init_conf,
    const Vector& end_conf, size_t total_step) {

  Values init_values;

  // init pose
  for (size_t i = 0; i <= total_step; i++) {
    Vector conf;
    if (i == 0)
      conf = init_conf;
    else if (i == total_step)
      conf = end_conf;
    else
      conf = static_cast<double>(i) / static_cast<double>(total_step) * end_conf +
          (1.0 - static_cast<double>(i) / static_cast<double>(total_step)) * init_conf;

    init_values.insert(Symbol('x', i), conf);
  }

  // init vel as avg vel
  Vector avg_vel = (end_conf - init_conf) / static_cast<double>(total_step);
  for (size_t i = 0; i <= total_step; i++)
    init_values.insert(Symbol('v', i), avg_vel);

  return init_values;
}

/* ************************************************************************** */

/**
 * @brief initialize the trajectory in configuration space as a straight line
 * @param init_pose trajectory start pose
 * @param init_conf trajectory start configuration
 * @param end_pose  trajectory end pose
 * @param end_conf  trajectory end configuration
 * @param total_step is how many intervals do you want in the traj
 * @return values, x(0) - x(total_step), v(0) - v(total_step)
 */
gtsam::Values initPose2VectorTrajStraightLine(const gtsam::Pose2& init_pose,
    const gtsam::Vector& init_conf, const gtsam::Pose2& end_pose, const gtsam::Vector& end_conf, size_t total_step);

/**
 * @brief initialize the trajectory in configuration space as a straight line
 * @param init_pose trajectory start pose
 * @param end_pose  trajectory end pose
 * @param total_step is how many intervals do you want in the traj
 * @return values, x(0) - x(total_step), v(0) - v(total_step)
 */
gtsam::Values initPose2TrajStraightLine(const gtsam::Pose2& init_pose,
    const gtsam::Pose2& end_pose, size_t total_step);

/**
 * @brief robot arm trajectory interpolator
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @return interpolated values, x(0) - x(total_step), v(0) - v(total_step)
 */
// gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
//     const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t output_inter_step);
/* ************************************************************************** */
gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step) {

  // inter setting
  double inter_dt = delta_t / static_cast<double>(inter_step + 1);

  size_t last_pos_idx;
  size_t inter_pos_count = 0;

  // results
  Values results;

  // TODO: gtsam keyvector has issue: free invalid pointer
  KeyVector key_vec = opt_values.keys();

  // sort key list
  std::sort(key_vec.begin(), key_vec.end());

  for (size_t i = 0; i < key_vec.size(); i++) {
    Key key = key_vec[i];

    if (Symbol(key).chr() == 'x') {
      size_t pos_idx = Symbol(key).index();

      if (pos_idx != 0) {
        // skip first pos to interpolate

        for (size_t inter_idx = 1; inter_idx <= inter_step+1; inter_idx++) {

          if (inter_idx == inter_step+1) {
            // last pose
            results.insert(Symbol('x', inter_pos_count), opt_values.at<Vector>(Symbol('x', pos_idx)));
            results.insert(Symbol('v', inter_pos_count), opt_values.at<Vector>(Symbol('v', pos_idx)));

          } else {
            // inter pose
            double tau = static_cast<double>(inter_idx) * inter_dt;
            GaussianProcessInterpolatorLinear gp_inter(Qc_model, delta_t, tau);
            Vector conf1 = opt_values.at<Vector>(Symbol('x', last_pos_idx));
            Vector vel1  = opt_values.at<Vector>(Symbol('v', last_pos_idx));
            Vector conf2 = opt_values.at<Vector>(Symbol('x', pos_idx));
            Vector vel2  = opt_values.at<Vector>(Symbol('v', pos_idx));
            Vector conf  = gp_inter.interpolatePose(conf1, vel1, conf2, vel2);
            Vector vel  = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2);
            results.insert(Symbol('x', inter_pos_count), conf);
            results.insert(Symbol('v', inter_pos_count), vel);
          }
          inter_pos_count++;
        }

      } else {
        // cache first pose
        results.insert(Symbol('x', 0), opt_values.at<Vector>(Symbol('x', 0)));
        results.insert(Symbol('v', 0), opt_values.at<Vector>(Symbol('v', 0)));
        inter_pos_count++;
      }

      last_pos_idx = pos_idx;
    }
  }

  return results;
}

/**
 * @brief robot arm trajectory interpolator between any two states
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @param start_index interpolate from this state
 * @param end_index interpolate till this state
 * @return interpolated values, x(start_index) - x(end_index), v(start_index) - v(end_index)
 */
gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t output_inter_step, 
    size_t start_index, size_t end_index);

/**
 * @brief mobile arm trajectory interpolator between any two states
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @param start_index interpolate from this state
 * @param end_index interpolate till this state
 * @return interpolated values, x(start_index) - x(end_index), v(start_index) - v(end_index)
 */
gtsam::Values interpolatePose2MobileArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index);

/**
 * @brief mobile base trajectory interpolator between any two states
 * @param opt_values optimized values from optimization
 * @param Qc_model GP hyperparameter
 * @param delta_t time interval between states in opt_values
 * @param output_inter_step is how many steps you want interpolate, 0 is not interpolate
 * @param start_index interpolate from this state
 * @param end_index interpolate till this state
 * @return interpolated values, x(start_index) - x(end_index), v(start_index) - v(end_index)
 */
gtsam::Values interpolatePose2Traj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index);
}
