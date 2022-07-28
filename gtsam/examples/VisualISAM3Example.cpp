/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualISAM2Example.cpp
 * @brief   A visualSLAM example for the structure-from-motion problem on a
 * simulated dataset This version uses iSAM2 to solve the problem incrementally
 * @author  Duy-Nguyen Ta
 */

/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

// For loading the data
#include "SFMdata.h"

// Camera observations of landmarks will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/BetweenFactor.h>
// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem
// incrementally, so include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set of new factors to be added stored in a factor
// graph, and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <vector>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(1000.0, 1000.0, 0.0, 50.0, 50.0));


  std::map<int, std::vector<int>> pose_to_landmark;

  pose_to_landmark[0] = {0};
  pose_to_landmark[1] = {0};
  pose_to_landmark[2] = {};
  pose_to_landmark[3] = {};
  pose_to_landmark[4] = {1};
  pose_to_landmark[5] = {1};
  
  // Define the camera observation noise model, 1 pixel stddev
  auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  std::cout << "===========Initializing isam ===============" << std::endl;
  parameters.setFactorization("QR");

  ISAM2 isam(parameters);
  isam.setEnableConstraints(true);

  cout << "fact " << parameters.factorization << endl;

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph graph;
  Values initialEstimate;
  static Pose3 kDeltaPose(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                            Point3(0.05, -0.10, 0.20));
  static Point3 kDeltaPoint(-0.25, 0.20, 0.15);
  Vector3 translation_noise(0.5, 0.1, 0.1);
   noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << translation_noise, Vector3::Constant(0.1))
        .finished());
  // Loop over the poses, adding the observations to iSAM incrementally
  map<int, int> seen;
  for (int i = 0; i < pose_to_landmark.size(); ++i) {
      cout << " =========================== Processing pose ================================" << i << " =========================== Processing pose ================================" << endl;
  
      vector<int> landmarks = pose_to_landmark[i]; 
      PinholeCamera<Cal3_S2> camera(poses[i], *K);
      for(int landmark: landmarks) {
        Point2 measurement = camera.project(points[landmark]);
        if (seen.find(landmark) == seen.end()) {
          seen[landmark]+=1;
        }
        else if(seen[landmark] == 1) {
            // Need to insert initial estimate and factor 
            initialEstimate.insert<Point3>(Symbol('l', landmark), points[landmark] + kDeltaPoint);
            graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
              measurement, measurementNoise, Symbol('x', i), Symbol('l', landmark), K);
            graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
              measurement, measurementNoise, Symbol('x', i-1), Symbol('l', landmark), K);
            seen[landmark] += 1;
        }
        else {
            graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
              measurement, measurementNoise, Symbol('x', i), Symbol('l', landmark), K);
            seen[landmark] += 1;
        }
      }
      if(i == 0) {
        static auto kPosePrior = noiseModel::Diagonal::Sigmas(
            (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
                .finished());
        graph.addPrior(Symbol('x', 0), poses[0], kPosePrior);

        // Add a prior on landmark l0
        static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
        graph.addPrior(Symbol('l', landmarks[0]), points[landmarks[0]], kPointPrior);
      }
     
      initialEstimate.insert(Symbol('x', i), poses[i] * kDeltaPose);
      //initialEstimate.print();
      if(i>0) {
        Pose3 posebetween = poses[i-1].inverse()*poses[i];
        
        BetweenFactor<Pose3> factor(Symbol('x', i-1), Symbol('x', i), posebetween , odometryNoise);
        std::cout << " Size of posebetween " << factor.dim() << " | " << factor.size() << std::endl;
        //factor.print("yp");
        //factor.setIsConstraintFactor(1);
        //factor.setConstraintFactorType(1);
        graph.add(BetweenFactor<Pose3>(Symbol('x', i-1), Symbol('x', i), posebetween , odometryNoise));
        std::cout << "***************************************UPDATE AGAIN*********************************" << std::endl;
        //std::cout << "Adding the following factors " << std::endl;
        //graph.print();
        isam.update(graph, initialEstimate);
        //std::cout << "Bayes tree after update " << std::endl;
        //isam.print();
        //isam.update();
        //isam.update();
        //isam.update();
        //isam.update();
        Values currentEstimate = isam.calculateEstimate();
        //LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
        //Values currentEstimate = optimizer.optimize();

        //currentEstimate.print("Current estimate: ");
        graph.resize(0);
        initialEstimate.clear();
      }       
  }

  return 0;
}

/*
Value l0: (Eigen::Matrix<double, 3, 1, 0, 3, 1>)
[
        9.9904113;
        9.98739329;
        10.0094893
]

Value l1: (Eigen::Matrix<double, 3, 1, 0, 3, 1>)
[
        -3.75449876;
        15.8396265;
        1.10404678
]

Value x0: (gtsam::Pose3)
R: [
        0.348585486, 0.0501561607, -0.935934035;
        0.915717356, -0.231169469, 0.328667614;
        -0.199874668, -0.9716198, -0.126511188
]
t:    30.0745892   0.138767432 -0.0836742238

Value x1: (gtsam::Pose3)
R: [
        -0.594192057, 0.0313443751, -0.803712218;
        0.798437436, -0.0976770705, -0.594101717;
        -0.0971260021, -0.994724444, 0.0330124169
]
t:   29.256598  22.4465312 -5.43894231

Value x2: (gtsam::Pose3)
R: [
        -0.988467598, 0.0313443088, -0.148153101;
        0.14448715, -0.0976771208, -0.984673877;
        -0.0453350904, -0.994724441, 0.0920218192
]
t:  9.58982764  34.1636951 -7.20922888

Value x3: (gtsam::Pose3)
R: [
        -0.803712201, 0.0313443041, 0.594192084;
        -0.594101736, -0.0976771366, -0.798437414;
        0.0330125163, -0.99472444, 0.0971260119
]
t: -12.6805264  28.5766013 -7.36235555

Value x4: (gtsam::Pose3)
R: [
        -0.148153095, 0.031344304, 0.988467599;
        -0.984673876, -0.0976771371, -0.144487144;
        0.0920218363, -0.99472444, 0.0453350875
]
t: -24.5087918  8.95809322 -5.80862783

Value x5: (gtsam::Pose3)
R: [
        0.593210002, -0.0447361277, 0.803803814;
        -0.798941296, -0.155458932, 0.580969298;
        0.0989681657, -0.98682886, -0.127961333
]
t: -18.9668144 -13.2000619 -3.45772371






*/

