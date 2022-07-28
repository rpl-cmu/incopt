/**
 * @file ContactFactors.h
 * @brief contact equality constraint factors
 * @author Paloma Sodhi
 */

// BEGIN CONSTRAINTS EDITS

#ifndef CONTACT_FACTORS_H_
#define CONTACT_FACTORS_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "cpp/pushing/dataio/DataLoaderJson.h"

#include "ConstraintUtils.h"
using json = nlohmann::json;

namespace incopt {

class ContactMeasurement {
 public:
  gtsam::Vector2 contactPoint__world_;
  gtsam::Vector2 eeCenter__world_;
  Matrix objPolyShape_;
  double eeRadius_;

  ContactMeasurement(const gtsam::Vector2 contactPoint__world, const gtsam::Vector2 eeCenter__world,
                     const Matrix objPolyShape, const double eeRadius)
      : contactPoint__world_(contactPoint__world), eeCenter__world_(eeCenter__world), objPolyShape_(objPolyShape), eeRadius_(eeRadius) {}

  void write(std::ostream& out) const {
    out << "("
        << "contact_point" << contactPoint__world_ << " "
        << "ee_center" << eeCenter__world_ << " "
        << "ee_radius"
        << eeRadius_ << ")";
  }
};

// factor for planar pushing dataset generated using physics simulator
class ContactPush2dUnaryFactor : public NoiseModelFactor1<Pose2> {
 private:
  ContactMeasurement contactMeas_;

 public:
  /**
   * constructor
   */
  ContactPush2dUnaryFactor(Key poseKey, ContactMeasurement contactMeas, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactor1(model, poseKey), contactMeas_(contactMeas) {
    isConstraintFactor_ = true;
    constraintFactorType_ = 1;  // equality
  }

  /**
   * evaluateError
   */
  Vector evaluateError(const Pose2& p, boost::optional<Matrix&> H = boost::none) const {
    // Vector errVec = conutils::contactPush2dUnaryError1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    // if (H) *H = conutils::contactPush2dUnaryErrorJacobian1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);

    Vector errVec = conutils::contactPush2dUnaryError3D(p, contactMeas_.eeCenter__world_, contactMeas_.contactPoint__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    if (H) *H = conutils::contactPush2dUnaryErrorJacobian3D(p, contactMeas_.eeCenter__world_, contactMeas_.contactPoint__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);

    return errVec;
  }

  /**
   * constraint error
   */
  Vector constraintError(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Vector1 errVec = conutils::contactPush2dUnaryError1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    return errVec;
  }
  /**
   * constraint jacobian
   */
  Matrix constraintJacobian(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Matrix G = conutils::contactPush2dUnaryErrorJacobian1D(p, contactMeas_.eeCenter__world_, contactMeas_.objPolyShape_, contactMeas_.eeRadius_);
    return G;
  }

  Vector2 getPointPolyProj(const Pose2& p) {
    double dist;
    Vector2 pointPoly__obj;
    Point2 eeCenter__obj = p.transformTo(Point2(contactMeas_.eeCenter__world_));
    p_poly_dist(eeCenter__obj, contactMeas_.objPolyShape_, Matrix(), Matrix(),
                &dist, NULL, NULL, NULL, NULL, &pointPoly__obj, NULL, NULL, 0);

    Point2 pointPoly__world = p.transformFrom(Point2(pointPoly__obj));

    return pointPoly__world;
  }
};

// factor for sim arm2d dataset generated using matlab
class ContactCircular2dFactor : public NoiseModelFactor1<Pose2> {
 private:
  double tx_, ty_;  // tx, ty

 public:
  /** Constructor
   * @param poseKey   associated pose variable key
   * @param model     noise model for contact sensor, 1-dim
   * @param toolPos   curr tool/endeff position in Vector2
  */
  ContactCircular2dFactor(Key poseKey, const Vector2 toolPos, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactor1(model, poseKey), tx_(toolPos.x()), ty_(toolPos.y()) {
    isConstraintFactor_ = true;
    constraintFactorType_ = 1;  // equality
  }

  /** error function
   * @param      curr pose in Pose2
   * @param H    optional Jacobian matrix, uses boost optional and has default null pointer
  */
  Vector evaluateError(const Pose2& p, boost::optional<Matrix&> H = boost::none) const {
    // note: use boost optional like a pointer
    // only calculate Jacobian matrix when non-null pointer exists
    if (H) *H = conutils::contactCircular2DErrorJacobian(p, tx_, ty_);

    // return error vector
    Vector1 errVec = conutils::contactCircular2DError(p, tx_, ty_);

    return (Vector1() << errVec(0)).finished();
  }

  /**
   * constraint params
   */
  Vector constraintParams() const {
    return (Vector2() << tx_, ty_).finished();
  }

  /**
   * constraint error
   */
  Vector constraintError(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Vector1 errVec = conutils::contactCircular2DError(p, tx_, ty_);
    return errVec;
  }
  /**
   * constraint jacobian
   */
  Matrix constraintJacobian(const Value& val) const {
    Pose2 p = val.cast<Pose2>();
    Matrix G = conutils::contactCircular2DErrorJacobian(p, tx_, ty_);
    return G;
  }
};

class contactConstraintUtils {

  public:
    contactConstraintUtils() {}

    void writeOutputsJson(const gtsam::ISAM2& isam2, const Values& poseValsGraph,
                          const Values& poseValsGT, const Values& poseValsOdom,
                          const Values& eePoseValsNoisy, const string& filename) {
      ofstream fileStream(filename);
      json js;

      map<unsigned int, vector<double> > posesGraph;
      map<unsigned int, vector<double> > posesGT;
      map<unsigned int, vector<double> > posesOdom;
      map<unsigned int, vector<double> > eePosesNoisy;

      // save object pose estimates from optimizer
      for (const auto& keyValue : poseValsGraph) {
        Pose2 pose = (keyValue.value).cast<Pose2>();
        vector<double> poseVec(3);

        poseVec[0] = pose.x();
        poseVec[1] = pose.y();
        poseVec[2] = pose.theta();

        pair<unsigned int, vector<double> > keyPosePair(keyValue.key, poseVec);
        posesGraph.insert(keyPosePair);

        // Vector2 pointPoly__world = getPolyProjPoint(pose);
        // pair<unsigned int, vector<double> > keyPosePair(keyValue.key, poseVec);
        // posesGraph.insert(keyPosePair);
      }
      js["obj_poses_graph"] = posesGraph;

      // save GT object poses
      for (const auto& keyValue : poseValsGT) {
        Pose2 pose = (keyValue.value).cast<Pose2>();
        vector<double> poseVec(3);

        poseVec[0] = pose.x();
        poseVec[1] = pose.y();
        poseVec[2] = pose.theta();

        pair<unsigned int, vector<double> > keyPosePair(keyValue.key, poseVec);
        posesGT.insert(keyPosePair);
      }
      js["obj_poses_gt"] = posesGT;

      // save object poses based on propogated odom only
      for (const auto& keyValue : poseValsOdom) {
        Pose2 pose = (keyValue.value).cast<Pose2>();
        vector<double> poseVec(3);

        poseVec[0] = pose.x();
        poseVec[1] = pose.y();
        poseVec[2] = pose.theta();

        pair<unsigned int, vector<double> > keyPosePair(keyValue.key, poseVec);
        posesOdom.insert(keyPosePair);
      }
      js["obj_poses_odom"] = posesOdom;

      // save ee poses with added noise
      for (const auto& keyValue : eePoseValsNoisy) {
        Pose2 pose = (keyValue.value).cast<Pose2>();
        vector<double> poseVec(3);

        poseVec[0] = pose.x();
        poseVec[1] = pose.y();
        poseVec[2] = pose.theta();

        pair<unsigned int, vector<double> > keyPosePair(keyValue.key, poseVec);
        eePosesNoisy.insert(keyPosePair);
      }
      js["ee_poses_noisy"] = eePosesNoisy;

      fileStream << js;

      // temp: to access constraint factor vars
      /* NonlinearFactorGraph constraintFactors_ = isam2.getConstraintFactorsUnsafe();
      for (size_t i = 0; i < constraintFactors_.size(); i++) {
        boost::shared_ptr<NonlinearFactor> factor = constraintFactors_.at(i);
        KeyVector varKeys = factor->keys();
        Vector2 point = factor->getPointPolyProj(poseValsGraph.at(varKeys[0]));
      } */
    }

    void saveGraph(const gtsam::ISAM2& isam2, string ssName) {
      std::ofstream ofs(ssName);
      auto fk = isam2.getFactorsUnsafe();
      std::cout << fk.size() << std::endl;
      (isam2.getFactorsUnsafe()).saveGraph(ofs, isam2.getLinearizationPoint());
    }

  };


}  // namespace incopt

#endif




// END CONSTRAINTS EDITS
