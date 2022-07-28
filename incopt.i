// ics variables


#include <cpp/incopt.h>
#include <pybind11/stl.h>

namespace incopt {
  
  #include <cpp/thirdparty/gpmp2/kinematics/ForwardKinematics.h>
  template<POSE = {gtsam::Vector}, VELOCITY={gtsam::Vector}>
  class ForwardKinematics {
      size_t dof() const;
      size_t nr_links() const;
      void forwardKinematics(const POSE& jp, 
                             boost::optional<const Vector&> jv,
                             std::vector<gtsam::Pose3>& jpx, 
                             boost::optional<std::vector<gtsam::Vector3>&> jvx) ;
    gtsam::Matrix forwardKinematicsPose(const POSE& jp) const;
    gtsam::Matrix forwardKinematicsPosition(const POSE& jp) const;
    gtsam::Matrix forwardKinematicsVel(const POSE& jp, const VELOCITY& jv) const;
  };

#include <cpp/thirdparty/gpmp2/kinematics/Arm.h>
  class Arm:  incopt::ForwardKinematics<gtsam::Vector, gtsam::Vector>  {
    //void example();
    Arm(size_t dof, gtsam::Vector a, gtsam::Vector alpha, gtsam::Vector d);
    Arm(size_t dof, gtsam::Vector a, gtsam::Vector alpha, gtsam::Vector d, const gtsam::Pose3& base_pose);
    Arm(size_t dof, gtsam::Vector a, gtsam::Vector alpha, gtsam::Vector d, const gtsam::Pose3& base_pose, gtsam::Vector theta_bias);
    // full forward kinematics
    gtsam::Matrix forwardKinematicsPose(Vector jp) const;
    gtsam::Matrix forwardKinematicsPosition(Vector jp) const;
    gtsam::Matrix forwardKinematicsVel(Vector jp, Vector jv) const;
    // // accesses
    size_t dof() const;
    Vector a() const;
    Vector d() const;
    Vector alpha() const;
    gtsam::Pose3 base_pose() const;
  };

  #include <cpp/thirdparty/gpmp2/kinematics/RobotModel.h>
  class BodySphere {
     BodySphere(size_t id, double r, const gtsam::Point3& c);
  };

  class BodySphereVector {
    BodySphereVector();
    void push_back(const incopt::BodySphere& sphere);
  };

  template <FK={incopt::Arm}>
  class RobotModel {
      RobotModel();
      RobotModel(const FK& fk_model, const vector<incopt::BodySphere>& body_spheres);
      void sphereCenters(const FK::Pose& jp, std::vector<gtsam::Point3>& sph_centers) const ;
      gtsam::Point3 sphereCenter(size_t sph_idx, const FK::Pose& jp) const ;
      gtsam::Matrix sphereCentersMat(const FK::Pose& jp) const ;
      const FK& fk_model() const;
      size_t nr_body_spheres() const;
      size_t sphere_link_id(size_t i) const;
      double sphere_radius(size_t i) const;
      const gtsam::Point3& sphere_center_wrt_link(size_t i) const;
  };

  #include <cpp/thirdparty/gpmp2/obstacle/PlanarSDF.h>
  class PlanarSDF {
    PlanarSDF();
    PlanarSDF(const gtsam::Point2& origin, double cell_size, const gtsam::Matrix& data) ;
    double getSignedDistance(const gtsam::Point2& point) const;
    double getSignedDistance(const gtsam::Point2& point, gtsam::Vector2& g) ;
    boost::tuple<double, double> convertPoint2toCell(const gtsam::Point2& point) const;
    gtsam::Point2 convertCelltoPoint2(const boost::tuple<double, double>& cell) const ;
    double signed_distance(const boost::tuple<double, double>& idx) const;
    gtsam::Vector2 gradient(const boost::tuple<double, double>& idx) const;
    double signed_distance(size_t r, size_t c) const ;
    gtsam::Point2& origin() const;
    size_t x_count() const;
    size_t y_count() const;
    double cell_size() const;
    const gtsam::Matrix& raw_data() const;
  };

  #include <cpp/thirdparty/gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
  template <ROBOT={incopt::RobotModel<incopt::Arm>}>
  class ObstaclePlanarSDFFactor: gtsam::NoiseModelFactor {
      ObstaclePlanarSDFFactor();
      ObstaclePlanarSDFFactor(gtsam::Key poseKey, const ROBOT& robot,
                              const incopt::PlanarSDF& sdf, double cost_sigma, double epsilon);

      // NOTE THAT WE DIRECTLY DECLARE ROBOT::Pose as a Vector here. Take another look if a different datatype is needed.              
      gtsam::Vector evaluateError(Vector& conf) const;
      gtsam::NonlinearFactor::shared_ptr clone() const;
  };

  // obstacle avoid factor
  #include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactorArm.h>
  virtual class ObstacleSDFFactorArm : gtsam::NoiseModelFactor {
    ObstacleSDFFactorArm(
        size_t poseKey, const incopt::ArmModel& arm,
        const incopt::SignedDistanceField& sdf, double cost_sigma, double epsilon);
    Vector evaluateError(Vector pose) const;
  };


  // obstacle avoid factor with GP interpolation
  #include <cpp/thirdparty/gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
  virtual class ObstacleSDFFactorGPArm : gtsam::NoiseModelFactor {
    ObstacleSDFFactorGPArm(
        size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
        const incopt::ArmModel& arm, const incopt::SignedDistanceField& sdf,
        double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
        double delta_t, double tau);
  };

  // signed distance field class
  #include <cpp/thirdparty/gpmp2/obstacle/SignedDistanceField.h>
  class SignedDistanceField {
    SignedDistanceField();
    SignedDistanceField(const gtsam::Point3& origin, double cell_size, size_t field_rows,
        size_t field_cols, size_t field_z);
    // insert field data
    void initFieldData(size_t z_idx, const Matrix& field_layer);
    // access
    double getSignedDistance(const gtsam::Point3& point) const;
    // void print(string s) const;
    void saveSDF(string filename);
    void loadSDF(string filename);
  };

  #include <cpp/thirdparty/gpmp2/gp/GaussianProcessPriorLinear.h>
  class GaussianProcessPriorLinear: gtsam::NoiseModelFactor {
      GaussianProcessPriorLinear();
      GaussianProcessPriorLinear(gtsam::Key poseKey1, gtsam::Key velKey1,
            gtsam::Key poseKey2, gtsam::Key velKey2,
            double delta_t, const gtsam::SharedNoiseModel Qc_model);
      gtsam::NonlinearFactor::shared_ptr clone() const;
      gtsam::Vector evaluateError(
            const gtsam::Vector& pose1, const gtsam::Vector& vel1,
            const gtsam::Vector& pose2, const gtsam::Vector& vel2);
      size_t dim() const;
      size_t size() const;
      bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const;
  };
  

  #include <cpp/thirdparty/gpmp2/planner/TrajUtils.h>

  // /// initialization
  gtsam::Values initArmTrajStraightLine(const gtsam::Vector& init_conf, const gtsam::Vector& end_conf, size_t total_step);
  // gtsam::Values initPose2VectorTrajStraightLine(const gtsam::Pose2& init_pose, Vector init_conf,
  //     const gtsam::Pose2& end_pose, Vector end_conf, size_t total_step);
  // gtsam::Values initPose2TrajStraightLine(const gtsam::Pose2& init_pose,
  //     const gtsam::Pose2& end_pose, size_t total_step);

  // /// robot arm trajectory interpolator
  gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
      const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step);

  // /// robot arm trajectory interpolator between any two states
  // gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
  //     const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step, 
  //     size_t start_index, size_t end_index);

  // /// mobile arm trajectory interpolator between any two states
  // gtsam::Values interpolatePose2MobileArmTraj(const gtsam::Values& opt_values,
  //     const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step, 
  //     size_t start_index, size_t end_index);

  // /// mobile base trajectory interpolator between any two states
  // gtsam::Values interpolatePose2Traj(const gtsam::Values& opt_values,
  //     const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step, 
  //     size_t start_index, size_t end_index);


  #include <cpp/pushing/constraints/ContactFactors.h>
  class ContactMeasurement {
        ContactMeasurement(const gtsam::Vector2 contactPoint__world, const gtsam::Vector2 eeCenter__world,
                     const gtsam::Matrix objPolyShape, const double eeRadius);
        void write(std::ostream& out) const ;

  };

  class ContactPush2dUnaryFactor: gtsam::NoiseModelFactor {
      ContactPush2dUnaryFactor(gtsam::Key poseKey, incopt::ContactMeasurement contactMeas, const gtsam::SharedNoiseModel& model);
      Vector evaluateError(const Pose2& p) const;
      Vector constraintError(const Value& val) const;
      Matrix constraintJacobian(const Value& val) const;
      Vector2 getPointPolyProj(const Pose2& p);

  };

  class contactConstraintUtils {
     contactConstraintUtils();
     void writeOutputsJson(const gtsam::ISAM2& isam2, const Values& poseValsGraph,
                              const Values& poseValsGT, const Values& poseValsOdom,
                              const Values& eePoseValsNoisy, const string& filename); 
     void saveGraph(const gtsam::ISAM2& isam2, string ssName);                                
  };


  class QuadraticUnaryFactor1D: gtsam::NoiseModelFactor {
    void example();
    QuadraticUnaryFactor1D(gtsam::Key varKey, const gtsam::Vector1 &meas, const int constraintFactorType, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector constraintError(const gtsam::Value& val);
    gtsam::Matrix constraintJacobian(const gtsam::Value& val);
    gtsam::Vector evaluateError(const gtsam::Vector& x);
    void evaluateErrorCustom();
  };

  class QuadraticBinaryFactor1D: gtsam::NoiseModelFactor {
    QuadraticBinaryFactor1D(gtsam::Key varKey1, gtsam::Key varKey2, const gtsam::Vector1 &meas, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector evaluateError(const gtsam::Vector &x1, const gtsam::Vector &x2);
  };

  class UnaryPose2Factor: gtsam::NoiseModelFactor {
    UnaryPose2Factor(gtsam::Key varKey, const gtsam::Pose2 &meas, const int constraintFactorType, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector evaluateError(const gtsam::Pose2& x);
  };

  class QuadraticUnaryFactor1DHinge: gtsam::NoiseModelFactor {
    QuadraticUnaryFactor1DHinge(gtsam::Key varKey, const gtsam::Vector1 &meas, const bool lessOrGreater, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector evaluateError(const gtsam::Vector& x);
  };

  class QuadraticUnaryFactorVector2Hinge: gtsam::NoiseModelFactor {
    QuadraticUnaryFactorVector2Hinge(gtsam::Key varKey, const gtsam::Vector2 &meas, const bool lessOrGreater, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector evaluateError(const gtsam::Vector2& x);
  };


  class QuadraticUnaryFactorVector2ConstrainedHinge: gtsam::NoiseModelFactor {
    QuadraticUnaryFactorVector2ConstrainedHinge(gtsam::Key varKey, const gtsam::Vector2 &meas, const bool lessOrGreater, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector evaluateError(const gtsam::Vector2& x);
  };

  class BinaryFactorPointRobot : gtsam::NoiseModelFactor {
    BinaryFactorPointRobot(gtsam::Key varKey1, gtsam::Key varKey2, const gtsam::Vector2 &meas, 
                           const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector evaluateError(const gtsam::Vector2& x1, const gtsam::Vector2& x2); 
  };

  class UnaryVector2Factor : gtsam::NoiseModelFactor {
     UnaryVector2Factor(gtsam::Key varKey, const gtsam::Vector2 &meas, const gtsam::SharedNoiseModel &model = nullptr);
     gtsam::Vector evaluateError(const gtsam::Vector2& x);
  };
}
// namespace incopt
