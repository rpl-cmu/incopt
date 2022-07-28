
#ifndef INCOPT_H
#define INCOPT_H

#include<gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <boost/optional/optional_io.hpp>


namespace incopt {

class UnaryPose3Factor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  gtsam::Pose3 meas_;
  
 public:
   typedef gtsam::OptionalJacobian<6, 6> ChartJacobian; 
   gtsam::LieGroup<gtsam::Pose3,6> Pose3LieGroup;

   void example() {
      std::cout << "an example " << std::endl;
   }
   
   UnaryPose3Factor(gtsam::Key varKey, const gtsam::Pose3 &meas, const int constraintFactorType, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     // 0: none, 1: equality, 2: inequality less than, 3: inequality greater than 
     constraintFactorType_ = constraintFactorType;
     isConstraintFactor_ = constraintFactorType > 0;
   }


  gtsam::Vector evaluateError(const gtsam::Pose3& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // compute jacobian
    gtsam::Vector error =  meas_.localCoordinates(x, boost::none, H ? H : boost::none);
    //if(H) std::cout << H << std::endl;
    return error;
  }
};


class UnaryPose2Factor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
 private:
  gtsam::Pose2 meas_;
  
 public:
   typedef gtsam::OptionalJacobian<3, 3> ChartJacobian; 
   gtsam::LieGroup<gtsam::Pose2,3> Pose2LieGroup;

   void example() {
      std::cout << "an example " << std::endl;
   }
   
   UnaryPose2Factor(gtsam::Key varKey, const gtsam::Pose2 &meas, const int constraintFactorType, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     // 0: none, 1: equality, 2: inequality
     constraintFactorType_ = constraintFactorType;
     isConstraintFactor_ = constraintFactorType > 0;
   }


  gtsam::Vector evaluateError(const gtsam::Pose2& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // compute jacobian
    //std::cout<< " ============= meas_ ================ " << std::endl;
    //meas_.print("\n");
    //std::cout<< "=============== x=================" << std::endl;
    //x.print("\n");

    gtsam::Vector error =  meas_.localCoordinates(x, boost::none, H ? H : boost::none);
    //if(H) std::cout << H << std::endl;

    //std::cout << std::endl;
    return error;
  }
};


class QuadraticUnaryFactor1D : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 private:
  gtsam::Vector meas_;

 public:

   void example() {
      std::cout << "an example " << std::endl;
   }
   
   QuadraticUnaryFactor1D(gtsam::Key varKey, const gtsam::Vector1 &meas, const int constraintFactorType, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     // 0: none, 1: equality, 2: inequality
     constraintFactorType_ = constraintFactorType;
     isConstraintFactor_ = constraintFactorType > 0;
   }

  void evaluateErrorCustom() {
    std::cout << "return here " << std::endl;
  }

  gtsam::Vector evaluateError(const gtsam::Vector& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    
    // compute jacobian
    gtsam::Matrix jac(1,1);
    jac << 1.;
    if (H) *H = jac;

    // compute error
    gtsam::Vector errorVector(1);
    std::cout << "Inside EvaluateError" << std::endl;
    std::cout << x << std::endl;
    std::cout << meas_ << std::endl;
    errorVector << (x - meas_);

    return errorVector;
  }

  gtsam::Vector constraintError(const gtsam::Value& val) const override {
    gtsam::Vector x = val.cast<gtsam::Vector>();
    gtsam::Vector errorVector(1);
    std::cout << "Inside constraintError" << std::endl;
    std::cout << x << std::endl;
    std::cout << meas_ << std::endl;
    errorVector <<  (x - meas_);

    return errorVector;
  }

  gtsam::Matrix constraintJacobian(const gtsam::Value& val) const override  {
    
    gtsam::Vector x = val.cast<gtsam::Vector>();

    gtsam::Matrix jac(1,1);
    jac << 1.;
    gtsam::Matrix G = jac;

    return G;
  }

};


class QuadraticUnaryFactor1DHinge : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 private:
  gtsam::Vector1 meas_;
  // 0 less, 1 greater
  bool lessOrGreater_; 

 public:

   QuadraticUnaryFactor1DHinge(gtsam::Key varKey, const gtsam::Vector1 &meas, const bool lessOrGreater, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     lessOrGreater_ = lessOrGreater;
   }

  gtsam::Vector1 hingeError(const gtsam::Vector1& x) const {
    double ret; 
    if(lessOrGreater_ == 0) {
      if(x[0] - meas_[0]  <= 0) ret = 0;
      else ret = (x[0]-meas_[0])*(x[0]-meas_[0]);
    }
    else if(lessOrGreater_ == 1) {
      if(x[0] - meas_[0]  >= 0) ret = 0;
      else ret = (x[0]-meas_[0])*(x[0]-meas_[0]);
    }
    //std::cout << "x0" << x[0] << std::endl;
    //std::cout << "meas_" << meas_[0] << std::endl;
    gtsam::Vector1 errorVector ;
    errorVector << ret;
    return errorVector;
  }


  gtsam::Vector evaluateError(const gtsam::Vector& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    gtsam::Vector1 err = hingeError(x);
    // compute jacobian
    if(H) {
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector1>(boost::bind(&QuadraticUnaryFactor1DHinge::hingeError, this, _1), x);
      std::cout << *H << std::endl;
    }
    // compute error
    std::cout << err << std::endl;
    
    return err;
  }

};


class QuadraticUnaryFactorVector2ConstrainedHinge : public gtsam::NoiseModelFactor1<gtsam::Vector2> {
 private:
  gtsam::Vector2 meas_;
  // 0 less, 1 greater
  bool lessOrGreater_; 

 public:

   QuadraticUnaryFactorVector2ConstrainedHinge(gtsam::Key varKey, const gtsam::Vector2 &meas, 
                                             const int constraintFactorType,
                                             const gtsam::SharedNoiseModel &model = nullptr)
                                            : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     constraintFactorType_ = constraintFactorType;
     isConstraintFactor_ = constraintFactorType > 0;
   }


  gtsam::Vector hingeError(const gtsam::Vector2& x) const {
    gtsam::Vector2 errorVector ;
    errorVector << x(0) - meas_(0), x(1) - meas_(1);
    return errorVector;
  }

  gtsam::Vector evaluateError(const gtsam::Vector2& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    gtsam::Vector2 err = hingeError(x);

    // compute jacobian
    if(H) {
      //*H = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose2>(boost::bind(&QuadraticUnaryFactorPose2ConstrainedHinge::hingeError, this, _1), x);
      //std::cout << x.x() - meas_.x()  << std::endl;
      //std::cout << x.y() - meas_.y() << std::endl;
      //std::cout << *H << std::endl;
      gtsam::Matrix22 U;
      U << 1, 0, 
           0, 1;
      *H = U;
    }
    // compute error
    //std::cout << err << std::endl;
    return err;
  }

};


class QuadraticUnaryFactorVector2Hinge : public gtsam::NoiseModelFactor1<gtsam::Vector2> {
 private:
  gtsam::Vector2 meas_;
  // 0 less, 1 greater
  bool lessOrGreater_; 

 public:

   QuadraticUnaryFactorVector2Hinge(gtsam::Key varKey, const gtsam::Vector2 &meas, 
                               bool lessOrGreater, 
                               const gtsam::SharedNoiseModel &model = nullptr)
                              : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     lessOrGreater_ = lessOrGreater;
   }


  gtsam::Vector hingeError(const gtsam::Vector2& x) const {
    double err_x = 0; 
    double err_y = 0;
    if(lessOrGreater_ == 0) {
      if (x(0) - meas_(0)  <= 0) err_x = 0;
      else err_x = x(0) - meas_(0);
      if (x(1) - meas_(1)  <= 0) err_y = 0;
      else err_y = x(1) - meas_(1);
    }
    else if(lessOrGreater_ == 1) {
      if (x(0) - meas_(0)  >= 0) err_x = 0;
      else err_x = meas_(0) - x(0);
      if (x(1) - meas_(1)  >= 0) err_y = 0;
      else err_y = meas_(1) - x(1); 
    }
    // std::cout << "++++++++++++++++++++++++++++++++++++++++=" << std::endl;
    // std::cout << "meas_ " << std::endl;
    // std::cout << meas_ << std::endl;
    // std::cout << "x" << std::endl;
    // std::cout << x << std::endl;
    // std::cout << "lessOrGreater_" << std::endl;
    // std::cout << lessOrGreater_ << std::endl;
    gtsam::Vector2 errorVector ;
    errorVector << err_x, err_y;
    // std::cout << "errorVector " << std::endl;
    // std::cout << errorVector << std::endl;
    return errorVector;
  }

  gtsam::Vector evaluateError(const gtsam::Vector2& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    gtsam::Vector2 err = hingeError(x);
    // compute jacobian
    if(H) {
      *H = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector2>(boost::bind(&QuadraticUnaryFactorVector2Hinge::hingeError, this, _1), x);
      //std::cout << *H << std::endl;
    }
    // compute error
    //std::cout << err << std::endl;
    
    return err;
  }

};


class BinaryFactorPointRobot : public gtsam::NoiseModelFactor2<gtsam::Vector2, gtsam::Vector2> {
 private:
  gtsam::Vector2 meas_;

 public:

   BinaryFactorPointRobot(gtsam::Key varKey1, gtsam::Key varKey2, const gtsam::Vector2 &meas, 
                               const gtsam::SharedNoiseModel &model = nullptr)
                              : NoiseModelFactor2(model, varKey1, varKey2), meas_(meas)
   {

   }


  gtsam::Vector error(const gtsam::Vector2& x1, const gtsam::Vector2& x2) const {
    gtsam::Vector2 errorVector ;
    // std::cout << "x2  " << x2 << std::endl;
    // std::cout << "x1   " << x1 << std::endl;
    // std::cout << "meas_   " << meas_ << std::endl;
    errorVector << x2 - x1 - meas_;

    return errorVector;
  }

  gtsam::Vector evaluateError(const gtsam::Vector2& x1, 
                              const gtsam::Vector2& x2, 
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none) const override {

    gtsam::Vector2 err = error(x1, x2);
    // compute jacobian
    if(H1) {
      *H1 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector2>(boost::bind(&BinaryFactorPointRobot::error, this, _1, x2), x1);
      // std::cout << err << std::endl;
      // std::cout << "*********" << std::endl;
      // std::cout << *H1 << std::endl;
    }
    if(H2) {
      *H2 = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector2>(boost::bind(&BinaryFactorPointRobot::error, this, x1, _1), x2);
      // std::cout << "*********" << std::endl;
      // std::cout << *H2 << std::endl;
    }
    
    return err;
  }

};


class UnaryVector2Factor : public gtsam::NoiseModelFactor1<gtsam::Vector2> {
 private:
  gtsam::Vector2 meas_;
  
 public:   
   UnaryVector2Factor(gtsam::Key varKey, const gtsam::Vector2 &meas, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     // 0: none, 1: equality, 2: inequality

   }

  gtsam::Vector error(const gtsam::Vector2& x) const {
    gtsam::Vector2 errorVector ;
    errorVector << x - meas_;
    return errorVector;
  }

  gtsam::Vector evaluateError(const gtsam::Vector2& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {    
    gtsam::Vector2 err = error(x);
    if(H) {
      //gtsam::Matrix22 U; 
      //U << 1, 0,
      //     0, 1;
      //*H = U;
      *H = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector2>(boost::bind(&UnaryVector2Factor::error, this, _1), x);
    }

    return err;
  }
};


class QuadraticBinaryFactor1D : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
 private:
  gtsam::Vector meas_;

 public:
   QuadraticBinaryFactor1D(gtsam::Key varKey1, gtsam::Key varKey2, const gtsam::Vector1 &meas, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor2(model, varKey1, varKey2), meas_(meas)
   {}

   gtsam::Vector evaluateError(const gtsam::Vector &x1, const gtsam::Vector &x2, boost::optional<gtsam::Matrix &> H1 = boost::none, boost::optional<gtsam::Matrix &> H2 = boost::none) const override
   {
       // compute jacobian
       if (H1) *H1 = (gtsam::Matrix11() << -1.).finished();
       if (H2) *H2 = (gtsam::Matrix11() << 1.).finished();

    // compute error
    gtsam::Vector errorVector(1);
    errorVector << ((x2 - x1) - meas_);
    
    return errorVector;
   }
};

}  // namespace incopt

#endif 

