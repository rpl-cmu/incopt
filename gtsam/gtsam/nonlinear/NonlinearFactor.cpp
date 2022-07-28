/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactor.cpp
 * @brief   Nonlinear Factor base classes
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>

namespace gtsam {

/* ************************************************************************* */
void NonlinearFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << "  keys = { ";
  for(Key key: keys()) {
    std::cout << keyFormatter(key) << " ";
  }
  std::cout << "}" << std::endl;
}

/* ************************************************************************* */
bool NonlinearFactor::equals(const NonlinearFactor& f, double tol) const {
  return Base::equals(f);
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr NonlinearFactor::rekey(
    const std::map<Key, Key>& rekey_mapping) const {
  shared_ptr new_factor = clone();
  for (size_t i = 0; i < new_factor->size(); ++i) {
    Key& cur_key = new_factor->keys()[i];
    std::map<Key, Key>::const_iterator mapping = rekey_mapping.find(cur_key);
    if (mapping != rekey_mapping.end())
      cur_key = mapping->second;
  }
  return new_factor;
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr NonlinearFactor::rekey(
    const KeyVector& new_keys) const {
  assert(new_keys.size() == keys().size());
  shared_ptr new_factor = clone();
  new_factor->keys() = new_keys;
  return new_factor;
}

/* ************************************************************************* */
void NoiseModelFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  if (noiseModel_)
    noiseModel_->print("  noise model: ");
  std::cout << "  constraintFactorType_ " << constraintFactorType_ << std::endl;
}

/* ************************************************************************* */
bool NoiseModelFactor::equals(const NonlinearFactor& f, double tol) const {
  const NoiseModelFactor* e = dynamic_cast<const NoiseModelFactor*>(&f);
  return e && Base::equals(f, tol)
      && ((!noiseModel_ && !e->noiseModel_)
          || (noiseModel_ && e->noiseModel_
              && noiseModel_->equals(*e->noiseModel_, tol)));
}

/* ************************************************************************* */
static void check(const SharedNoiseModel& noiseModel, size_t m) {
  if (noiseModel && m != noiseModel->dim())
    throw std::invalid_argument(
        boost::str(
            boost::format(
                "NoiseModelFactor: NoiseModel has dimension %1% instead of %2%.")
                % noiseModel->dim() % m));
}

/* ************************************************************************* */
Vector NoiseModelFactor::whitenedError(const Values& c) const {
  const Vector b = unwhitenedError(c);
  check(noiseModel_, b.size());
  return noiseModel_ ? noiseModel_->whiten(b) : b;
}

/* ************************************************************************* */
double NoiseModelFactor::errorConstrained(const Values& c) const {
  Vector b = unwhitenedError(c);
  setSatisfiedConstraintb(&b);
  check(noiseModel_, b.size());
  if (noiseModel_)
    return noiseModel_->loss(noiseModel_->squaredMahalanobisDistance(b));
  else
    return 0.5 * b.squaredNorm();
}

/* ************************************************************************* */
double NoiseModelFactor::errorUnwhitened(const Values& c) const {
  Vector b = unwhitenedError(c);
  setSatisfiedConstraintb(&b);
  return 0.5 * b.squaredNorm();
}

/* ************************************************************************* */
Vector NoiseModelFactor::errorBConstrained(const Values& c) const {
  Vector b = unwhitenedError(c);
  setSatisfiedConstraintb(&b);
  return b;
}

/* ************************************************************************* */
Vector NoiseModelFactor::unweightedWhitenedError(const Values& c) const {
  const Vector b = unwhitenedError(c);
  check(noiseModel_, b.size());
  return noiseModel_ ? noiseModel_->unweightedWhiten(b) : b;
}

/* ************************************************************************* */
double NoiseModelFactor::weight(const Values& c) const {
  if (active(c)) {
    if (noiseModel_) {
      const Vector b = unwhitenedError(c);
      check(noiseModel_, b.size());
      return 0.5 * noiseModel_->weight(b);
    }
    else
      return 1.0;
  } else {
    return 0.0;
  }
}

/* ************************************************************************* */
double NoiseModelFactor::error(const Values& c) const {
  if (active(c)) {
    const Vector b = unwhitenedError(c);
    check(noiseModel_, b.size());
    if (noiseModel_)
      return noiseModel_->loss(noiseModel_->squaredMahalanobisDistance(b));
    else
      return 0.5 * b.squaredNorm();
  } else {
    return 0.0;
  }
}

// No need for this function. Merge with setSatisfiedConstraint
void NoiseModelFactor::setSatisfiedConstraintb(Vector* b) const {
  for(int i =0; i < b->size(); i++) {
    if((*b)[i] < 0 && this->isConstraintFactor_ && this->constraintFactorType_ == 2) 
    { // i.e if constraints are satisfied
      (*b)[i] = 0 ;
    }
    else if((*b)[i] > 0 && this->isConstraintFactor_ && this->constraintFactorType_ == 3) 
    { // i.e if constraints are satisfied
      (*b)[i] = 0 ;
    }
  }
}

/*  Here we set the inequality constraint augmentation term according to:
    max(H \delta x + h(x0) , 0 ) 
    Since \delta x in ADMM Loop is initialized to the zero vector. Then the condition becomes:
    max(h(x0), 0) 
    where h(x0) = b */
void NoiseModelFactor::setSatisfiedConstraint(Vector* b, std::vector<Matrix>* A) const {
  for(int i =0; i < b->size(); i++) {
    if((*b)[i] < 0 && this->isConstraintFactor_ && this->constraintFactorType_ == 2) { // aka if constraints are satisfied
      (*b)[i] = 0 ;
      for(int j =0; j < A->size(); j++) {
        (*A)[j].row(i).setZero();
      }
    }
    else if((*b)[i] > 0 && this->isConstraintFactor_ && this->constraintFactorType_ == 3) { // aka if constraints are satisfied
      (*b)[i] = 0 ;
      for(int j =0; j < A->size(); j++) {
        (*A)[j].row(i).setZero();
      }
    }
  }
}


void NoiseModelFactor::rhoScaling(Vector* b, std::vector<Matrix>* A) const {
  FastVector<double> rho_FV = rho_;
  Eigen::Map<Vector> rho_V(rho_FV.data(), rho_FV.size());
  rho_V = (rho_V/2).cwiseSqrt();
  auto diagRho = (rho_V/2).cwiseSqrt().asDiagonal();
  for(int j =0; j < A->size(); j++) {
    (*A)[j] = diagRho*(*A)[j];
  }
  *b = diagRho*(*b);
}


/* ************************************************************************* */
boost::shared_ptr<GaussianFactor> NoiseModelFactor::linearize(
    const Values& x) const {

  // Only linearize if the factor is active
  if (!active(x))
    return boost::shared_ptr<JacobianFactor>();

  // Call evaluate error to get Jacobians and RHS vector b
  std::vector<Matrix> A(size());
  Vector b;
  if(this->isConstraintFactor_) {
    b = unwhitenedError(x, A);
    if(this->constraintFactorType_ > 1) {
      setSatisfiedConstraint(&b, &A);
    }
  }
  else {
    b = -unwhitenedError(x, A);
  }
  check(noiseModel_, b.size());

  // Whiten the corresponding system now
  if (noiseModel_ && !this->isConstraintFactor_)
    noiseModel_->WhitenSystem(A, b);

  if(this->isConstraintFactor_) {
    rhoScaling(&b, &A);
  }
  // Fill in terms, needed to create JacobianFactor below
  std::vector<std::pair<Key, Matrix> > terms(size());
  for (size_t j = 0; j < size(); ++j) {
    terms[j].first = keys()[j];
    terms[j].second.swap(A[j]);
  }

  // TODO pass unwhitened + noise model to Gaussian factor
  using noiseModel::Constrained;
  if (noiseModel_ && noiseModel_->isConstrained()) {
    GaussianFactor::shared_ptr gf(new JacobianFactor(terms, b,boost::static_pointer_cast<Constrained>(noiseModel_)->unit()));
    if (this->isConstraintFactor_) {
      gf->setIsConstraintFactor(1);
      gf->setConstraintFactorType(this->constraintFactorType_);
      gf->setdualV(this->dualV_);
      gf->setRho(this->rho_);

    }
    return gf;

  }
  else {
    GaussianFactor::shared_ptr gf(new JacobianFactor(terms, b));
    if (this->isConstraintFactor_) {
      gf->setIsConstraintFactor(1);
      gf->setConstraintFactorType(this->constraintFactorType_);
      gf->setdualV(this->dualV_);
      gf->setRho(this->rho_);
    }
    return gf;
   }
}

/* ************************************************************************* */

} // \namespace gtsam
