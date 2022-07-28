/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2Clique.cpp
 * @brief   Specialized iSAM2 Clique
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

#include <gtsam/inference/BayesTreeCliqueBase-inst.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearAlgorithms-inst.h>
#include <gtsam/nonlinear/ISAM2Clique.h>

#include <stack>
#include <utility>

using namespace std;

namespace gtsam {

// Instantiate base class
template class BayesTreeCliqueBase<ISAM2Clique, GaussianFactorGraph>;

/* ************************************************************************* */
void ISAM2Clique::setEliminationResult(
    const FactorGraphType::EliminationResult& eliminationResult) {
  conditional_ = eliminationResult.first;
  cachedFactor_ = eliminationResult.second;
  // Compute gradient contribution
  gradientContribution_.resize(conditional_->cols() - 1);
  // Rewrite -(R * P')'*d   as   -(d' * R * P')'   for computational speed
  // reasons
  gradientContribution_ << -conditional_->R().transpose() *
                               conditional_->d(),
      -conditional_->S().transpose() * conditional_->d();
}

Vector updateStepSizeRho(const Vector& rhoVec, const Vector& constraintVals,
                                      const Vector& constraintValsPrev, int factorType,
                                        const std::map<std::string, double>& constraintPenaltyParams_) {

  double growth = constraintPenaltyParams_.at("growth"); // 5.0;
  double reduction = constraintPenaltyParams_.at("reduction"); // 0.2
  double maxRhoVal = constraintPenaltyParams_.at("maxRhoVal"); // 1e3
  double minRhoVal = constraintPenaltyParams_.at("minRhoVal"); // 1e3
  double minChange = constraintPenaltyParams_.at("minChange"); // 0.5 
  
  double w = 0.0;  // for momentum term

  Vector rhoVecNew = rhoVec;

  for (int i = 0; i < constraintVals.size(); i++) {
      if (factorType > 1) {
      // i. inequality constraint satisfied by curr step
          if (constraintVals[i] < 0) {
              rhoVecNew[i] = std::max(rhoVec[i] * reduction + w * rhoVec[i], minRhoVal);
              continue;
          }
      // ii. inequality constraint satisfaction changed from prev to curr step
          if ((constraintVals[i] > 0) && (constraintValsPrev[i] < 0)) {
              rhoVecNew[i] = std::min(rhoVec[i] * growth + w * rhoVec[i], maxRhoVal);
              continue;
          }
      }

      // for both equality, inequality constraints
      if (std::abs(constraintVals[i]) > std::abs(constraintValsPrev[i]) * minChange) {
          rhoVecNew[i] = std::min(rhoVec[i] * growth + w * rhoVec[i], maxRhoVal);
      }
  }
  
  return rhoVecNew;
} 


Vector vectorFromFastMap(FastVector<double>* fastMap) {
  Eigen::Map<Vector> v(fastMap->data(), fastMap->size());
  return v;
}

FastVector<double> fastMapFromVector(Vector v) {
  FastVector<double> blockSV(v.data(), v.data() + v.rows() * v.cols());
  return blockSV;
}

/* ************************************************************************* */
void ISAM2Clique::updateMultiplierTerms(map<int, tuple<FastVector<double>*, FastVector<double>*, FastVector<double>*>>& cliqueMultiplierUpdate,
                                        const VectorValues* delta,
                                        int iter, double* residualPrimal,
                                        const std::map<std::string, double>& constraintPenaltyParams_) {
  for(auto it = this->GMap_.begin(); it != this->GMap_.end(); it++) {
    int factor_index = it->first;
    if(!it->second) continue;
    Gtype parts = *it->second;
    Vector T;
    Vector g;
    int factorType = 1;
    for(auto& v: parts) {
      Key k = std::get<0>(v);
      Matrix G = std::get<1>(v);
      g = std::get<2>(v);
      factorType = std::get<3>(v);

      if(T.size() == 0 ) T = G*delta->at(k);
      else T += G*delta->at(k);
    }
    Vector constraintVals = T + g;
    FastVector<double>* dualV_ = get<0>(cliqueMultiplierUpdate.at(factor_index));
    FastVector<double>* rho_ = get<1>(cliqueMultiplierUpdate.at(factor_index));
    // This is the "prev" constraintViolation of factor.at(factor_index)
    FastVector<double>* constraintViolation_ = get<2>(cliqueMultiplierUpdate.at(factor_index));


    Vector rho_V = vectorFromFastMap(rho_);
    Vector dualV_V = vectorFromFastMap(dualV_);
    Vector constraintViolation_prev = vectorFromFastMap(constraintViolation_);

    if(iter == 0) {
      constraintViolation_prev = g;
    }
    

    Vector dualV = dualV_V + rho_V.cwiseProduct(constraintVals);
    if(factorType == 3 || factorType == 2) {
      dualV = (0.0 < dualV.array()).select(dualV, 0.0);        
    }
    *residualPrimal += constraintVals.array().square().sum();

    *dualV_ = fastMapFromVector(dualV);
    *constraintViolation_ = fastMapFromVector(constraintVals);
    Vector constraintViolation_prev_out = vectorFromFastMap(constraintViolation_);

    Vector newRho = updateStepSizeRho(rho_V, constraintVals, constraintViolation_prev, factorType, constraintPenaltyParams_);
    
    *rho_ = fastMapFromVector(newRho);

  }
}

/* ************************************************************************* */
void ISAM2Clique::updateMap(FastMap<Key, boost::optional<Mtype>>* T_Map,  Key key, int factorIndex, FastVector<int>* cols_T, 
                            Matrix& GSub, Vector& gSub, FastVector<int>* IndicesSeen_T ) 
  {
      if(T_Map->at(key)) {
          T_Map->at(key)->push_back(make_tuple(GSub, gSub, factorIndex));
      }
      else {
          Mtype t{make_tuple(GSub, gSub, factorIndex)};
          T_Map->at(key) = t;
      }
      // IndicesSeen_T contains the indices of the corresponding lagrange multiplier (= indices of non linear factors)
      // cols_T contains the start and end cols corresponding to each lagrange multiplier
      if(find(IndicesSeen_T->begin(), IndicesSeen_T->end(), factorIndex) == IndicesSeen_T->end()) {
          IndicesSeen_T->push_back(factorIndex);
          cols_T->push_back(cols_T->back() + GSub.cols());
      }                
  }

void ISAM2Clique::updateGMap(int factorIndex, Key key, Matrix GSub, Vector gSub, int factorType,
                             FastMap<int, boost::optional<Gtype>>* GMap_) {
  if(GMap_->find(factorIndex) == GMap_->end()) (*GMap_)[factorIndex];
  if(GMap_->at(factorIndex)) {
    GMap_->at(factorIndex)->push_back(make_tuple(key, GSub, gSub, factorType));
  }
  else {
    Gtype t{make_tuple(key, GSub, gSub, factorType)};
    GMap_->at(factorIndex) = t;
  }
}

/* ************************************************************************* */
void ISAM2Clique::_merge(FastMap<Key, boost::optional<Mtype>>* FrontalMap, 
                         FastMap<Key, boost::optional<Mtype>>* SeparatorMap,
                         FastMap<Key, boost::optional<Mtype>>& childSeparatorMap,
                         FastVector<int>* IndicesSeenFrontal_,  FastVector<int>* columnsFrontals_,
                         FastVector<int>* IndicesSeenSeparator_,  FastVector<int>* columnsSeparator_) {
  // i.e there are no separator gradient coming from children
  if(childSeparatorMap.size() == 0) {
    return;
  }
  // Loop over all keys in child seperators 
  for(auto it = childSeparatorMap.begin(); it != childSeparatorMap.end(); it++) {
    auto key = it->first; 

    if(!it->second) {
      continue;
    }
    // For each key , loop over all propagated gradients 
    for(auto itt = it->second->begin(); itt != it->second->end(); itt++) {
      int factorIndex = get<2>(*itt);
      Matrix GSub = get<0>(*itt);
      Vector gSub = get<1>(*itt);
      
      // Any additional keys coming from children
      // i.e if the key propagated upward is a frontal variable of this clique, include it in the frontalMap.
      if (FrontalMap->exists(key)) {
        if(FrontalMap->at(key)) {
            FrontalMap->at(key)->push_back(*itt);
        }
        else {
            Mtype t{make_tuple(GSub, gSub, factorIndex)};
            FrontalMap->at(key) = t;
        }
        if(find(IndicesSeenFrontal_->begin(), IndicesSeenFrontal_->end(), factorIndex) == IndicesSeenFrontal_->end()) {
            IndicesSeenFrontal_->push_back(factorIndex);
            columnsFrontals_->push_back(columnsFrontals_->back() + GSub.cols());
        }
      }
      // Otherwise, this key belong to a frontal variable further up the tree. So add it to the SeparatorMap of this clique 
      // which will accessed by the parent
      else {
        if(SeparatorMap->at(key)) {
            SeparatorMap->at(key)->push_back(*itt);
        }
        else {
            Mtype t{make_tuple(GSub, gSub, factorIndex)};
            SeparatorMap->at(key) = t;
        }
       
      }
    }
  }
}

void rhoUnscale(Vector* b, Matrix* A, Vector rho) {
  Vector rho_V_inv = (2*(rho.cwiseInverse())).cwiseSqrt();
  auto diagRho = rho_V_inv.asDiagonal();
  *A = diagRho*(*A);
  *b = diagRho*(*b);
}


void ISAM2Clique::getVectors() {
  for (auto f = this->conditional_->beginFrontals(); f != this->conditional_->endFrontals(); f++) {
      FrontalMap_[*f];
  }
  for (auto s = this->conditional_->beginParents() ; s != this->conditional_->endParents(); s++) {
      SeparatorMap_[*s];
      childContribution_[*s];
  }
  auto fc = this->constraintFactors_;
  FastVector<int> factorIndices = this->factorIndices_;
  for(size_t i=0; i< fc.size(); i++) {
      boost::shared_ptr<JacobianFactor> jacFactor = fc[i];

      int factorIndex = factorIndices[i];
      int factor_type = jacFactor->constraintFactorType();
      vector<Matrix> tmpG;
      vector<Vector> tmpg;
      
      for (auto key = jacFactor->keys().begin(); key != jacFactor->keys().end(); ++key) {
          // We have constraint in the form:
          // g(x) - A = 0 or h(x) - B <= 0 where A and B are passed as "measurements" inside factors
          // Hence this is in the same form as in the term h(x) - z which is typically
          // linearized into \del h \delx \delta x + h(x0) - z 
          Matrix GSub;
          Vector gSub;
          Matrix GSub_T;
          if(factor_type == 3) {
              GSub = -jacFactor->getA(key);
              gSub = -jacFactor->getb();
          }
          else{
              GSub = jacFactor->getA(key);
              gSub = jacFactor->getb();
          }
          FastVector<double>* rho = jacFactor->getRho();
          Vector rho_V = vectorFromFastMap(rho);

          rhoUnscale(&gSub, &GSub, rho_V);
          GSub_T = GSub.transpose();         
          this->updateGMap(factorIndex, *key, GSub, gSub, factor_type, &this->GMap_);
          if(FrontalMap_.find(*key) != FrontalMap_.end()) {
              this->updateMap(&FrontalMap_, *key, factorIndex,  &columnsFrontals_, GSub_T, gSub, &IndicesSeenFrontal_);
          }
          else {
              this->updateMap(&SeparatorMap_, *key, factorIndex, &columnsSeparator_, GSub_T, gSub, &IndicesSeenSeparator_);
          }
      }
  }

  for (const auto& child : this->children) {
      this->_merge(&FrontalMap_, &SeparatorMap_, child->SeparatorMap_, &IndicesSeenFrontal_, &columnsFrontals_,
                   &IndicesSeenSeparator_, &columnsSeparator_ );
  }
}

/* ************************************************************************* */
void ISAM2Clique::cacheConstraintFactors(FastVector<boost::shared_ptr<GaussianFactor>> constraintFactors,
                                         FastVector<int> factorIndices) {
  
  //TODO we don't really need to store constraintFactors_ and factorIndices_ anymore
  for(auto fc: constraintFactors) {
      boost::shared_ptr<JacobianFactor> jacFactor = boost::make_shared<JacobianFactor>(*fc); 
      jacFactor->setIsConstraintFactor(1);
      jacFactor->setConstraintFactorType(fc->constraintFactorType());
      jacFactor->setRho(*(fc->getRho()));
      constraintFactors_.push_back(jacFactor);
  }
  factorIndices_ = factorIndices;
  this->getVectors();
}

/* ************************************************************************* */
bool ISAM2Clique::equals(const This& other, double tol) const {
  return Base::equals(other) &&
         ((!cachedFactor_ && !other.cachedFactor_) ||
          (cachedFactor_ && other.cachedFactor_ &&
           cachedFactor_->equals(*other.cachedFactor_, tol)));
}

/* ************************************************************************* */
void ISAM2Clique::print(const string& s, const KeyFormatter& formatter) const {
  std::cout << "*****CLIQUE*****" << std::endl;
  Base::print(s, formatter);
  if (cachedFactor_)
    cachedFactor_->print(s + "Cached: ", formatter);
  else
    cout << s << "Cached empty" << endl;
  if (gradientContribution_.rows() != 0)
    gtsam::print(gradientContribution_, "Gradient contribution: ");
  
  if (factorIndices_.size() != 0) {
    std::cout << "Factor Indices:" << std::endl;
    for (int j=0; j < factorIndices_.size(); j++) {
      std::cout << factorIndices_[j] << ", ";
    }
    std::cout << std::endl;
    std::cout << "Constrained Factors:" << std::endl;
    for (int j=0; j < factorIndices_.size(); j++) {
      constraintFactors_[j]->print();
    }
  }
  std::cout << "y_" << std::endl;
  std::cout << y_ << std::endl;
}

/* ************************************************************************* */
bool ISAM2Clique::isDirty(const KeySet& replaced, const KeySet& changed) const {
  // if none of the variables in this clique (frontal and separator!) changed
  // significantly, then by the running intersection property, none of the
  // cliques in the children need to be processed

  // Are any clique variables part of the tree that has been redone?
  bool dirty = replaced.exists(conditional_->frontals().front());
#if !defined(NDEBUG) && defined(GTSAM_EXTRA_CONSISTENCY_CHECKS)
  for (Key frontal : conditional_->frontals()) {
    assert(dirty == replaced.exists(frontal));
  }
#endif

  // If not, then has one of the separator variables changed significantly?
  if (!dirty) {
    for (Key parent : conditional_->parents()) {
      if (changed.exists(parent)) {
        dirty = true;
        break;
      }
    }
  }
  return dirty;
}

/* ************************************************************************* */
/**
 * Back-substitute - special version stores solution pointers in cliques for
 * fast access.
 */
void ISAM2Clique::fastBackSubstitute(VectorValues* delta) const {
#ifdef USE_BROKEN_FAST_BACKSUBSTITUTE
  // TODO(gareth): This code shares a lot of logic w/ linearAlgorithms-inst,
  // potentially refactor

  // Create solution part pointers if necessary and possible - necessary if
  // solnPointers_ is empty, and possible if either we're a root, or we have
  // a parent with valid solnPointers_.
  ISAM2Clique::shared_ptr parent = parent_.lock();
  if (solnPointers_.empty() && (isRoot() || !parent->solnPointers_.empty())) {
    for (Key frontal : conditional_->frontals())
      solnPointers_.emplace(frontal, delta->find(frontal));
    for (Key parentKey : conditional_->parents()) {
      assert(parent->solnPointers_.exists(parentKey));
      solnPointers_.emplace(parentKey, parent->solnPointers_.at(parentKey));
    }
  }

  // See if we can use solution part pointers - we can if they either
  // already existed or were created above.
  if (!solnPointers_.empty()) {
    GaussianConditional& c = *conditional_;
    // Solve matrix
    Vector xS;
    {
      // Count dimensions of vector
      DenseIndex dim = 0;
      FastVector<VectorValues::const_iterator> parentPointers;
      parentPointers.reserve(conditional_->nrParents());
      for (Key parent : conditional_->parents()) {
        parentPointers.push_back(solnPointers_.at(parent));
        dim += parentPointers.back()->second.size();
      }

      // Fill parent vector
      xS.resize(dim);
      DenseIndex vectorPos = 0;
      for (const VectorValues::const_iterator& parentPointer : parentPointers) {
        const Vector& parentVector = parentPointer->second;
        xS.block(vectorPos, 0, parentVector.size(), 1) =
            parentVector.block(0, 0, parentVector.size(), 1);
        vectorPos += parentVector.size();
      }
    }

    // NOTE(gareth): We can no longer write: xS = b - S * xS
    // This is because Eigen (as of 3.3) no longer evaluates S * xS into
    // a temporary, and the operation trashes valus in xS.
    // See: http://eigen.tuxfamily.org/index.php?title=3.3
    const Vector rhs = c.getb() - c.S() * xS;
    const Vector solution = c.R().triangularView<Eigen::Upper>().solve(rhs);

    // Check for indeterminant solution
    if (solution.hasNaN())
      throw IndeterminantLinearSystemException(c.keys().front());

    // Insert solution into a VectorValues
    DenseIndex vectorPosition = 0;
    for (GaussianConditional::const_iterator frontal = c.beginFrontals();
         frontal != c.endFrontals(); ++frontal) {
      solnPointers_.at(*frontal)->second =
          solution.segment(vectorPosition, c.getDim(frontal));
      vectorPosition += c.getDim(frontal);
    }
  } else {
    // Just call plain solve because we couldn't use solution pointers.
    delta->update(conditional_->solve(*delta));
  }
#else
  delta->update(conditional_->solve(*delta));
#endif
}

// BEGIN CONSTRAINTS EDITS
/* ************************************************************************* */
/**
 * Back-substitute - special version stores solution pointers in cliques for
 * fast access.
 */
void ISAM2Clique::fastBackSubstituteOtherRHS(VectorValues* delta, 
                                             std::map<int, std::tuple<FastVector<double>*, FastVector<double>*, FastVector<double>*>>& cliqueMultiplierUpdate, 
                                             int iter, double* residualPrimal,
                                             const std::map<std::string, double>& constraintPenaltyParams_) {
  std::pair<VectorValues, Vector>  ret = conditional_->solveConstrained(*delta, this->y_);
  delta->update(ret.first);
  //delta->capInPlace(1);
  this->updateMultiplierTerms(cliqueMultiplierUpdate, const_cast<VectorValues*>(delta), iter, residualPrimal, constraintPenaltyParams_);  
}

/* ************************************************************************* */
bool ISAM2Clique::valuesChanged(const KeySet& replaced,
                                const Vector& originalValues,
                                const VectorValues& delta,
                                double threshold) const {
  auto frontals = conditional_->frontals();
  if (replaced.exists(frontals.front())) return true;
  Vector diff = originalValues - delta.vector(frontals);
  return diff.lpNorm<Eigen::Infinity>() >= threshold;
}

/* ************************************************************************* */
/// Set changed flag for each frontal variable
void ISAM2Clique::markFrontalsAsChanged(KeySet* changed) const {
  for (Key frontal : conditional_->frontals()) {
    changed->insert(frontal);
  }
}

/* ************************************************************************* */
void ISAM2Clique::restoreFromOriginals(const Vector& originalValues,
                                       VectorValues* delta) const {
  size_t pos = 0;
  for (Key frontal : conditional_->frontals()) {
    auto v = delta->at(frontal);
    v = originalValues.segment(pos, v.size());
    pos += v.size();
  }
}

/* ************************************************************************* */
// Note: not being used right now in favor of non-recursive version below.
void ISAM2Clique::optimizeWildfire(const KeySet& replaced, double threshold,
                                   KeySet* changed, VectorValues* delta,
                                   size_t* count) const {
  if (isDirty(replaced, *changed)) {
    // Temporary copy of the original values, to check how much they change
    auto originalValues = delta->vector(conditional_->frontals());

    // Back-substitute
    fastBackSubstitute(delta);
    count += conditional_->nrFrontals();

    if (valuesChanged(replaced, originalValues, *delta, threshold)) {
      markFrontalsAsChanged(changed);
    } else {
      restoreFromOriginals(originalValues, delta);
    }

    // Recurse to children
    for (const auto& child : children) {
      child->optimizeWildfire(replaced, threshold, changed, delta, count);
    }
  }
}

size_t optimizeWildfire(const ISAM2Clique::shared_ptr& root, double threshold,
                        const KeySet& keys, VectorValues* delta) {
  KeySet changed;
  size_t count = 0;
  // starting from the root, call optimize on each conditional
  if (root) root->optimizeWildfire(keys, threshold, &changed, delta, &count);
  return count;
}

/* ************************************************************************* */
bool ISAM2Clique::optimizeWildfireNode(const KeySet& replaced, double threshold,
                                       KeySet* changed, VectorValues* delta,
                                       size_t* count) const {
  // TODO(gareth): This code shares a lot of logic w/ linearAlgorithms-inst,
  // potentially refactor
  bool dirty = isDirty(replaced, *changed);
  if (dirty) {
    // Temporary copy of the original values, to check how much they change
    auto originalValues = delta->vector(conditional_->frontals());

    // Back-substitute
    fastBackSubstitute(delta);
    count += conditional_->nrFrontals();

    if (valuesChanged(replaced, originalValues, *delta, threshold)) {
      markFrontalsAsChanged(changed);
    } else {
      restoreFromOriginals(originalValues, delta);
    }
  }

  return dirty;
}

size_t optimizeWildfireNonRecursive(const ISAM2Clique::shared_ptr& root,
                                    double threshold, const KeySet& keys,
                                    VectorValues* delta) {
  KeySet changed;
  size_t count = 0;

  if (root) {
    std::stack<ISAM2Clique::shared_ptr> travStack;
    travStack.push(root);
    ISAM2Clique::shared_ptr currentNode = root;
    while (!travStack.empty()) {
      currentNode = travStack.top();
      travStack.pop();
      bool dirty = currentNode->optimizeWildfireNode(keys, threshold, &changed,
                                                     delta, &count);
      if (dirty) {
        for (const auto& child : currentNode->children) {
          travStack.push(child);
        }
      }
    }
  }

  return count;
}

// BEGIN CONSTRAINTS EDITS

/* ************************************************************************* */
bool ISAM2Clique::optimizeWildfireNodeOtherRHS(const KeySet& replaced, double threshold,
                                       KeySet* changed, VectorValues* delta, size_t* count,
                                       NonlinearFactorGraph nonlinearFactors_,
                                       int iter, double* residualPrimal,
                                       const std::map<std::string, double>& constraintPenaltyParams_) {
  // TODO(gareth): This code shares a lot of logic w/ linearAlgorithms-inst,
  // potentially refactor
  bool dirty = isDirty(replaced, *changed);
  if (dirty) {
    // Temporary copy of the original values, to check how much they change
    auto originalValues = delta->vector(conditional_->frontals());


    map<int, tuple<FastVector<double>*, FastVector<double>*, FastVector<double>*>> cliqueMultiplierUpdate;
    for(auto &v: this->factorIndices_) {
      cliqueMultiplierUpdate[v] = make_tuple(nonlinearFactors_.at(v)->getdualV(),
                                            nonlinearFactors_.at(v)->getRho(),
                                            nonlinearFactors_.at(v)->getConstraintViolation());
    }
    // Back-substitute
    fastBackSubstituteOtherRHS(delta, cliqueMultiplierUpdate, iter, residualPrimal, constraintPenaltyParams_);
    count += conditional_->nrFrontals();

    if (valuesChanged(replaced, originalValues, *delta, threshold)) {
      markFrontalsAsChanged(changed);
    } else {
      restoreFromOriginals(originalValues, delta);
    }
  }

  return dirty;
}

/* ************************************************************************* */
size_t optimizeWildfireNonRecursiveOtherRHS(const ISAM2Clique::shared_ptr& root,
                                    double threshold, const KeySet& keys,
                                    VectorValues* delta,
                                    NonlinearFactorGraph nonlinearFactors_,
                                    int iter, double* residualPrimal,
                                    FastList<ISAM2Clique::shared_ptr>* stopCliques,
                                    const std::map<std::string, double>& constraintPenaltyParams_) {
  KeySet changed;
  size_t count = 0;

  if (root) {
    std::stack<ISAM2Clique::shared_ptr> travStack;
    travStack.push(root);
    ISAM2Clique::shared_ptr currentNode = root;
    while (!travStack.empty()) {
      currentNode = travStack.top();
      travStack.pop();
      bool dirty = currentNode->optimizeWildfireNodeOtherRHS(keys, threshold, &changed,
                                                     delta, &count, nonlinearFactors_,
                                                     iter, residualPrimal, constraintPenaltyParams_);
      if (dirty) {
        for (const auto& child : currentNode->children) {
          travStack.push(child);
        }
      }
      else {
        stopCliques->push_back(currentNode);
      }
    }
  }

  return count;
}
// END CONSTRAINTS EDITS

/* ************************************************************************* */
void ISAM2Clique::nnz_internal(size_t* result) const {
  size_t dimR = conditional_->rows();
  size_t dimSep = conditional_->S().cols();
  *result += ((dimR + 1) * dimR) / 2 + dimSep * dimR;
  // traverse the children
  for (const auto& child : children) {
    child->nnz_internal(result);
  }
}

/* ************************************************************************* */
size_t ISAM2Clique::calculate_nnz() const {
  size_t result = 0;
  nnz_internal(&result);
  return result;
}

/* ************************************************************************* */
void ISAM2Clique::findAll(const KeySet& markedMask, KeySet* keys) const {
  static const bool debug = false;
  // does the separator contain any of the variables?
  bool found = false;
  for (Key key : conditional_->parents()) {
    if (markedMask.exists(key)) {
      found = true;
      break;
    }
  }
  if (found) {
    // then add this clique
    keys->insert(conditional_->beginFrontals(), conditional_->endFrontals());
    if (debug) print("Key(s) marked in clique ");
    if (debug) cout << "so marking key " << conditional_->front() << endl;
  }
  for (const auto& child : children) {
    child->findAll(markedMask, keys);
  }
}

/* ************************************************************************* */
void ISAM2Clique::addGradientAtZero(VectorValues* g) const {
  // Loop through variables in each clique, adding contributions
  DenseIndex position = 0;
  for (auto it = conditional_->begin(); it != conditional_->end(); ++it) {
    const DenseIndex dim = conditional_->getDim(it);
    const Vector contribution = gradientContribution_.segment(position, dim);
    VectorValues::iterator values_it;
    bool success;
    std::tie(values_it, success) = g->tryInsert(*it, contribution);
    if (!success) values_it->second += contribution;
    position += dim;
  }

  // Recursively add contributions from children
  for (const auto& child : children) {
    child->addGradientAtZero(g);
  }
}

/* ************************************************************************* */
}  // namespace gtsam
