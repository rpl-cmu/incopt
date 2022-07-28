/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/nonlinear/ISAM2Clique.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/nonlinear/ISAM2Constrained-impl.h>


//#include <boost/bimap.hpp>

#include <vector>

// BEGIN CONSTRAINTS EDITS
#include <Eigen/Sparse>
// END CONSTRAINTS EDITS

namespace gtsam {

/**
 * @addtogroup ISAM2
 * Implementation of the full ISAM2 algorithm for incremental nonlinear
 * optimization.
 *
 * The typical cycle of using this class to create an instance by providing
 * ISAM2Params to the constructor, then add measurements and variables as they
 * arrive using the update() method.  At any time, calculateEstimate() may be
 * called to obtain the current estimate of all variables.
 *
 */
class GTSAM_EXPORT ISAM2 : public BayesTree<ISAM2Clique> {
 protected:
  
  /** The current linearization point */
  Values theta_;

  /** VariableIndex lets us look up factors by involved variable and keeps track
   * of dimensions */
  VariableIndex variableIndex_;

  /** The linear delta from the last linear solution, an update to the estimate
   * in theta
   *
   * This is \c mutable because it is a "cached" variable - it is not updated
   * until either requested with getDelta() or calculateEstimate(), or needed
   * during update() to evaluate whether to relinearize variables.
   */
  mutable VectorValues delta_;

  mutable VectorValues deltaNewton_;  // Only used when using Dogleg - stores
                                      // the Gauss-Newton update
  mutable VectorValues RgProd_;  // Only used when using Dogleg - stores R*g and
                                 // is updated incrementally

  typedef ConcurrentMap<Key, sharedClique> Nodes;
  /** A cumulative mask for the variables that were replaced and have not yet
   * been updated in the linear solution delta_, this is only used internally,
   * delta will always be updated if necessary when requested with getDelta()
   * or calculateEstimate().
   *
   * This is \c mutable because it is used internally to not update delta_
   * until it is needed.
   */

  mutable KeySet deltaReplacedMask_;  // TODO(dellaert): Make sure accessed in
                                      // the right way

  /** All original nonlinear factors are stored here to use during
   * relinearization */
  NonlinearFactorGraph nonlinearFactors_;

  /** The current linear factors, which are only updated as needed */
  mutable GaussianFactorGraph linearFactors_;

  /** The current parameters */
  ISAM2Params params_;

  /** The current Dogleg Delta (trust region radius) */
  mutable boost::optional<double> doglegDelta_;

  /** Set of variables that are involved with linear factors from marginalized
   * variables and thus cannot have their linearization points changed. */
  KeySet fixedVariables_;

  int update_count_;  ///< Counter incremented every update(), used to determine
                      ///< periodic relinearization

  // BEGIN CONSTRAINTS EDITS
  /*
  typedef boost::bimap<Key, size_t > bm_keyorder;
  typedef bm_keyorder::left_map::const_iterator left_bm_keyorder_const_iterator;
  typedef bm_keyorder::right_map::const_iterator right_bm_keyorder_const_iterator;
  */

  bool enableConstraints_;
  NonlinearFactorGraph nonlinearConstraintFactors_;


  Ordering orderedKeys_;
  // accVarDims_ is used when constructing the R and Rt matrices
  std::vector<size_t> accVarDims_;
  FastMap<Key, size_t> orderedKeysInverse_; // inverse mapping (not reverse ordering!)

  Eigen::SparseMatrix<double, Eigen::RowMajor> RTranspose_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> R_;
  Matrix d_;
  VectorValues rhsOther_;
  Vector constraintTypeMap_; 
  double constraintDeltaMaxStep_ = 0.1; // Cap to delta x in constrained optimization
  int numInnerIter_ = 1;
  bool debug_print_ = false;

  #define requireDebug(req, msg) if (!(req)) {fputs(msg, stderr);fputs("\n\n", stderr); abort();}
  // END CONSTRAINTS EDITS

 public:
  using This = ISAM2;                       ///< This class
  using Base = BayesTree<ISAM2Clique>;      ///< The BayesTree base class
  using Clique = Base::Clique;              ///< A clique
  using sharedClique = Base::sharedClique;  ///< Shared pointer to a clique
  using Cliques = Base::Cliques;            ///< List of Cliques

  /** Create an empty ISAM2 instance */
  explicit ISAM2(const ISAM2Params& params);

  /** Create an empty ISAM2 instance using the default set of parameters (see
   * ISAM2Params) */
  ISAM2();

  /** default virtual destructor */
  virtual ~ISAM2() {}

  /** Compare equality */
  virtual bool equals(const ISAM2& other, double tol = 1e-9) const;

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Optionally, this function remove existing factors from the system to enable
   * behaviors such as swapping existing factors with new ones.
   *
   * Add new measurements, and optionally new variables, to the current system.
   * This runs a full step of the ISAM2 algorithm, relinearizing and updating
   * the solution as needed, according to the wildfire and relinearize
   * thresholds.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the
   * system. You must include here all new variables occuring in newFactors
   * (which were not already in the system).  There must not be any variables
   * here that do not occur in newFactors, and additionally, variables that were
   * already in the system must not be included here.
   * @param removeFactorIndices Indices of factors to remove from system
   * @param force_relinearize Relinearize any variables whose delta magnitude is
   * sufficiently large (Params::relinearizeThreshold), regardless of the
   * relinearization interval (Params::relinearizeSkip).
   * @param constrainedKeys is an optional map of keys to group labels, such
   * that a variable can be constrained to a particular grouping in the
   * BayesTree
   * @param noRelinKeys is an optional set of nonlinear keys that iSAM2 will
   * hold at a constant linearization point, regardless of the size of the
   * linear delta
   * @param extraReelimKeys is an optional set of nonlinear keys that iSAM2 will
   * re-eliminate, regardless of the size of the linear delta. This allows the
   * provided keys to be reordered.
   * @return An ISAM2Result struct containing information about the update
   */
  virtual ISAM2Result update(
      const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
      const Values& newTheta = Values(),
      const FactorIndices& removeFactorIndices = FactorIndices(),
      const boost::optional<FastMap<Key, int> >& constrainedKeys = boost::none,
      const boost::optional<FastList<Key> >& noRelinKeys = boost::none,
      const boost::optional<FastList<Key> >& extraReelimKeys = boost::none,
      bool force_relinearize = false);

  void updateDeltaConstrained(double* result, bool forceFullSolve);

  double errorConstrained();
  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Alternative signature of update() (see its documentation above), with all
   * additional parameters in one structure. This form makes easier to keep
   * future API/ABI compatibility if parameters change.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the
   * system. You must include here all new variables occuring in newFactors
   * (which were not already in the system).  There must not be any variables
   * here that do not occur in newFactors, and additionally, variables that were
   * already in the system must not be included here.
   * @param updateParams Additional parameters to control relinearization,
   * constrained keys, etc.
   * @return An ISAM2Result struct containing information about the update
   * @note No default parameters to avoid ambiguous call errors.
   */
  virtual ISAM2Result update(const NonlinearFactorGraph& newFactors,
                             const Values& newTheta,
                             const ISAM2UpdateParams& updateParams);

  /** Marginalize out variables listed in leafKeys.  These keys must be leaves
   * in the BayesTree.  Throws MarginalizeNonleafException if non-leaves are
   * requested to be marginalized.  Marginalization leaves a linear
   * approximation of the marginal in the system, and the linearization points
   * of any variables involved in this linear marginal become fixed.  The set
   * fixed variables will include any key involved with the marginalized
   * variables in the original factors, and possibly additional ones due to
   * fill-in.
   *
   * If provided, 'marginalFactorsIndices' will be augmented with the factor
   * graph indices of the marginal factors added during the 'marginalizeLeaves'
   * call
   *
   * If provided, 'deletedFactorsIndices' will be augmented with the factor
   * graph indices of any factor that was removed during the 'marginalizeLeaves'
   * call
   */
  void marginalizeLeaves(
      const FastList<Key>& leafKeys,
      boost::optional<FactorIndices&> marginalFactorsIndices = boost::none,
      boost::optional<FactorIndices&> deletedFactorsIndices = boost::none);

  /// Access the current linearization point
  const Values& getLinearizationPoint() const { return theta_; }

  /// Check whether variable with given key exists in linearization point
  bool valueExists(Key key) const { return theta_.exists(key); }

  /** Compute an estimate from the incomplete linear delta computed during the
   * last update. This delta is incomplete because it was not updated below
   * wildfire_threshold.  If only a single variable is needed, it is faster to
   * call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const;
  Values calculateEstimateConstrained();
  /** Compute an estimate for a single variable using its incomplete linear
   * delta computed during the last update.  This is faster than calling the
   * no-argument version of calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template <class VALUE>
  VALUE calculateEstimate(Key key) const {
    const Vector& delta = getDelta()[key];
    return traits<VALUE>::Retract(theta_.at<VALUE>(key), delta);
  }

  /** Compute an estimate for a single variable using its incomplete linear
   * delta computed during the last update.  This is faster than calling the
   * no-argument version of calculateEstimate, which operates on all variables.
   * This is a non-templated version that returns a Value base class for use
   * with the MATLAB wrapper.
   * @param key
   * @return
   */
  const Value& calculateEstimate(Key key) const;

  /** Return marginal on any variable as a covariance matrix */
  Matrix marginalCovariance(Key key) const;

  /// @name Public members for non-typical usage
  /// @{

  /** Compute an estimate using a complete delta computed by a full
   * back-substitution.
   */
  Values calculateBestEstimate() const;

  /** Access the current delta, computed during the last call to update */
  const VectorValues& getDelta() const;

  const VectorValues& getDeltaConstrained();
  /** Compute the linear error */
  double error(const VectorValues& x) const;

  /** Access the set of nonlinear factors */
  const NonlinearFactorGraph& getFactorsUnsafe() const {
    return nonlinearFactors_;
  }

  /** Access the nonlinear variable index */
  const VariableIndex& getVariableIndex() const { return variableIndex_; }

  /** Access the nonlinear variable index */
  const KeySet& getFixedVariables() const { return fixedVariables_; }

  const ISAM2Params& params() const { return params_; }

  /** prints out clique statistics */
  void printStats() const { getCliqueData().getStats().print(); }

  /** Compute the gradient of the energy function, \f$ \nabla_{x=0} \left\Vert
   * \Sigma^{-1} R x - d \right\Vert^2 \f$, centered around zero. The gradient
   * about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&,
   * const VectorValues&).
   *
   * @return A VectorValues storing the gradient.
   */
  VectorValues gradientAtZero() const;

  // BEGIN CONSTRAINTS EDITS

  std::map<std::string, double> constraintPenaltyParams_ = {
    {"growth",  5},
    {"reduction",  0.2},
    {"maxRhoVal",  1e2},
    {"minRhoVal",  2},
    {"minChange",  0.5}
  };
  Vector capByDim_;

  void updateOrdering();
  void updateRTranspose();
  void updateRTransposeOld();
  void setEnableConstraints(bool flag) { enableConstraints_ = flag; }
  const Eigen::SparseMatrix<double, Eigen::RowMajor>& getRTranspose() const { return RTranspose_; }

  void createConstraintJacobian(const bool batch, Matrix& G, Matrix& g, Vector& constraintTypeMap, Matrix& A, Matrix& b, Matrix& Afull, Matrix& bfull) const;
  void formConstrainedSystem(Matrix& G, Matrix& g, Vector& constraintTypeMap, Matrix& A, Matrix& b, 
                             Matrix& Afull, Matrix& bfull, int numRowsHardConstraints, int numRowsSoftConstraints, int n); 
  const VectorValues solveConstraints(Matrix& G, Matrix& g, Vector& constraintTypeMap, Matrix& A, Matrix& b, 
                                      Matrix& Afull, Matrix& bfull, int numRowsHardConstraints, int numRowsSoftConstraints, int n);
  void updateDeltaOtherRHS(bool forceFullSolve) const;

  double avgErrorBConstrained();
  double sumErrorConstrained();
  double avgErrorConstrained();

  const NonlinearFactorGraph& getConstraintFactorsUnsafe() const {
    return nonlinearConstraintFactors_;
  }

  void setConstraintDeltaMaxStep(double constraintDeltaMaxStep) {
    constraintDeltaMaxStep_ = constraintDeltaMaxStep;
  } 

  double getConstraintDeltaMaxStep() {
    return constraintDeltaMaxStep_;
  } 

  void setNumInnerIter(int numInnerIter) {
    numInnerIter_ = numInnerIter;
  } 

  int getNumInnerIter() {
    return numInnerIter_;
  } 

  void setDebugPrint(bool debug_print) {
    debug_print_ = debug_print;
  } 

  bool getDebugPrint() {
    return debug_print_;
  } 

  void setCapByDim(Vector capByDim) {
    capByDim_ = capByDim;
  } 

  Vector getCapByDim() {
    return capByDim_;
  } 

  void setRhoGrowth(double growth) {
    constraintPenaltyParams_["growth"] = growth;
  }

  void setRhoReduction(double reduction) {
    constraintPenaltyParams_["reduction"] = reduction;
  }  

  void setRhoMaxRhoVal(double maxRhoVal) {
    constraintPenaltyParams_["maxRhoVal"] = maxRhoVal;
  }  

  void setRhominRhoVal(double minRhoVal) {
    constraintPenaltyParams_["minRhoVal"] = minRhoVal;
  }  

  void setRhoMinChange(double minChange) {
    constraintPenaltyParams_["minChange"] = minChange;
  }  

  void getConstraintPenaltyParams() {
    for(auto it = constraintPenaltyParams_.begin(); it != constraintPenaltyParams_.end(); it++) {
      std::cout<< it->first << " = " << it->second << std::endl;
    }
  }

  unsigned int numRowsHardConstraints_ = 0;
  unsigned int numNonLinearFactorsOld_ = 0;
  Cliques orphans_;




  // END CONSTRAINTS EDITS
  
  /// @}

 protected:
  /// Remove marked top and either recalculate in batch or incrementally.
  void recalculate(const ISAM2UpdateParams& updateParams,
                   const KeySet& relinKeys, ISAM2Result* result);

  // Do a batch step - reorder and relinearize all variables
  void recalculateBatch(const ISAM2UpdateParams& updateParams,
                        KeySet* affectedKeysSet, ISAM2Result* result);

  // retrieve all factors that ONLY contain the affected variables
  // (note that the remaining stuff is summarized in the cached factors)
  GaussianFactorGraph relinearizeAffectedFactors(
      const ISAM2UpdateParams& updateParams, const FastList<Key>& affectedKeys,
      const KeySet& relinKeys,
      FastVector<int>* factorIndices);

  void recalculateIncremental(const ISAM2UpdateParams& updateParams,
                              const KeySet& relinKeys,
                              const FastList<Key>& affectedKeys,
                              KeySet* affectedKeysSet, Cliques* orphans,
                              ISAM2Result* result);

  /**
   * Add new variables to the ISAM2 system.
   * @param newTheta Initial values for new variables
   * @param variableStatus optional detailed result structure
   */
  void addVariables(const Values& newTheta,
                    ISAM2Result::DetailedResults* detail = 0);

  /**
   * Remove variables from the ISAM2 system.
   */
  void removeVariables(const KeySet& unusedKeys);

  void updateDelta(bool forceFullSolve = false) const;
};  // ISAM2

/// traits
template <>
struct traits<ISAM2> : public Testable<ISAM2> {};

}  // namespace gtsam
