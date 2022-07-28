/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2Constrained-impl.h
 * @brief   Incremental constrained update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Paloma Sodhi, Mohamad Qadri
 */

#pragma once

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2-impl.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <unordered_map>

#include <gtsam/base/debug.h>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include <map>
#include <queue>          // std::queue
#include <fstream>
#include <iostream>
#include <Eigen/Sparse>
#include <boost/optional/optional_io.hpp>

using namespace std;


namespace gtsam {

/* ************************************************************************* */
/**
 * Implementation functions for all methods pertaining to constrained smoothing
 */
struct GTSAM_EXPORT ISAM2Constrained: BayesTree<ISAM2Clique>{
    
    public:
        typename  ConcurrentMap<Key, sharedClique>::const_iterator node;
        typedef Eigen::SparseMatrix<double, Eigen::RowMajor> Rtype;
        typedef Eigen::Triplet<double> Trip;
        typedef vector<tuple<Matrix, Vector, int>> Mtype;
        const ISAM2Params params_;
        const ISAM2UpdateParams updateParams_;
        bool debug_print_ = false;
        std::map<std::string, double> constraintPenaltyParams_;

    ISAM2Constrained(ISAM2Params params, bool debug_print, 
                     std::map<std::string, double> constraintPenaltyParams): params_(params), debug_print_(debug_print), constraintPenaltyParams_(constraintPenaltyParams){}

    template<typename T>
    void printVector(const T& t) {
        std::copy(t.cbegin(), t.cend(), std::ostream_iterator<typename T::value_type>(std::cout, ", "));
        std::cout << std::endl;
    }

   void updateDeltaOtherRHS(bool forceFullSolve, VectorValues* delta_, Roots& roots_, 
                            KeySet& deltaReplacedMask_, NonlinearFactorGraph nonlinearFactors_,
                            int iter, double* residualPrimal,
                            FastList<ISAM2::sharedClique>* stopCliques) const {
        
        if (params_.optimizationParams.type() == typeid(ISAM2GaussNewtonParams)) {
            // If using Gauss-Newton, update with wildfireThreshold
            const ISAM2GaussNewtonParams& gaussNewtonParams =
                boost::get<ISAM2GaussNewtonParams>(params_.optimizationParams);
            const double effectiveWildfireThreshold =
                forceFullSolve ? 0.0 : gaussNewtonParams.wildfireThreshold;

            DeltaImpl::UpdateGaussNewtonDeltaOtherRHS(roots_, deltaReplacedMask_,
                                                      effectiveWildfireThreshold, delta_,
                                                      nonlinearFactors_, iter, residualPrimal, 
                                                      stopCliques, constraintPenaltyParams_);
            deltaReplacedMask_.clear();

        } 
        else {
            throw std::runtime_error("iSAM2: unknown ISAM2Params type");
        }
    }

    void findLeaves(Roots& roots, std::queue<sharedClique>& upwardPassQueue) 
    {

        std::queue<sharedClique> travQueue;
        sharedClique currentNode;
        for (auto root: roots) {
            travQueue.push(root);
            while (!travQueue.empty()) {
                currentNode = travQueue.front();
                travQueue.pop();
                if(currentNode->children.size() == 0) {
                    upwardPassQueue.push(currentNode);
                }
                else {
                    for (const sharedClique& child : currentNode->children) {
                        travQueue.push(child);
                    }
                }
            }
        }
    }

    void printQueue(std::queue<sharedClique>& q) {
        sharedClique currentNode;
        while (!q.empty()) {
            currentNode = q.front();
            q.pop();
            std::cout << "Address" << std::endl;
            std::cout << currentNode << std::endl;
            std::cout << "Clique" << std::endl;
            currentNode->print();    
            std::cout << "factorIndices " << std::endl;
            for (auto e: currentNode->factorIndices_) std::cout << e << std::endl;
            std::cout << "factors " << std::endl;
            for (auto e: currentNode->constraintFactors_) e->print();
            if(currentNode->parent())
                q.push(currentNode->parent());
        }
    }
    
    KeyVector getCliqueFrontals(sharedClique& currentNode) {
        KeyVector frontals;
        for (auto f = currentNode->conditional_->beginFrontals(); f != currentNode->conditional_->endFrontals(); f++) {
            frontals.push_back(*f);
        }
        return frontals;
    }

    KeyVector getCliqueFrontalsConst(const sharedClique& currentNode) {
        KeyVector frontals;
        for (auto f = currentNode->conditional_->beginFrontals(); f != currentNode->conditional_->endFrontals(); f++) {
            frontals.push_back(*f);
        }
        return frontals;
    }

    KeyVector getCliqueSeparators(sharedClique& currentNode) {
        KeyVector separators;
        for (auto s = currentNode->conditional_->beginParents() ; s != currentNode->conditional_->endParents(); s++) {
            separators.push_back(*s);
        }
        return separators;
    }

    void collectUTerm(sharedClique& currentNode, Vector& U, FastMap<Key, int>& rowsFrontals, Values& theta_, KeyVector& F) {
        for (const sharedClique& child : currentNode->children) {
            for (auto it = child->childContribution_.begin() ; it != child->childContribution_.end(); it++) {
                if(!it->second) continue;
                if (std::find(F.begin(), F.end(), it->first) == F.end()) continue; 
                Vector tmp = Vector::Zero(U.rows());
                int numRows = theta_.at(it->first).dim();
                tmp.block(rowsFrontals[it->first], 0, numRows, 1) = *it->second;
                U = U + tmp;
            }
        }
    }

    void setUTerm(sharedClique& currentNode) {
       // Loop over separator variable of current node
        for (auto s = currentNode->conditional_->beginParents() ; s != currentNode->conditional_->endParents(); s++) {
            currentNode->childContribution_[*s] = currentNode->conditional_->S(s).transpose()*currentNode->y_;
            // for each child see if there are any additional contributions for this separator variable.
            for (const auto& child : currentNode->children) {
                if (child->childContribution_.find(*s)  != child->childContribution_.end()) {

                    if (child->childContribution_[*s]->size() != 0)
                        if(child->childContribution_[*s])
                            currentNode->childContribution_[*s] = *(currentNode->childContribution_[*s]) + *(child->childContribution_[*s]);
                }
            }
        }   
    }

    void setMatrixPartG(Matrix& GClique, Matrix& G, int startRow, int startCol, int endRow, int endCol) {
        GClique.block(startRow, startCol, endRow - startRow, endCol - startCol) = G;
    }
    void setVectorPart(Vector& gClique, Vector& gSub, int startRow, int endRow) {
        gClique.block(startRow, 0, endRow - startRow, 1) = gSub;
    }

    void getVectors(sharedClique& currentNode, 
                    NonlinearFactorGraph nonlinearFactors_,
                    Values& theta_) {
        // R.T y = G.T dualV

        auto RT = currentNode->conditional_->R().transpose();
        Matrix GTClique = Matrix::Zero(RT.rows(), currentNode->columnsFrontals_.back());
        Vector gClique = Vector::Zero(currentNode->columnsFrontals_.back());
        Vector vClique = Vector::Zero(currentNode->columnsFrontals_.back());

        // This means that there are no hard constraints specified in this Bayes Clique
        // so we perform the update R*delta = d - y where y = 0
        if(currentNode->columnsFrontals_.back() == 0) {
            currentNode->y_ = Vector::Zero(RT.rows());
            return;
        }
        
        FastMap<Key, int> order; 
        FastMap<Key, int> rowsFrontals;
        KeyVector F = getCliqueFrontals(currentNode);

        int j =0;
        int prev_dim;
        // loop over all frontal variables
        // the rowsFrontals hold the start and end ROW of each variable with respect to the previous dimension:
        // i.e the first frontal variable starts at row = 0 .
        // the second frontal variable starts at row = firstVar.dim()
        // the third frontal variable starts at row = secondVar.dim()
        
        for(auto& k: F) {
            // set the order of frontal var
            order[k] = j;
            if(j==0) {
                rowsFrontals[k] = 0;
                prev_dim = theta_.at(k).dim();
                j++;
                continue;
            }
            rowsFrontals[k] = prev_dim;
            prev_dim = theta_.at(k).dim() + prev_dim;
            j++;
        }

        // We need to construct GTclique, vClique and gClique
        // Loop over Frontal Variables in order
        int index_r = -1;
        for (auto it = currentNode->FrontalMap_.begin(); it != currentNode->FrontalMap_.end(); ++it) {
            index_r++;
            Key key = it->first;
            // if there are stored jacobians 
            if(it->second) {
                Mtype parts = *(it->second);
                for(auto &p: parts) {
                    Matrix G = get<0>(p);
                    Vector g = get<1>(p);
                    // I is the constraint factor index
                    int I = get<2>(p);
                    int index_c;
                    // auto <=> std::vector<int>::iterator
                    //IndicesSeenFrontal_ contains the indices of the corresponding lagrange multiplier (= indices of non linear factors)
                    // get the index of I in the IndicesSeenFrontal vector. THis is index_c
                    auto it = std::find(currentNode->IndicesSeenFrontal_.begin(), currentNode->IndicesSeenFrontal_.end(), I);
                    if(it != currentNode->IndicesSeenFrontal_.end()) {
                        index_c = std::distance(currentNode->IndicesSeenFrontal_.begin(), it);
                    }
                    else {
                        throw std::runtime_error("Factor index does not exist in IndicesSeenFrontal");
                    }
                    // rowsFrontals tells the start and end row . Row aligned with variables
                    // and currentNode->columnsFrontals_[index_c] tells the start and end column. Column index should be correctly aligned with
                    // the lagrange multiplier index 

                    /* 
                        [\partial g_1 \partial x_1       0      
                                                                                    x [v1 v2]^T                    
                        \partial g_1 \partial x_2       \partial g_2 \partial x_2]
                    */
                    int start_row = rowsFrontals.at(key);
                    int end_row = theta_.at(key).dim() + start_row;
                    setMatrixPartG(GTClique, G, start_row, currentNode->columnsFrontals_[index_c], end_row, currentNode->columnsFrontals_[index_c+1]);
                    setVectorPart(gClique, g, currentNode->columnsFrontals_[index_c], currentNode->columnsFrontals_[index_c+1]);
                    FastVector<double>* dualV = nonlinearFactors_.at(I)->getdualV();
                    Eigen::Map<Vector> dualVM(dualV->data(), dualV->size()); 
                    vClique.block(currentNode->columnsFrontals_[index_c], 0,  currentNode->columnsFrontals_[index_c+1] - currentNode->columnsFrontals_[index_c], 1) = dualVM;
                }
            }
        }
        currentNode->GT_ = GTClique;
        currentNode->g_ = gClique;
        currentNode->dualV_ = vClique;
        Vector U = Vector::Zero(GTClique.rows());
        collectUTerm(currentNode, U, rowsFrontals, theta_, F);
        currentNode->y_ = RT.triangularView<Eigen::Lower>().solve(GTClique * vClique - U);
        setUTerm(currentNode);
    }
    
    bool stopAtOrphan(sharedClique node_, Cliques& orphans) {
        if (std::count(orphans.begin(), orphans.end(), node_)) {
            if(node_->y_.size() > 0) {
                return true;
            }
        }
        return false;
    }

    void upwardPass(sharedClique node_, NonlinearFactorGraph nonlinearFactors_, Values& theta_,  Cliques& orphans) 
    {
        bool s = stopAtOrphan(node_, orphans);
        if (node_->children.size() == 0 || s) { 
            getVectors(node_, nonlinearFactors_, theta_);
            return;
        }
        for(sharedClique& child: node_->children) {
            upwardPass(child, nonlinearFactors_, theta_, orphans);
        }
        getVectors(node_, nonlinearFactors_, theta_);
    }


    void capInPlace(VectorValues* delta_, Vector& capByDim ) {
    // Count dimensions
    auto j1 = delta_->begin();
    for(; j1 != delta_->end(); ++j1) {
        Vector* t = &(j1->second);
        for(int i = 0; i<capByDim.size(); i++) {
            if((*t)[i] > capByDim(i) - 1e-7) {
                (*t)[i] = capByDim(i);
            }
            else if ((*t)[i] < -capByDim(i) + 1e-7) {
                (*t)[i] = -capByDim(i);
            }
        }
        }
    }

    std::tuple<bool, FastList<ISAM2::sharedClique>>  downwardPass(VectorValues* delta_, Roots& roots_, KeySet& deltaReplacedMask_, NonlinearFactorGraph nonlinearFactors_, double constraintDeltaMaxStep_,
                                                                    int iter, int numRowsConstrained, bool forceFullSolve, Vector& capByDim) {
       double epsPrimal = 1e-12;  // max constraint violation
       bool ret = false;
       double residualPrimal = 0;
       FastList<ISAM2::sharedClique> stopCliques;
       updateDeltaOtherRHS(forceFullSolve, delta_, roots_, deltaReplacedMask_, nonlinearFactors_, iter, &residualPrimal, &stopCliques);
       if (delta_->vector().lpNorm<Eigen::Infinity>() > constraintDeltaMaxStep_) ret = true; 
       if (capByDim.size() > 0) {
            capInPlace(delta_, capByDim );
       }
       else {
            delta_->capInPlace(constraintDeltaMaxStep_);
       }
       residualPrimal = sqrt(residualPrimal)/numRowsConstrained;
       if(debug_print_) std::cout << "residualPrimal " << residualPrimal << std::endl;  
       if (residualPrimal < epsPrimal) ret = true;
       return std::tuple<bool, FastList<ISAM2::sharedClique>>(ret, stopCliques);
    }

    double avgErrorBConstrained(NonlinearFactorGraph nonlinearFactors_, Values currValues, int numRowsHardConstraints) 
    {
        Vector nonlinError = Vector::Zero(numRowsHardConstraints);
        int j = 0;
        for (size_t i = 0; i < nonlinearFactors_.size(); i++) {
            if(nonlinearFactors_.factorIndexToconstraintFactorType_.at(i) < 1) continue;
            boost::shared_ptr<NonlinearFactor> factor = nonlinearFactors_.at(i);
            Vector nonlinErrorSub = factor->errorBConstrained(currValues);
            int dim = factor->dim();
            nonlinError.block(j, 0, dim, 1) = nonlinErrorSub;
            j+=dim;
        }
        double residualPrimalNonlin = nonlinError.norm() / nonlinError.rows();
        return residualPrimalNonlin;
    }

    double sumErrorConstrained(NonlinearFactorGraph nonlinearFactors_, Values currValues) 
    {
        double nonlinError = 0;
        int j = 0;
        for (size_t i = 0; i < nonlinearFactors_.size(); i++) {
            if(nonlinearFactors_.factorIndexToconstraintFactorType_.at(i) < 1) continue;
            boost::shared_ptr<NonlinearFactor> factor = nonlinearFactors_.at(i);
            nonlinError += factor->errorConstrained(currValues);
        }
        return nonlinError;
    }

    double avgErrorConstrained(NonlinearFactorGraph nonlinearFactors_, Values currValues) 
    {
        double nonlinError = 0;
        int count = 0;
        for (size_t i = 0; i < nonlinearFactors_.size(); i++) {
            if(nonlinearFactors_.factorIndexToconstraintFactorType_.at(i) < 1) continue;
            boost::shared_ptr<NonlinearFactor> factor = nonlinearFactors_.at(i);
            nonlinError += factor->errorConstrained(currValues);
            count++; 
        }
        return nonlinError/count;
    }

  }; 
}