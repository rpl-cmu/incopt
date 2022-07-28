
// function [d,dd_dx,dd_dM,dG_dx,dG_dM,pt_poly,contact_type,normvec] = p_poly_dist2(pt, M, dpt_dx, dptc_dx)

#pragma once
//#include "sp_util.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
// using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;
// typedef Eigen::Matrix<double, Dynamic, 1> VectorXd;

double min(const Eigen::VectorXd& a,
           int* minI);

Eigen::VectorXi find(const VectorXb& a);

void p_poly_dist(const Eigen::Vector2d& pt,
                 const Eigen::MatrixXd& M,
                 const Eigen::MatrixXd& dpt_dx,
                 const Eigen::MatrixXd& dptc_dx,

                 double* d,
                 Eigen::MatrixXd* dd_dx,
                 Eigen::MatrixXd* dd_dM,
                 Eigen::MatrixXd* dG_dx,
                 Eigen::MatrixXd* dG_dM,
                 Eigen::Vector2d* pt_poly,
                 int* contact_type,
                 Eigen::Vector2d* normvec,
                 int hack = 1);

void shape__probe_obj__find_norm(const Eigen::Vector2d& pt_obj,
                                 const Eigen::MatrixXd& M,

                                 Eigen::Vector2d* normvec);