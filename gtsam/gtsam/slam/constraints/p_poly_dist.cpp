//
// Created by mcube on 1/6/17.
//

// function [d,dd_dx,dd_dM,dG_dx,dG_dM,pt_poly,contact_type,normvec] = p_poly_dist2(pt, M, dpt_dx, dptc_dx)
#include <iostream>
#include <vector>
#include "p_poly_dist.h"
using namespace std;

// using namespace Eigen;

double min(const Eigen::VectorXd& a,
           int* minI) {
  double minn = a(0);
  if (minI) *minI = 0;
  for (int i = 1; i < a.size(); i++) {
    if (a(i) < minn) {
      minn = a(i);
      if (minI) *minI = i;
    }
  }
  return minn;
}

Eigen::VectorXi find(const VectorXb& a) {
  vector<int> inds;
  for (int i = 0; i < a.size(); i++) {
    if (a(i)) inds.push_back(i);
  }
  return Eigen::VectorXi::Map(inds.data(), inds.size());
}

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
                 int hack) {
  const double x = pt(0);
  const double y = pt(1);
  const int n_M = M.cols();
  if (dG_dM) {
    dd_dM->resize(2, n_M);
    dG_dM->resize(2, n_M * 2);
    dd_dM->setZero();
    dG_dM->setZero();
  }

  Eigen::VectorXd xv = M.row(0).transpose();
  Eigen::VectorXd yv = M.row(1).transpose();
  xv.conservativeResize(xv.rows() + 1);
  xv(xv.size() - 1) = xv(0);
  yv.conservativeResize(yv.rows() + 1);
  yv(yv.size() - 1) = yv(0);

  // linear parameters of segments that connect the vertices
  // Ax + By + C = 0
  Eigen::VectorXd A = -(yv.tail(n_M) - yv.head(n_M));
  Eigen::VectorXd B = (xv.tail(n_M) - xv.head(n_M));
  Eigen::VectorXd C = yv.tail(n_M).cwiseProduct(xv.head(n_M)) - xv.tail(n_M).cwiseProduct(yv.head(n_M));

  // % find the projection of point (x,y) on each rib
  Eigen::VectorXd AB = (A.array().square() + B.array().square()).array().inverse();
  Eigen::VectorXd vv = A * x + B * y + C;
  Eigen::VectorXd xx;
  xx.resize(n_M);
  xx.fill(x);
  Eigen::VectorXd yy;
  yy.resize(n_M);
  yy.fill(y);
  Eigen::VectorXd xp = xx - (A.cwiseProduct(AB)).cwiseProduct(vv);
  Eigen::VectorXd yp = yy - (B.cwiseProduct(AB)).cwiseProduct(vv);

  // Test for the case where a polygon rib is
  // either horizontal or vertical. From Eric Schmitz
  for (int i = 0; i < n_M; i++)
    if (fabs(xv(i + 1) - xv(i)) < 1e-15)
      xp(i) = xv(i);

  for (int i = 0; i < n_M; i++)
    if (fabs(yv(i + 1) - yv(i)) < 1e-15)
      yp(i) = yv(i);

  // find all cases where projected point is inside the segment
  //#ifdef HACK
  //VectorXb idx_x = (xp.array() >= xv.head(n_M).array()) & (xp.array() <= xv.tail(n_M).array()) &
  //                 (xp.array() >= xv.tail(n_M).array()) & (xp.array() <= xv.head(n_M).array());
  //VectorXb idx_y = (yp.array() >= yv.head(n_M).array()) & (yp.array() <= yv.tail(n_M).array()) &
  //                 (yp.array() >= yv.tail(n_M).array()) & (yp.array() <= yv.head(n_M).array());
  //VectorXb idx = idx_x.cwiseProduct(idx_y);
  VectorXb idx(n_M);
  for (int i = 0; i < n_M; i++) {
    idx[i] = ((xp[i] >= xv[i] && xp[i] <= xv[i + 1]) ||
              (xp[i] >= xv[i + 1] && xp[i] <= xv[i])) &&
             ((yp[i] >= yv[i] && yp[i] <= yv[i + 1]) ||
              (yp[i] >= yv[i + 1] && yp[i] <= yv[i]));
  }

  // distance from point (x,y) to the vertices
  Eigen::VectorXd dv = ((xv.head(n_M).array() - x).square() + (yv.head(n_M).array() - y).square()).sqrt();

  const double inf = numeric_limits<double>::infinity();
  if (normvec) {
    normvec->fill(inf);
  }

  if (!idx.any()) {  // all projections are outside of polygon ribs
    int I;
    double _d = min(dv, &I);
    *d = _d;

    if (dG_dM) {                                  // find the gradient
      (*dd_dM)(0, I) = 1.0 / _d * (M(0, I) - x);  //% only relate to the closest M(I)
      (*dd_dM)(1, I) = 1.0 / _d * (M(1, I) - y);
      *dd_dx = 1.0 / _d * (pt - M.col(I)).transpose() * dpt_dx;
      *dG_dx = dptc_dx;
      dG_dM->middleCols(I * 2, 2) = -Eigen::MatrixXd::Identity(2, 2);
    }
    Eigen::Vector2d _pt_poly(xv(I), yv(I));
    if (pt_poly)
      *pt_poly = _pt_poly;

    if (contact_type)
      *contact_type = 1;

    if (normvec) {
      *normvec = _pt_poly - pt;
      *normvec = *normvec / normvec->norm();
    }
  } else {
    // distance from point (x,y) to the projection on ribs
    vector<double> _dp;
    for (int i = 0; i < idx.size(); i++)
      if (idx(i))
        _dp.push_back(sqrt((xp(i) - x) * (xp(i) - x) + (yp(i) - y) * (yp(i) - y)));
    Eigen::VectorXd dp = Eigen::VectorXd::Map(_dp.data(), _dp.size());

    int I1, I2, I;
    double min_dv, min_dp, _d;
    min_dv = min(dv, &I1);
    min_dp = min(dp, &I2);

    if (min_dv < min_dp) {
      _d = min_dv;
      I = 0;
    } else {
      _d = min_dp;
      I = 1;
    }
    *d = _d;

    if (I == 0) {
      if (dG_dM) {                                    // find the gradient
        (*dd_dM)(0, I1) = 1.0 / _d * (M(0, I1) - x);  //% only relate to the closest M(I)
        (*dd_dM)(1, I1) = 1.0 / _d * (M(1, I1) - y);
        *dd_dx = 1.0 / _d * (pt - M.col(I1)).transpose() * dpt_dx;
        *dG_dx = dptc_dx;
        dG_dM->middleCols(I1 * 2, 2) = -Eigen::MatrixXd::Identity(2, 2);
      }
      Eigen::Vector2d _pt_poly(xv(I1), yv(I1));
      if (pt_poly)
        *pt_poly = _pt_poly;

      if (contact_type)
        *contact_type = -1;

      if (normvec) {
        *normvec = _pt_poly - pt;
        *normvec = *normvec / normvec->norm();
      }
    } else {
      Eigen::VectorXi idxs = find(idx);
      int idxsI2 = idxs(I2);
      int idxsI2_next = (idxsI2 + 1) % n_M;
      if (dG_dM) {  // find the gradient
        Eigen::Vector2d M1 = M.col(idxsI2);
        Eigen::Vector2d M2 = M.col(idxsI2_next);
        double M1x = M1(0), M1y = M1(1);
        double M2x = M2(0), M2y = M2(1);
        double Px = pt(0), Py = pt(1);
        double f = (M1 - M2).dot(pt - M2);

        Eigen::Vector2d df_dM1 = pt - M2;
        double df_dM1x = df_dM1(0);
        double df_dM1y = df_dM1(1);

        Eigen::Vector2d df_dM2 = -1 * (pt - M2) - (M1 - M2);
        double df_dM2x = df_dM2(0);
        double df_dM2y = df_dM2(1);

        double dg_dM1x = 2 * (M1x - M2x);
        double dg_dM1y = 2 * (M1y - M2y);
        double dg_dM2x = -2 * (M1x - M2x);
        double dg_dM2y = -2 * (M1y - M2y);
        double g = (M1 - M2).squaredNorm();
        double M1x_M2x = M1x - M2x;
        double M1y_M2y = M1y - M2y;

        double sqg = g * g;
        double dD1_dM1x = -(g * (f + df_dM1x * M1x_M2x) - dg_dM1x * f * M1x_M2x) / sqg;       //% dD1_dM1x
        double dD1_dM1y = -(g * df_dM1y * M1x_M2x - dg_dM1y * f * M1x_M2x) / sqg;             //% dD1_dM1y
        double dD1_dM2x = -(g * (-f + df_dM2x * M1x_M2x) - dg_dM2x * f * M1x_M2x) / sqg - 1;  //% dD1_dM2x
        double dD1_dM2y = -(g * df_dM2y * M1x_M2x - dg_dM2y * f * M1x_M2x) / sqg;             //% dD1_dM2y

        double dD2_dM1x = -(g * df_dM1x * M1y_M2y - dg_dM1x * f * M1y_M2y) / sqg;            // % dD2_dM1x
        double dD2_dM1y = -(g * (df_dM1y * M1y_M2y + f) - dg_dM1y * f * M1y_M2y) / sqg;      //% dD2_dM1y
        double dD2_dM2x = -(g * df_dM2x * M1y_M2y - dg_dM2x * f * M1y_M2y) / sqg;            //% dD2_dM2x
        double dD2_dM2y = -(g * (df_dM2y * M1y_M2y - f) - dg_dM2y * f * M1y_M2y) / sqg - 1;  //% dD2_dM2y

        double D1 = x - xp(idxsI2);
        double D2 = y - yp(idxsI2);
        double dd_dM1x = 1 / _d * (D1 * dD1_dM1x + D2 * dD2_dM1x);
        double dd_dM1y = 1 / _d * (D1 * dD1_dM1y + D2 * dD2_dM1y);
        double dd_dM2x = 1 / _d * (D1 * dD1_dM2x + D2 * dD2_dM2x);
        double dd_dM2y = 1 / _d * (D1 * dD1_dM2y + D2 * dD2_dM2y);

        (*dd_dM)(0, idxsI2) = dd_dM1x;
        (*dd_dM)(1, idxsI2) = dd_dM1y;
        (*dd_dM)(0, idxsI2_next) = dd_dM2x;
        (*dd_dM)(1, idxsI2_next) = dd_dM2y;

        Eigen::MatrixXd dD1_dx = dpt_dx.row(0) - (M1 - M2).transpose() * dpt_dx / g * M1x_M2x;
        Eigen::MatrixXd dD2_dx = dpt_dx.row(1) - (M1 - M2).transpose() * dpt_dx / g * M1y_M2y;

        *dd_dx = 1 / _d * (D1 * dD1_dx + D2 * dD2_dx);

        *dG_dx = dptc_dx - ((M1 - M2) / g) * ((M1 - M2).transpose() * dpt_dx);
        Eigen::Matrix2d dG_dM1;
        dG_dM1 << dD1_dM1x, dD1_dM1y, dD2_dM1x, dD2_dM1y;
        Eigen::Matrix2d dG_dM2;
        dG_dM2 << dD1_dM2x, dD1_dM2y, dD2_dM2x, dD2_dM2y;

        dG_dM->middleCols(idxsI2 * 2, 2) = dG_dM1;
        dG_dM->middleCols(idxsI2_next * 2, 2) = dG_dM2;
      }

      if (pt_poly)
        *pt_poly << xp(idxsI2), yp(idxsI2);

      if (contact_type)
        *contact_type = 2;  //edge

      if (normvec) {
        *normvec << -(yv(idxsI2_next) - yv(idxs(I2))), (xv(idxsI2_next) - xv(idxsI2));
        *normvec = *normvec / normvec->norm();
      }
    }
  }
  //#endif
}

void shape__probe_obj__find_norm(const Eigen::Vector2d& pt_obj,
                                 const Eigen::MatrixXd& M,

                                 Eigen::Vector2d* normvec) {
  double d;
  p_poly_dist(pt_obj, M, Eigen::MatrixXd(), Eigen::MatrixXd(),
              &d, NULL, NULL, NULL, NULL, NULL, NULL, normvec);
}