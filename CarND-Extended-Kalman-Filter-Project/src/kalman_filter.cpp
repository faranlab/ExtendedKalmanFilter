#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

// This code has many segments taken from the lessons

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  // new state
  MatrixXd I;
  I = MatrixXd::Identity(4, 4);
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // pi
  const double pi = atan(1)*4;
  const double pi2 = (atan(1)*4)*2;

  VectorXd y = z - h_;
  if (y(1)>0)
    y(1) = fmod(y(1)+pi, pi2)-pi;
  else
    y(1) = fmod(y(1)-pi, pi2)+pi;

  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Hjt * Si;
  // new state
  MatrixXd I;
  I = MatrixXd::Identity(4, 4);
  x_ = x_ + (K * y);
  P_ = (I - K * Hj_) * P_;
}
