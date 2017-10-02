#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  VectorXd y = z - H_ * x_;
  UpdateCommon(y);
  //MatrixXd S = H_ * P_* H_.transpose() + R_;
  //MatrixXd K = P_ * H_.transpose() * S.inverse();
  //x_ = x_ + K * y;
  //P_ -= K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  if (px == 0 && py == 0) {
    cout << "Division by zero!" << endl;
  }
  else {
    float square = px * px + py * py;
    float root = pow(square, 0.5);
    float rho = root;
    float phi = atan2(py, px);
    float rho_dot = (px * vx + py * vy) / root;
    VectorXd h = VectorXd(3);
    h << rho, phi, rho_dot;
    VectorXd y = z - h;
    // normalize angle
    while (y[1]> M_PI) y[1] -= 2 * M_PI;
    while (y[1]<-M_PI) y[1] += 2 * M_PI;

    UpdateCommon(y);
    //MatrixXd S = H_ * P_* H_.transpose() + R_;
    //MatrixXd K = P_ * H_.transpose() * S.inverse();
    //x_ = x_ + K * y;
    //P_ = (I - K * H_) * P_;
  }
}

void KalmanFilter::UpdateCommon(const VectorXd &y) {
  const MatrixXd PHt = P_ * H_.transpose();
  const MatrixXd S = H_ * PHt + R_;
  const MatrixXd K = PHt * S.inverse();

  x_ += K * y;
  P_ -= K * H_ * P_;
}
