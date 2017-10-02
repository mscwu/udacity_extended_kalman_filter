#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Estimations must be non-empty and the same length as ground truth!" << endl;
  }
  else {
    for (unsigned int i=0; i<estimations.size(); ++i) {
      VectorXd residual = estimations[i] - ground_truth[i];
      residual = residual.array() * residual.array();
      rmse += residual;
    }

    rmse = rmse.array() / estimations.size();
    rmse = rmse.array().sqrt();    
  }
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  MatrixXd Hj(3,4);
  Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
  //check division by zero
  if (px==0 && py==0) {
      cout << "Division by zero!" << endl;
  }
  else {
      float square = px * px + py * py;
      float root = pow(square, 0.5);
      Hj(0,0) = px / root;
      Hj(0,1) = py / root;
      Hj(1,0) = -py / square;
      Hj(1,1) = px / square;
      Hj(2,0) = py * (vx * py - vy * px) / pow(square, 1.5);
      Hj(2,1) = px * (px * vy - py * vx) / pow(square, 1.5);
      Hj(2,2) = Hj(0,0);
      Hj(2,3) = Hj(0,1);
  }
  return Hj;
}
