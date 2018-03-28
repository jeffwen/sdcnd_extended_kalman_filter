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
  
  // predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  // update the state by using Kalman Filter equations
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  KalmanFilter::UpdateUniversal(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  //update the state by using Extended Kalman Filter equations

  //recover state parameters
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  
  double px2_py2 =  px*px+py*py;
  double sqrt_px2_py2 =  sqrt(px2_py2);
  
  //check division by zero
  if(fabs(px2_py2) < 0.0001){
    std::cout << "Update EKF Error - Division by Zero" << '\n';
  }
  
  H_conv_ = VectorXd(3);
  H_conv_ << sqrt_px2_py2, atan2(py, px), (px*vx+py*vy)/sqrt_px2_py2;
  
  std::cout << "raw_measure" << z <<'\n';
  std::cout << "H_conv_" << H_conv_ <<'\n';
  
  VectorXd y = z - H_conv_;
  
  // make sure that the y is in between -pi and pi
  while (y[1] < -M_PI || y[1] > M_PI){
    if (y[1] < -M_PI){
      y[1] += 2*M_PI;
    } else {
      y[1] -= 2*M_PI;
    }
  }
  
  KalmanFilter::UpdateUniversal(y);
  
}

void KalmanFilter::UpdateUniversal(const VectorXd &y) {

  // actually using H jacobian here because of radar
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
