#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using std::vector;

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Define local variables to work on
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  // Define Radar variables
  float rho = sqrt(pow(px,2)+pow(py,2));
  float phi = atan2(py,px); // The use of atan2 return values between -pi and pi instead of -pi/2 and pi/2 for atan
  float rhodot;

  //Avoid Division by 0
  if (fabs(rho) < 0.0001) {
    rhodot = 0;
  } 
  else rhodot = (vx*px + vy*py)/rho;

  // Create a function that contains our predictions
  VectorXd z_pred (3);
  z_pred << rho, phi, rhodot;

  // y is the error vector (actual values - estimations)
  VectorXd y = z - z_pred;

//Normalizing the angles so y[1] always stays between -Pi and +Pi
    while (y[1] > M_PI){
       y[1] -= (2 * M_PI);
    }
    while (y[1] < - M_PI){
       y[1] += (2 * M_PI);
    }
  
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