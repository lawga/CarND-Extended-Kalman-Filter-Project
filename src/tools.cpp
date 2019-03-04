#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
	VectorXd rmse(4);
  rmse << 0,0,0,0;

  // YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if(estimations.size() == 0){
      cout<<"the estimation vector size should not be zero"<<endl;
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()){
      cout<<"the estimation vector size should equal ground truth vector size"<<endl;
      return rmse;
  }
  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
      
    VectorXd residuals = estimations[i] - ground_truth[i];
    residuals = residuals.array() * residuals.array();
    rmse += residuals;
  }

  // calculate the mean
    rmse = rmse/estimations.size();
  // calculate the squared root
    rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
	// recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // YOUR CODE HERE 
  float px2 = px*px;
  float py2 =py*py;
  float sumSQR = px2+py2;
  float sqrtHalf = sqrt(sumSQR);
  float sqrtthreeHalfs = sqrtHalf*sumSQR;
  

  // check division by zero
  if (std::abs(sumSQR) < 0.001){
      cout<<"Error - Division by zero"<<endl;
      return Hj;
  };
  // compute the Jacobian matrix
  Hj<<px/sqrtHalf,											py/sqrtHalf,										 0,					 	0,
      -py/(sumSQR),											px/(sumSQR),										 0, 					0,
      py*(vx*py-vy*px)/sqrtthreeHalfs,	px*(vy*px-vx*py)/sqrtthreeHalfs, px/sqrtHalf, py/sqrtHalf;

  return Hj;
}
