#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
 
  VectorXd s(4);
  VectorXd rsme(4);
  VectorXd mean(4);
  for(int i=0;i<estimations.size();i++){
  	VectorXd err = (estimations[i] - ground_truth[i]).array() * (estimations[i] - ground_truth[i]).array();
  	s += err;
  }
  cout << estimations[estimations.size()-1] << endl;
  cout << ground_truth[estimations.size()-1] << endl;
  mean = s/estimations.size();
  rsme = mean.array().sqrt();
  cout << rsme << endl;
  return rsme;
}