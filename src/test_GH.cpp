//
// Created by hongzhe on 5/11/22.
//
#include "../include/GaussianHermite.h"
#include <assert.h>

double gx(const VectorXd& x){
    return x.norm()*x.norm();
}


int main(){
    int dim = 3;
    VectorXd m = VectorXd::Ones(dim);
    MatrixXd P = MatrixXd::Identity(dim, dim) * 1.2;
    GaussHermite<std::function<double(const VectorXd&)>> gausshermite(10, dim, m, P, gx);
    VectorXd weights = gausshermite.getWeights();
    cout << weights << endl;
    double integral{gausshermite.Integrate()};
    cout << "integrate " << endl << integral << endl;
}