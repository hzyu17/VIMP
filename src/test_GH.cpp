//
// Created by hongzhe on 5/11/22.
//
#include "../include/GaussHermite.h"

MatrixXd gx_1d(const VectorXd& x){
    return MatrixXd{MatrixXd::Constant(1, 1, x.norm()*x.norm())};
}

MatrixXd gx_2d(const VectorXd& x){
    return MatrixXd{x};
}

MatrixXd gx_3d(const VectorXd& x){
    return MatrixXd{x*x.transpose().eval()};
}


int main(){
    int dim = 2;
    VectorXd m = VectorXd::Ones(dim);
    MatrixXd P = MatrixXd::Identity(dim, dim) * 1.2;
    GaussHermite<std::function<MatrixXd(const VectorXd&)>> gausshermite(10, dim, m, P, gx_3d);

    // test lambda function parameter
    std::function<MatrixXd(const VectorXd&)>gx_2d_new = [&](const VectorXd& x){return MatrixXd{x};};
    gausshermite.update_integrand(gx_2d_new);
    MatrixXd integral2{gausshermite.Integrate()};
    cout << "integrate " << endl << integral2 << endl;

    std::function<MatrixXd(const VectorXd&)>gx_3d_new = [&](const VectorXd& x){return MatrixXd{x* x.transpose().eval()};};
    gausshermite.update_integrand(gx_3d_new);
    VectorXd weights = gausshermite.getWeights();
    cout << weights << endl;
    MatrixXd integral3{gausshermite.Integrate()};
    cout << "integrate " << endl << integral3 << endl;
    return 0;
}
