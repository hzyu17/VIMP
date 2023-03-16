/**
 * @file linear_dynamics.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The linear dynamics (A, B): \dot x = Ax + Bu
 * @version 0.1
 * @date 2023-02-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../3rd-part/eigen-3.4.0/unsupported/Eigen/MatrixFunctions"


using namespace Eigen;

namespace vimp
{
class LinearSystem{
public:
    LinearSystem(){};
    LinearSystem(const MatrixXd& A, const MatrixXd& B=MatrixXd::Zero(1, 1)):_A(A), _B(B), 
                                                       _dim(A.cols()), 
                                                       _dim_control(B.cols()),
                                                       _Phi(MatrixXd::Zero(_dim, _dim)){}

public:
    MatrixXd Phi(double t){
        compute_transition_matrix(t);
        return _Phi;
    }

    MatrixXd A(){
        return _A;
    }

private:
    MatrixXd _A, _B;
    VectorXd _b; // drift term
    MatrixXd _Phi; // state transition matrix
    int _dim; // state dimension
    int _dim_control; //control dimension

    void compute_transition_matrix(const double & t){
        _Phi = _A.exp()*t;
    }

};
} // namespace vimp
