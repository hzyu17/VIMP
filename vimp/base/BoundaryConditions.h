/**
 * @file BoundaryConditions.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Definition of the boundary conditions for covariance steering problem.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<Eigen/Dense>

namespace vimp{

class BoundaryConditions{
public:
    BoundaryConditions(){}

    BoundaryConditions(int nx):_nx(nx), 
                               _m0(Eigen::VectorXd::Zero(_nx)), 
                               _mT(Eigen::VectorXd::Ones(_nx)),
                               _Sig0(0.01*Eigen::MatrixXd::Identity(_nx, _nx)),
                               _SigT(0.01*Eigen::MatrixXd::Identity(_nx, _nx)){}

    BoundaryConditions(int nx, const Eigen::VectorXd& m0, 
                               const Eigen::VectorXd& mT,
                               const Eigen::MatrixXd& Sig0,
                               const Eigen::MatrixXd& SigT):_nx(nx), 
                               _m0(m0), 
                               _mT(mT),
                               _Sig0(Sig0),
                               _SigT(SigT){}

    Eigen::VectorXd m0(){
        return _m0;
    }

    Eigen::VectorXd mT(){
        return _mT;
    }

    Eigen::MatrixXd Sig0(){
        return _Sig0;
    }

    Eigen::MatrixXd SigT(){
        return _SigT;
    }

protected:
    int _nx;
    Eigen::VectorXd _m0, _mT;
    Eigen::MatrixXd _Sig0, _SigT;
};

}