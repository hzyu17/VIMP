/**
 * @file LinearDynamics.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The definition of linear dynamics. 
 * Here we put it as a special nonlinear dynamics, which differs from the control literature.
 * @version 0.1
 * @date 2023-03-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "helpers/Matrix.h"
#include "dynamics/Dynamics.h"
#include <Eigen/Dense>

using namespace Eigen;

namespace vimp{

class LinearDynamics: public Dynamics{

public:
    LinearDynamics(){}
    LinearDynamics(int nx, 
                   int nu, 
                   int nt): Dynamics(nx, nu, nt), _At(Matrix3D(nx, nx, nt)), _Bt(Matrix3D(nx, nu, nt)), _at(Matrix3D(nx, 1, nt)){}  

    LinearDynamics(int nx, 
                   int nu, 
                   int nt, 
                   const Matrix3D& At, 
                   const Matrix3D Bt, 
                   const Matrix3D at): Dynamics(nx, nu, nt), _At(At), _Bt(Bt), _at(at){}             

    inline Matrix3D At(){return _At;}
    inline Matrix3D Bt(){return _Bt;}
    inline Matrix3D at(){return _at;}

protected:
    Matrix3D _At, _Bt, _at;
};


class LTI: public LinearDynamics{
public:
    LTI(){}
    LTI(int nx, int nu, int nt):LinearDynamics(nx, nu, nt), _A(nx,nx), _B(nx,nu), _a(nx,1){}

    LTI(int nx, int nu, int nt, 
        const MatrixXd& A, 
        const MatrixXd& B, 
        const MatrixXd& a):LinearDynamics(nx, nu, nt, 
                                        _ei.replicate3d(A, nt), 
                                        _ei.replicate3d(B, nt), 
                                        _ei.replicate3d(a, nt)), _A(A), _B(B), _a(a){}

    void update_matrices(const MatrixXd& A, const MatrixXd& B, const VectorXd& a){
        _A = A;
        _B = B;
        _a = a;
        _At = _ei.replicate3d(A, _nt); 
        _Bt = _ei.replicate3d(B, _nt); 
        _at = _ei.replicate3d(a, _nt);
    }

    inline Matrix3D A0(){return _A;}
    inline Matrix3D B0(){return _B;}
    inline Matrix3D a0(){return _a;}

protected:
    MatrixXd _A, _B, _a;

};

class ConstantVelDynamics: public LTI{

public:
    ConstantVelDynamics(){}
    ConstantVelDynamics(int nx, int nu, int nt):LTI(nx, nu, nt)
    {   
        if (nx/2 != nu){ std::cout << "wrong dimensions " << std::endl; }
        MatrixXd A(_nx, _nx), B(_nx, _nu), a(_nx, 1);
        A.setZero(); B.setZero(); a.setZero();
        A.block(0, _nx/2, _nx/2, _nx/2) = MatrixXd::Identity(_nx/2, _nx/2);
        B.block(_nx/2, 0, _nu, _nu) = MatrixXd::Identity(_nu, _nu);
        update_matrices(A, B, a);
    }

};

}