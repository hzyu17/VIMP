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

#include "../base/Matrix.h"
#include "Dynamics.h"
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
    // compressed 3D matrices
    // shape: 
    // _At (_nx*_nx, _nt)
    // _Bt (_nx*_nu, _nt)
    // _at (_nx, _nt)
    Matrix3D _At, _Bt, _at;
    int _nx, _nu, _nt;
};


class LTI: public LinearDynamics{
public:
    LTI(){}
    LTI(int nx, int nu, int nt):LinearDynamics(nx, nu, nt){}

    LTI(int nx, int nu, int nt, 
        const MatrixXd& A, 
        const MatrixXd& B, 
        const MatrixXd& a):LinearDynamics(nx, nu, nt, 
                                        _ei.replicate3d(A, nt), 
                                        _ei.replicate3d(B, nt), 
                                        _ei.replicate3d(a, nt)){}

    void update_matrices(const MatrixXd& A, const MatrixXd& B, const MatrixXd& a){
        _At = _ei.replicate3d(A, _nt); 
        _Bt = _ei.replicate3d(B, _nt); 
        _at = _ei.replicate3d(a, _nt);
    }

};
}