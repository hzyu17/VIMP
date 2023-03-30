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

#include <Eigen/Dense>
#include "../helpers/eigen_wrapper.h"

using namespace Eigen;

namespace vimp{

class LinearDynamics{

public:
    LinearDynamics(){}
    LinearDynamics(int nx, 
                   int nu, 
                   int nt, 
                   const Matrix3D& At, 
                   const Matrix3D Bt, 
                   const Matrix3D at): _At(At), _Bt(Bt), _at(at){}             

    inline Matrix3D At(){return _At;}
    inline Matrix3D Bt(){return _Bt;}
    inline Matrix3D at(){return _at;}

private:
    int _nx, _nu, _nt;

    // compressed 3D matrices
    // shape: 
    // _At (_nx*_nx, _nt)
    // _Bt (_nx*_nu, _nt)
    // _at (_nx, _nt)
    Matrix3D _At, _Bt, _at;
};


class LTI: public LinearDynamics{
public:
    LTI(){}
    LTI(int nt, 
        const MatrixXd A, 
        const MatrixXd B, 
        const MatrixXd a):LinearDynamics(A.rows(), 
                                        B.cols(), 
                                        nt, 
                                        _ei.replicate3d(A, nt), 
                                        _ei.replicate3d(B, nt), 
                                        _ei.replicate3d(a, nt)){}

private:
    EigenWrapper _ei;
};
}