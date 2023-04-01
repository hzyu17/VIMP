/**
 * @file Matrix.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Define some special matrix and vector classes.
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <Eigen/Dense>

using namespace Eigen;

namespace vimp{

class Matrix3D : public MatrixXd{
public:
    Matrix3D(){}
    Matrix3D(int row, int col, int nt):MatrixXd(row*col, nt) {}    
    Matrix3D(const Matrix3D& mat): Eigen::MatrixXd(mat) {}
    Matrix3D(const MatrixXd & mat): MatrixXd(mat){}

    // Overloaded assignment operator that takes an Eigen::MatrixXd as input
    Matrix3D& operator=(const Eigen::MatrixXd& other) {
        Eigen::MatrixXd::operator=(other); // call base class assignment operator
        // additional custom logic for MyMatrix
        return *this;
    }
};

class Vector3D : public VectorXd{
public:
    Vector3D(){}
    Vector3D(int row, int nt):VectorXd(row, nt){}   

};


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