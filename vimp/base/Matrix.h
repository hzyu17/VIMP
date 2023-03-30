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
#include <Eigen/Dense>

using namespace Eigen;

namespace vimp{

class Matrix3D : public MatrixXd{
public:
    Matrix3D(){}
    Matrix3D(int row, int col, int nt):MatrixXd(row*col, nt){}    
    Matrix3D(const MatrixXd & mat): MatrixXd(mat){}
};

class Vector3D : public VectorXd{
public:
    Vector3D(){}
    Vector3D(int row, int nt):VectorXd(row, nt){}   

};

}