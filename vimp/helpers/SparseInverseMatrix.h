/**
 * @file SparseInverseMatrix.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Algorithm to calculate the inverse of a sparse matrix
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef VIMP_SPARSEINVERSEMATRIX_H
#define VIMP_SPARSEINVERSEMATRIX_H

#endif /// VIMP_SPARSEINVERSEMATRIX_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <iostream>
#include "data_io.h"

typedef Eigen::SparseMatrix<double> SpMatrix;
using namespace Eigen;
using namespace std;

namespace vimp{
       struct dense_inverser{
        dense_inverser(){}
        dense_inverser(const MatrixXd& _input_dense, int dim_conf):
        T_(_input_dense.cols()),
        input_dense_{_input_dense}{
            
            MatrixIO m_io;

            dim_conf_ = dim_conf;
            width_ = 4*dim_conf;

        }

        /**
         * @brief Inverse with given input
         * 
         * @param input matrix want to inverse.
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd inverse(const MatrixXd& input_matrix) const{
            
            LLT<MatrixXd, Eigen::Lower> choleskyLDLT = input_matrix.llt();
            MatrixXd L = choleskyLDLT.matrixL();
            MatrixXd D = L.diagonal().array().square().matrix().asDiagonal();
            MatrixXd inv_sqrt_D = L.diagonal().array().inverse().matrix().asDiagonal();

            L = L * inv_sqrt_D;

            MatrixXd inverse = MatrixXd::Zero(T_, T_);

            /// 1 dimensional case
            if (L.cols()==1){
                assert(input_matrix(0,0) != 0);
                return MatrixXd{MatrixXd::Constant(1, 1, 1 / input_matrix(0,0))};
            }

            inverse(T_-1, T_-1) = 1.0 / D(T_-1, T_-1);
            for (int j=T_-2; j>=0; j--){
                for (int i=min(j+1+width_,T_-1); i>=j; i--){
                    /// non-zero entries of L, tri-diagonal
                    double value = 0;
                    if (i==j){value = 1/D(i,i);}

                    for (int l = j+1; l < min(j+1+width_, T_); l++){
                        value = value - inverse(i, l) * L(l, j);
                    }
                    inverse(i, j) = value;
                    inverse(j, i) = value;
                }
            }
            return inverse;
        }

        /**
         * @brief Default inverser
         * 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd inverse() const{
            return inverse(input_dense_);
        }

    private:
        int T_;
        MatrixXd input_dense_;
        int dim_conf_;
        int width_;
    };

}
