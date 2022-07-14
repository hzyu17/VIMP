/**
 * @file SparseInverseMatrix.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/// Algorithm to calculate the inverse of a sparse matrix

#ifndef MPVI_SPARSEINVERSEMATRIX_H
#define MPVI_SPARSEINVERSEMATRIX_H

#endif /// MPVI_SPARSEINVERSEMATRIX_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <iostream>

typedef Eigen::SparseMatrix<double> SpMatrix;
using namespace Eigen;
using namespace std;

namespace SparseInverse{
    struct sparse_inverser{
        sparse_inverser(const SpMatrix& _input_sparse):
        K(_input_sparse.cols()){
            input_matrix_ = _input_sparse;
            inverse_.resize(K, K);
            SimplicialLLT<SpMatrix, Eigen::Lower, Eigen::NaturalOrdering<int>> choleskyLLT{_input_sparse};
            L_ = choleskyLLT.matrixL();
            D_ = L_.diagonal().array().square().matrix().asDiagonal();
            inv_sqrt_D_ = L_.diagonal().array().inverse().matrix().asDiagonal();
            L_ = L_ * inv_sqrt_D_;

        }

        Eigen::MatrixXd inverse(){
            cout << "true inverse " << endl << input_matrix_.toDense().inverse() << endl;
            inverse_(K-1, K-1) = 1.0 / D_.coeff(K-1, K-1);
            inverse_(K-1, K-2) = -inverse_(K-1, K-1) * L_.coeff(K-1, K-2);
            for (int k=K-2; k>=0; k--){
                for (int j=K-1; j>=k; j--){
                    /// non-zero entries of L for the tri-diagonal precision matrix
                    double value = -inverse_(j, k+1) * L_.coeff(k+1, k);
                    if (j==k){
                        value += 1.0 / D_.coeff(j, j);
                    }
                    inverse_(j, k) = value;
                    inverse_(k, j) = value;
                }
            }
            cout << "inverse calculated " << endl << inverse_ << endl;
            return inverse_;
        }

        void update_sparse_precision(const SpMatrix& new_precision){
            assert(new_precision.cols() == L_.cols());

            input_matrix_ = new_precision;
            SimplicialLLT<SpMatrix, Eigen::Lower, Eigen::NaturalOrdering<int>> choleskyLLT{new_precision};
            L_ = choleskyLLT.matrixL();
            D_ = L_.diagonal().array().square().matrix().asDiagonal();
            inv_sqrt_D_ = L_.diagonal().array().inverse().matrix().asDiagonal();
            L_ = L_ * inv_sqrt_D_;

            cout << "L_" << endl << L_.toDense() << endl;
            cout << "precision" << endl << new_precision.toDense() << endl;
            cout << "LDLT" << endl << (L_ * D_ * L_.transpose()).toDense() << endl;

            cout << "aa" << endl;
        }

        Eigen::MatrixXd inverse(const SpMatrix& input_matrix){
            update_sparse_precision(input_matrix);
            return inverse();
        }

        MatrixXd inverse_;
        SpMatrix L_;
        SpMatrix input_matrix_;
        SpMatrix D_;
        int K;
        SpMatrix inv_sqrt_D_;
    };

    struct dense_inverser{
        dense_inverser(const MatrixXd& _input_dense):K_(_input_dense.cols()){
            inverse_.resize(K_, K_);
            input_dense_ = _input_dense;
            LLT<MatrixXd, Eigen::Lower> choleskyLDLT = _input_dense.llt();
            L_ = choleskyLDLT.matrixL();

            D_ = L_.diagonal().array().square().matrix().asDiagonal();
            inv_sqrt_D_ = D_.diagonal().array().sqrt().inverse().matrix().asDiagonal();
            L_ = L_ * inv_sqrt_D_;
        }

        Eigen::MatrixXd inverse(){

            LLT<MatrixXd, Eigen::Lower> choleskyLDLT = input_dense_.llt();

            L_ = choleskyLDLT.matrixL();

            D_ = L_.diagonal().array().square().matrix().asDiagonal();
            inv_sqrt_D_ = L_.diagonal().array().inverse().matrix().asDiagonal();

            L_ = L_ * inv_sqrt_D_;

            inverse_(K_-1, K_-1) = 1.0 / D_(K_-1, K_-1);
            inverse_(K_-1, K_-2) = -inverse_(K_-1, K_-1) * L_(K_-1, K_-2);
            for (int k=K_-2; k>=0; k--){
                for (int j=K_-1; j>=k; j--){
                    /// non-zero entries of L, tri-diagonal
                    double value = -inverse_(j, k+1) * L_(k+1, k);
                    if (j==k){
                        value += 1.0 / D_(j, j);
                    }
                    inverse_(j, k) = value;
                    inverse_(k, j) = value;
                }
            }
            return inverse_;
        }

        Eigen::MatrixXd inverse(const MatrixXd& input_matrix){
            assert(input_matrix.cols() == L_.cols());
            input_dense_ = input_matrix;
            return inverse();
        }

        MatrixXd inverse_;
        MatrixXd L_;
        MatrixXd D_;
        int K_;
        MatrixXd input_dense_;
        MatrixXd inv_sqrt_D_;
    };

}
