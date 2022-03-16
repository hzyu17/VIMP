//
// Created by hongzhe on 3/13/22.
//
/// Algorithm to calculate the inverse of a sparse matrix

#ifndef MPVI_SPARSEINVERSEMATRIX_H
#define MPVI_SPARSEINVERSEMATRIX_H

#endif //MPVI_SPARSEINVERSEMATRIX_H
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/3rdparty/Eigen/Eigen/Sparse>
#include <gtsam/3rdparty/Eigen/Eigen/SparseCore>
#include <gtsam/3rdparty/Eigen/Eigen/SparseCholesky>
#include <iostream>

typedef Eigen::SparseMatrix<double> SpMatrix;
using namespace Eigen;
using namespace std;

namespace SparseInverse{
    struct sparse_inverser{
        sparse_inverser(const SpMatrix& _input_sparse):
        K(_input_sparse.cols()){
            inverse_.resize(K, K);
            SimplicialLDLT<SpMatrix, Eigen::Lower, Eigen::NaturalOrdering<int>> choleskyLDLT{_input_sparse};
            L_ = choleskyLDLT.matrixL();
            D_ = choleskyLDLT.vectorD().asDiagonal();

            cout << "_input_sparse " << endl << _input_sparse << endl;
            cout << "D matrix " << endl << choleskyLDLT.vectorD() << endl;
            cout << "product LDLT " << endl << L_ * D_ * L_.transpose() << endl;
        }

        Eigen::MatrixXd inverse(){
            inverse_.setZero();
            inverse_(K-1, K-1) = 1.0 / D_.coeff(K-1, K-1);
            inverse_(K-1, K-2) = -inverse_(K-1, K-1) * L_.coeff(K-1, K-2);
            for (int k=K-2; k>=0; k--){
                for (int j=K-1; j>=0; j--){
                    /// non-zero entries of L for the tri-diagonal precision matrix
                    double value = -inverse_(j, k+1) * L_.coeff(k+1, k);
                    if (j==k){
                        value += 1.0 / D_.coeff(j, j);
                    }
                    inverse_(j, k) = value;
                    inverse_(k, j) = value;
                }
            }

            return inverse_;
        }

        void update_sparse_precision(const SpMatrix& new_precision){
            assert(new_precision.cols() == L_.cols());
            SimplicialLDLT<SpMatrix, Eigen::Lower, Eigen::NaturalOrdering<int>> choleskyLDLT{new_precision};
            L_ = choleskyLDLT.matrixL();
            D_ = choleskyLDLT.vectorD().asDiagonal();

        }

        Eigen::MatrixXd inverse(const SpMatrix& input_matrix){
            update_sparse_precision(input_matrix);
            return inverse();
        }

        MatrixXd inverse_;
        SpMatrix L_;
        SpMatrix D_;
        int K;
    };

    struct dense_inverser{
        dense_inverser(const MatrixXd& _input_dense):
                K(_input_dense.cols()){
            inverse_.resize(K, K);
            LDLT<MatrixXd, Eigen::Lower> choleskyLDLT{_input_dense};
            L_ = choleskyLDLT.matrixL();
            D_ = choleskyLDLT.vectorD().asDiagonal();

            cout << "_input_dense " << endl << _input_dense << endl;
            cout << "D matrix " << endl << choleskyLDLT.vectorD() << endl;
            cout << "product LDLT " << endl << L_ * D_ * L_.transpose() << endl;
        }

        Eigen::MatrixXd inverse(){
            inverse_.setZero();
            inverse_(K-1, K-1) = 1.0 / D_(K-1, K-1);
            inverse_(K-1, K-2) = -inverse_(K-1, K-1) * L_(K-1, K-2);
            for (int k=K-2; k>=0; k--){
                for (int j=K-1; j>=0; j--){
                    /// non-zero entries of L for the tri-diagonal precision matrix
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

        void update_precision(const MatrixXd& new_precision){
            assert(new_precision.cols() == L_.cols());
            LDLT<MatrixXd, Eigen::Lower> choleskyLDLT{new_precision};
            L_ = choleskyLDLT.matrixL();
            D_ = choleskyLDLT.vectorD().asDiagonal();

        }

        Eigen::MatrixXd inverse(const MatrixXd& input_matrix){
            update_precision(input_matrix);
            return inverse();
        }

        MatrixXd inverse_;
        MatrixXd L_;
        MatrixXd D_;
        int K;
    };

}
