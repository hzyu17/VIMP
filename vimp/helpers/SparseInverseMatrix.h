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

typedef Eigen::SparseMatrix<double> SpMatrix;
using namespace Eigen;
using namespace std;

namespace vimp{
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
                    double value = 0;
                    for (int l=1; l<K-k-1; l++){
                        value = value - inverse_(j, k+l) * L_.coeff(k+l, k);
                    }
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
        dense_inverser(){}
        dense_inverser(const MatrixXd& _input_dense):
        K_(_input_dense.cols()),
        input_dense_{_input_dense}{
            inverse_.resize(K_, K_);
            LLT<MatrixXd, Eigen::Lower> choleskyLDLT = _input_dense.llt();
            L_ = choleskyLDLT.matrixL();

            D_ = L_.diagonal().array().square().matrix().asDiagonal();
            inv_sqrt_D_ = D_.diagonal().array().sqrt().inverse().matrix().asDiagonal();
            L_ = L_ * inv_sqrt_D_;
        }

        /**
         * @brief Inverse with given input
         * 
         * @param input matrix want to inverse.
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd inverse(const MatrixXd& input_matrix){

            assert(input_matrix.cols() == L_.cols());

            /// 1 dimensional case
            if (L_.cols()==1){
                assert(input_matrix(0,0) != 0);
                return MatrixXd{MatrixXd::Constant(1, 1, 1 / input_matrix(0,0))};
            }

            LLT<MatrixXd, Eigen::Lower> choleskyLDLT = input_matrix.llt();

            L_ = choleskyLDLT.matrixL();

            D_ = L_.diagonal().array().square().matrix().asDiagonal();
            inv_sqrt_D_ = L_.diagonal().array().inverse().matrix().asDiagonal();

            L_ = L_ * inv_sqrt_D_;

            inverse_(K_-1, K_-1) = 1.0 / D_(K_-1, K_-1);
            inverse_(K_-1, K_-2) = -inverse_(K_-1, K_-1) * L_(K_-1, K_-2);
            for (int k=K_-2; k>=0; k--){
                for (int j=K_-1; j>=k; j--){
                    /// non-zero entries of L, tri-diagonal
                    double value = 0;
                    for (int l = k+1; l < K_; l++){
                        value = value - inverse_(j, l) * L_(l, k);
                    }
                    
                    if (j==k){
                        value += 1.0 / D_(j, j);
                    }
                    inverse_(j, k) = value;
                    inverse_(k, j) = value;
                }
            }
            return inverse_;
        }

        // /**
        //  * @brief log det function
        //  * 
        //  * @param input_matrix 
        //  * @return double 
        //  */
        // double logdetD(const MatrixXd& input_matrix){
        //     assert(input_matrix.cols() == L_.cols());

        //     /// 1 dimensional case
        //     if (L_.cols()==1){
        //         return input_matrix(0,0);
        //     }

        //     LLT<MatrixXd, Eigen::Lower> choleskyLDLT = input_matrix.llt();

        //     L_ = choleskyLDLT.matrixL();

        //     D_ = L_.diagonal().array().square().matrix().asDiagonal();
        //     cout << "D_ " << D_ << endl;
        //     assert((L_*D_*L_.transpose().eval() - input_matrix).norm() == 0);
        //     cout << "D_.determinant() " << D_.determinant() << endl;
        //     return log(D_.determinant());

        // }

        // double logdetD(){
        //     return logdetD(input_dense_);
        // }


        /**
         * @brief Default inverser
         * 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd inverse(){
            return inverse(input_dense_);
        }

        /**
         * @brief update the default matrix.
         * 
         * @param new_matrix input
         */
        inline void update_matrix(const MatrixXd& new_matrix){
            input_dense_ = new_matrix;
        }

    private:
        MatrixXd inverse_;
        MatrixXd L_;
        MatrixXd D_;
        int K_;
        MatrixXd input_dense_;
        MatrixXd inv_sqrt_D_;
    };

}
