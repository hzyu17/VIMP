/**
 * @file EigenWrapper.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Wrappers for Eigen libraries.
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#ifndef EIGEN_WRAPPER_H
#define EIGEN_WRAPPER_H

#include "MatrixHelper.h"
#include <random>

using namespace Eigen;

namespace vimp{
class Random{

public:
    Random(){};

    int randint(int range_from, int range_to) {
        std::mt19937                        generator(_rand_dev());
        std::uniform_int_distribution<>    distr(range_from, range_to);
        return distr(generator);
    }

    double rand_double(double range_from, double range_to) {
        std::mt19937                        generator(_rand_dev());
        std::uniform_real_distribution<>    distr(range_from, range_to);
        return distr(generator);
    }

private:
    std::random_device  _rand_dev;

};

class EigenWrapper{
public:
    EigenWrapper(){};
    
    template <typename Derived>
    bool matrix_equal(const Eigen::MatrixBase<Derived>& m1, const Eigen::MatrixBase<Derived>& m2){
        if (m1.rows()!=m2.rows() || m1.cols() != m2.cols()){
            std::cout << "not same dimension!" << std::endl;
            return false;
        }
        bool res = (m1 - m2).norm() < 1e-10;
        if (res){
            return res;
        }else{
            std::cout << "not equal, norm difference: " << (m1 - m2).norm() << std::endl;
            return res;
        }
    }

    /**
     * @brief overload in case that one matrix is sparse.
     */
    bool matrix_equal(const Eigen::MatrixXd& m1, const Eigen::MatrixXd& m2){
        if (m1.rows()!=m2.rows() || m1.cols() != m2.cols()){
            std::cout << "not same dimension!" << std::endl;
            return false;
        }
        bool res = (m1 - m2).norm() < 1e-10;
        if (res){
            return res;
        }else{
            std::cout << "not equal, norm difference: " << (m1 - m2).norm() << std::endl;
            return res;
        }
    }

    // ================= PSD matrix =================
    /**
     * @brief sparse psd matrix. P = A^T*A.
     * 
     * @param n : size of matrix P.
     * @param nnz : number of non-zeors in A.
     * @return SpMat 
     */
    SpMat randomd_sparse_psd(int n, int nnz){
        if (nnz > n*n){
            throw std::invalid_argument( "received negative value" );
        }
        std::vector<Trip> tripletList;
        tripletList.reserve(nnz);

        Random random;

        for (int i=0; i<nnz; i++){
            int i_row = random.randint(0, n-1);
            int j_col = random.randint(0, n-1);
            double val = random.rand_double(0.0, 10.0);
            tripletList.push_back(Trip(i_row, j_col, val));
        }

        SpMat A(n, n);
        A.setFromTriplets(tripletList.begin(), tripletList.end());

        return A.transpose() * A;
    }

    // ================= IO for matrix and digits =================
    template <typename Derived>
    void print_matrix(const Eigen::MatrixBase<Derived>& m, std::string header="matrix printed"){  
        Eigen::IOFormat CleanFmt(6, 0, ",", "\n", "[","]");
        std::cout << header << std::endl;
        std::cout << m.format(CleanFmt) << _sep;
    }

    void print_matrix(const SpMat& sp_m, std::string header="matrix printed"){  
        Eigen::IOFormat CleanFmt(6, 0, ",", "\n", "[","]");
        std::cout << header << std::endl;
        MatrixXd m(sp_m);
        std::cout << m.format(CleanFmt) << _sep;
    }

    /**
     * @brief finding non-zero elements in a \b Row major sparse matrix
     * so that the sorted I,J will be row-order-major.
     */
    // credit to: https://github.com/libigl/libigl/blob/main/include/igl/find.cpp
    template <typename DerivedI, typename DerivedJ, typename DerivedV>
    inline int find_nnz(
    const SpMat& X,
    Eigen::DenseBase<DerivedI> & I,
    Eigen::DenseBase<DerivedJ> & J,
    Eigen::DenseBase<DerivedV> & V)
    {
    // Resize outputs to fit nonzeros
    I.derived().resize(X.nonZeros(),1);
    J.derived().resize(X.nonZeros(),1);
    V.derived().resize(X.nonZeros(),1);

    int i = 0;
    // Iterate over outside
    for(int k=0; k<X.outerSize(); ++k)
    {
        // Iterate over inside
        for(SpMat::InnerIterator it (X,k); it; ++it)
        {   
            V(i) = it.value();
            I(i) = it.row();
            J(i) = it.col();
            i++;
        }
    }
    return I.rows();
    }

    void find_nnz_known_ij(const SpMat & mat, const Eigen::VectorXi& Rows, const Eigen::VectorXi& Cols, Eigen::VectorXd& Vals){
        int nnz = Rows.rows();
        Vals.resize(nnz);
        for (int i=0; i<nnz; i++){
            Vals(i) = mat.coeff(Rows(i), Cols(i));
        }
    }

    // ================= GVIBlock operations ================= 
    SpMat block_extract_sparse(SpMat & mat, int start_row, int start_col, int nrows, int ncols){
        return mat.middleRows(start_row, nrows).middleCols(start_col, ncols);
    }

    void block_insert_sparse(SpMat & mat, int start_row, int start_col, int nrows, int ncols, const Eigen::MatrixXd & block){
        if (nrows>mat.rows() || ncols>mat.cols()){
            std::__throw_out_of_range("row or column length out of range!");
        }
        // sparse matrix block is not writtable, conversion to dense first.
        Eigen::MatrixXd mat_full{mat};
        
        mat_full.block(start_row, start_col, nrows, ncols) = block;
        mat = mat_full.sparseView();
    }

    Eigen::VectorXd block_extract(Eigen::VectorXd & mat, int start_row, int start_col, int nrows, int ncols){
        return mat.block(start_row, start_col, nrows, ncols);
    }

    void block_insert(Eigen::VectorXd & mat, int start_row, int start_col, int nrows, int ncols, const Eigen::VectorXd& block){
        mat.block(start_row, start_col, nrows, ncols) = block;
    }

    void block_insert(Eigen::MatrixXd & mat, int start_row, int start_col, int nrows, int ncols, const Eigen::MatrixXd& block){
        mat.block(start_row, start_col, nrows, ncols) = block;
    }

    Eigen::MatrixXd block_extract(Eigen::MatrixXd & mat, int start_row, int start_col, int nrows, int ncols){
        return mat.block(start_row, start_col, nrows, ncols);
    }

    Eigen::MatrixXd psd_sqrtm(const Eigen::MatrixXd & m){
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(m);
        return es.operatorSqrt();
    }

    Eigen::MatrixXd psd_invsqrtm(const Eigen::MatrixXd & m){
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(m);
        return es.operatorInverseSqrt();
    }

    /**
     * @brief extract the i_th index from a 3d matrix in shape (rows*cols, nt):
     * return the matrix mat in shape (rows, cols) from the i_th column. 
     */
    void decomp3d(Matrix3D mat3d, Eigen::MatrixXd& mat, 
                      int rows, int cols, int i){
        mat = mat3d.col(i).reshaped(rows, cols);
    }

    Eigen::MatrixXd decomp3d(Matrix3D mat3d, int rows, int cols, int i){
        Eigen::MatrixXd mat(rows, cols);
        mat = mat3d.col(i).reshaped(rows, cols);
        return mat;
    }

    void compress3d(Eigen::MatrixXd mat, Matrix3D& mat3d, int i){
        Eigen::VectorXd column(mat.rows()*mat.cols());
        column = mat.reshaped(mat.rows()*mat.cols(), 1);
        mat3d.col(i) = column;
    }

    Matrix3D replicate3d(Eigen::MatrixXd mat, const int len){
        int rows = mat.rows(), cols = mat.cols();
        Eigen::MatrixXd mat3 = Eigen::MatrixXd::Zero(rows*cols, len);
        mat3 = mat.reshaped(rows*cols, 1).replicate(1, len);
        return mat3;
    }

    
    Matrix3D vec2mat3d(const std::vector<Matrix3D>& vec){
        int len = vec.size();
        Matrix3D m0 = vec[0];
        int rc = m0.rows();
        int nt = m0.cols();

        Eigen::MatrixXd res(rc, len*nt);

        int cnt=0;
        for (auto& item:vec){
            res.block(0, cnt*nt, rc, nt) = item;
            cnt += 1;
        }
        return res;
    }

    // MatrixXd linspace(const VectorXd & x0, const VectorXd & xT, int nt){
    //     int rows = x0.rows();
    //     VectorXd step_vec(rows);
    //     step_vec.setZero();
    //     step_vec = (xT-x0)/(nt-1);
    //     MatrixXd res(rows, nt);
    //     res.setZero();
    //     for (int i=0; i<nt; i++){
    //         res.col(i) = x0 + step_vec*i;
    //     }
    //     return res;
    // }

    using vec_1d = std::vector<double>;
    using vec_2d = std::vector<vec_1d>;

    vec_2d eigen_to_vector(const Eigen::MatrixXd& mat){
        vec_2d vec(mat.rows());
        for (int i=0; i<mat.rows(); i++){
            vec_1d row_i(mat.cols());
            for (int j=0; j<mat.cols(); j++){
                row_i[j] = mat.coeffRef(i, j);
            }
            vec[i] = row_i;
        }
        return vec;
    }

    
    /**
     * @brief Sparse inversion for a psd matrix X
     * 
     * @param X matrix to inverse
     * @param X_inv lower triangular part of the inverse
     * @param I sorted row index of nnz elements in L
     * @param J col index of nnz elements in L
     */
    void inv_sparse(const SpMat & X, 
    SpMat & X_inv, 
    const Eigen::VectorXi& Rows, 
    const Eigen::VectorXi& Cols, 
    int nnz){
        // ----------------- sparse ldlt decomposition -----------------
        SparseLDLT ldlt_sp(X);
        SpMat Lsp = ldlt_sp.matrixL();
        Eigen::VectorXd Dsp_inv = ldlt_sp.vectorD().real().cwiseInverse();

        // int nnz = Rows.rows();
        X_inv.setZero();
        for (int index=nnz-1; index>=0; index--){ // iterator j, only for nnz in L
            int j = Rows(index);
            int k = Cols(index);
            double cur_val = 0.0;

            if (j==k){ // diagonal
                cur_val = Dsp_inv(j);
            }
            // find upward the starting point for l = k+1
            int s_indx = index;
            while(true){
                if(Cols(s_indx)<k || s_indx==0){
                    s_indx = s_indx+1;
                    break;
                }
                s_indx -= 1;
            }
            
            // iterate downward in L(l\in(k+1,K), k)
            for (int l_indx=s_indx; l_indx < nnz; l_indx++){ 
                if (Cols.coeff(l_indx) > k){
                    break;
                }

                int l = Rows(l_indx);
                if (l > j){
                    cur_val = cur_val - X_inv.coeff(l, j) * Lsp.coeff(l, k);
                }else{
                    cur_val = cur_val - X_inv.coeff(j, l) * Lsp.coeff(l, k);
                }
            }
            X_inv.coeffRef(j, k) = cur_val;
            // X_inv.coeffRef(k, j) = cur_val;
        }
        SpMat upper_tri = X_inv.triangularView<Eigen::StrictlyLower>().transpose();
        X_inv = X_inv + upper_tri;
        
    }

    /**
     * @brief inverse with known ldlt.
     */
    void inv_sparse(const SpMat & X, 
    SpMat & X_inv, 
    const Eigen::VectorXi& Rows, 
    const Eigen::VectorXi& Cols, 
    const Eigen::VectorXd& Vals,
    const Eigen::VectorXd& Dinv)
    {   
        int nnz = Rows.rows();
        X_inv.setZero();
        for (int index=nnz-1; index>=0; index--){ // iterator j, only for nnz in L
            int j = Rows(index);
            int k = Cols(index);
            double cur_val = 0.0;

            if (j==k){ // diagonal
                cur_val = Dinv(j);
            }
            // find upward the starting point for l = k+1
            int s_indx = index;
            while(true){
                if(Cols(s_indx)<k || s_indx==0){
                    s_indx = s_indx+1;
                    break;
                }
                s_indx -= 1;
            }
            
            // iterate downward in L(l\in(k+1,K), k)
            for (int l_indx=s_indx; l_indx < nnz; l_indx++){ 
                if (Cols.coeff(l_indx) > k){
                    break;
                }

                int l = Rows(l_indx);
                if (l > j){
                    cur_val = cur_val - X_inv.coeff(l, j) * Vals(l_indx);
                }else{
                    cur_val = cur_val - X_inv.coeff(j, l) * Vals(l_indx);
                }
            }
            X_inv.coeffRef(j, k) = cur_val;
            // X_inv.coeffRef(k, j) = cur_val;
        }
        SpMat upper_tri = X_inv.triangularView<Eigen::StrictlyLower>().transpose();
        X_inv = X_inv + upper_tri;
    }

private:
    // IO related
    std::string _sep = "\n----------------------------------------\n";
    
};

}

#endif // EIGEN_WRAPPER_H