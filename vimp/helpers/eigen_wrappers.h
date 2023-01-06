/**
 * @file eigen_wrappers.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Wrappers for Eigen libraries.
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<Eigen/Dense>
#include<Eigen/Sparse>
#include<Eigen/SparseCholesky>
#include"random.h"

typedef Eigen::VectorXd Vector;
typedef Eigen::VectorXcd VectorC;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::SparseVector<double> SpVec; 
typedef Eigen::Triplet<double> T;

class MatrixClass{
public:
    MatrixClass(){};

    // ================= min and max values in a matrix or a vector =================
    double minval(const Matrix& m){
        return m.minCoeff();
    }

    double maxval(const Matrix& m){
        return m.maxCoeff();
    }
    
    // ================= random matrix, full and sparse =================
    Matrix random_matrix(int m, int n){
        return Matrix::Random(m ,n);
    }

    SpMat random_sparse_matrix(int m, int n, int nnz){
        if (nnz > m*n){
            throw std::invalid_argument( "received negative value" );
        }
        std::vector<T> tripletList;
        tripletList.reserve(nnz);

        Random random;

        for (int i=0; i<nnz; i++){
            int i_row = random.randint(0, m-1);
            int j_col = random.randint(0, n-1);
            double val = random.rand_double(0.0, 10.0);
            tripletList.push_back(T(i_row, j_col, val));
        }

        SpMat mat(m, n);
        mat.setFromTriplets(tripletList.begin(), tripletList.end());

        return mat;
    }

    Vector random_vector(int n){
        return Vector::Random(n);
    }


    // ================= Eigen valules and eigen vectors =================
    VectorC eigen_values(const Matrix& m){
        return m.eigenvalues();
    }

    // return real part of eigen values.
    Vector real_eigenvalues(const Matrix& m){
        VectorC eigen_vals{eigen_values(m)};
        return eigen_vals.real();
    }


    // ================= PSD matrix =================
    Matrix random_psd(int n){
        Matrix m{random_matrix(n, n)};
        return m.transpose() * m;
    }

    /**
     * @brief sparse psd matrix. P = A^T*A.
     * 
     * @param n : size of matrix P.
     * @param nnz : number of non-zeors in A.
     * @return SpMat 
     */
    SpMat randomd_sparse_psd(int n, int nnz){
        SpMat A{random_sparse_matrix(n, n, nnz)};        
        return A.transpose() * A;
    }

    bool is_sparse_positive(const SpMat& spm){
        Matrix m{spm};
        Eigen::LDLT<Matrix> ldlt(m);        
        return ldlt.isPositive();
    }

    bool is_positive(const Matrix& m){
        Eigen::LDLT<Matrix> ldlt(m);
        return ldlt.isPositive();
    }

    // ================= IO for matrix =================
    void print_matrix(const Matrix& m){  
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        std::cout << m.format(CleanFmt) << _sep;
    }

    void print_spmatrix(const SpMat& sp_m){
        Matrix m(sp_m);
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        std::cout << m.format(CleanFmt) << _sep;
    }

private:
    // IO related
    std::string _sep = "\n----------------------------------------\n";
};