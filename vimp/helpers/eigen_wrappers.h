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

typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::SparseVector<double> SpVec; 
typedef Eigen::Triplet<double> Trip;

class EigenWrapper{
public:
    EigenWrapper(){};

    // ================= min and max values in a matrix or a vector =================
    double minval(const Eigen::MatrixXd& m){
        return m.minCoeff();
    }

    double maxval(const Eigen::MatrixXd& m){
        return m.maxCoeff();
    }
    
    // ================= random matrix, full and sparse =================
    Eigen::MatrixXd random_matrix(int m, int n){
        return Eigen::MatrixXd::Random(m ,n);
    }

    void random_sparse_matrix(SpMat & mat, int m, int n, int nnz){
        if (nnz > m*n){
            throw std::invalid_argument( "received negative value" );
        }
        std::vector<Trip> tripletList;
        tripletList.reserve(nnz);

        Random random;

        for (int i=0; i<nnz; i++){
            int i_row = random.randint(0, m-1);
            int j_col = random.randint(0, n-1);
            double val = random.rand_double(0.0, 10.0);
            tripletList.push_back(Trip(i_row, j_col, val));
        }
        mat.setFromTriplets(tripletList.begin(), tripletList.end());
    }

    SpMat random_sparse_matrix(int m, int n, int nnz){
        if (nnz > m*n){
            throw std::invalid_argument( "received negative value" );
        }
        std::vector<Trip> tripletList;
        tripletList.reserve(nnz);

        Random random;

        for (int i=0; i<nnz; i++){
            int i_row = random.randint(0, m-1);
            int j_col = random.randint(0, n-1);
            double val = random.rand_double(0.0, 10.0);
            tripletList.push_back(Trip(i_row, j_col, val));
        }

        SpMat mat(m, n);
        mat.setFromTriplets(tripletList.begin(), tripletList.end());

        return mat;
    }

    Eigen::VectorXd random_vector(int n){
        return Eigen::VectorXd::Random(n);
    }

    SpMat sp_eye(int n){
        std::vector<Trip> tripletList;
        tripletList.reserve(n);
        for (int i=0; i<n; i++){
            tripletList.push_back(Trip(i, i, 1));
        }
        SpMat mat(n, n);
        mat.setFromTriplets(tripletList.begin(), tripletList.end());
        return mat;
    }

    bool matrix_equal(const Eigen::MatrixXd& m1, const Eigen::MatrixXd& m2){
        return (m1 - m2).sum() < 1e-10;
    }

    // ================= Eigen valules and eigen vectors =================
    Eigen::VectorXcd eigen_values(const Eigen::MatrixXd& m){
        return m.eigenvalues();
    }

    // return real part of eigen values.
    Eigen::VectorXd real_eigenvalues(const Eigen::MatrixXd& m){
        Eigen::VectorXcd eigen_vals{eigen_values(m)};
        return eigen_vals.real();
    }

    // ================= decompositions for psd matrices =================
    Eigen::LDLT<Eigen::MatrixXd> ldlt_full(const Eigen::MatrixXd& m){
        return m.ldlt();
    }

    // ================= solving sparse equations =================
    Eigen::VectorXd solve_cgd_sp(const SpMat& A, const Eigen::VectorXd& b){
        Eigen::ConjugateGradient<SpMat, Eigen::Upper> solver;
        return solver.compute(A).solve(b);
    }

    Eigen::VectorXd solve_llt(const Eigen::MatrixXd& A, const Eigen::VectorXd& b){
        return A.llt().solve(b);
    }

    // ================= PSD matrix =================
    Eigen::MatrixXd random_psd(int n){
        Eigen::MatrixXd m{random_matrix(n, n)};
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
        Eigen::MatrixXd m{spm};
        Eigen::LDLT<Eigen::MatrixXd> ldlt(m);        
        return ldlt.isPositive();
    }

    bool is_positive(const Eigen::MatrixXd& m){
        Eigen::LDLT<Eigen::MatrixXd> ldlt(m);
        return ldlt.isPositive();
    }

    // ================= IO for matrix =================
    void print_matrix(const Eigen::MatrixXd& m){  
        Eigen::IOFormat CleanFmt(3, 0, ",", "\n", "[","]");
        std::cout << m.format(CleanFmt) << _sep;
    }

    void print_spmatrix(const SpMat& sp_m){
        Eigen::MatrixXd m(sp_m);
        Eigen::IOFormat CleanFmt(3, 0, ",", "\n", "[","]");
        std::cout << m.format(CleanFmt) << _sep;
    }

    template<typename MatrixType>
    void print_element(const MatrixType& spm, int row, int col){
        std::cout << "element at (" << std::to_string(row) << ", " << std::to_string(col) << "): " << std::endl << spm.coeff(row, col) << std::endl;
    }

    /**
     * @brief finding non-zero elements in a \b Row major sparse matrix
     * so that the sorted I,J will be row-order-major.
     */
    // credit to: https://github.com/libigl/libigl/blob/main/include/igl/find.cpp
    template <typename DerivedI, typename DerivedJ, typename DerivedV>
    inline void find_nnz(
    const Eigen::SparseMatrix<double, Eigen::ColMajor>& X,
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
        for(Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it (X,k); it; ++it)
        {
        V(i) = it.value();
        I(i) = it.row();
        J(i) = it.col();
        i++;
        }
    }
    }

    template <typename DerivedI, typename DerivedJ, typename DerivedV>
    inline void assemble(Eigen::SparseMatrix<double, Eigen::ColMajor> & X,
    Eigen::DenseBase<DerivedI> & I,
    Eigen::DenseBase<DerivedJ> & J,
    Eigen::DenseBase<DerivedV> & V){
        std::vector<Trip> tripletList;
        int nnz = I.size();
        tripletList.reserve(nnz);
        for (int i=0; i<nnz; i++){
            tripletList.push_back(Trip(I(i, 0), J(i, 0), V(i, 0)));
        }
        X.setFromTriplets(tripletList.begin(), tripletList.end());
    }


    // ================= Block operations ================= 
    SpMat block_extract_sparse(SpMat & mat, int start_row, int end_row, int start_col, int end_col){
        int nrows = end_row-start_row+1;
        int ncols = end_col-start_col+1;
        return mat.middleRows(start_row, nrows).middleCols(start_col, ncols);
    }

    void block_insert_sparse(SpMat & mat, const int start_row, int end_row, int start_col, int end_col, SpMat & block){
        int nrows = end_row - start_row + 1;
        int ncols = end_col - start_col + 1;
        // sparse matrix block is not writtable, conversion to dense first.
        Eigen::MatrixXd mat_full{mat};
        
        mat_full.block(start_row, start_col, nrows, ncols) = block;
        mat = mat_full.sparseView();
    }

    Eigen::VectorXd block_extract(Eigen::VectorXd & mat, int start_row, int end_row, int start_col, int end_col){
        int nrows = end_row-start_row+1;
        int ncols = end_col-start_col+1;
        return mat.block(start_row, start_col, nrows, ncols);
    }

    void block_insert(Eigen::VectorXd & mat, int start_row, int end_row, int start_col, int end_col, const Eigen::VectorXd& block){
        int nrows = end_row-start_row+1;
        int ncols = end_col-start_col+1;
        mat.block(start_row, start_col, nrows, ncols) = block;
    }

    bool is_symmetric(const Eigen::MatrixXd& m){
        Eigen::MatrixXd lower = m.triangularView<Eigen::StrictlyLower>();
        Eigen::MatrixXd upper_trans = m.triangularView<Eigen::StrictlyUpper>().transpose();
        return (lower - upper_trans).sum() <= 1e-14;
    }

    /**
     * @brief Sparse inversion for a psd matrix X
     * 
     * @param X matrix to inverse
     * @param X_inv lower triangular part of the inverse
     * @param I sorted row index of nnz elements in L
     * @param J col index of nnz elements in L
     * @param V value of nnz elements in L
     */
    void inv_sparse(const SpMat& X, SpMat & X_inv, const Eigen::VectorXd& Rows, const Eigen::VectorXd& Cols, const Eigen::VectorXd& Nzros){
        // ----------------- sparse ldlt decomposition -----------------
        typedef Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::NaturalOrdering<int>> SparseLDLT;
        
        int nnz = Rows.rows();
        int size = X.rows();
        SparseLDLT ldlt_sp(X);
        SpMat Lsp = ldlt_sp.matrixL();
        Eigen::VectorXd Dsp = ldlt_sp.vectorD().real();
        Eigen::VectorXd Dsp_inv = ldlt_sp.vectorD().real().cwiseInverse();
        
        X_inv.setZero();
        for (int index=nnz-1; index>=0; index--){ // iterator j, only for nnz in L
            int j = Rows.coeff(index);
            int k = Cols.coeff(index);
            // std::cout << "(j, k): (" << j << ", " << k << ")." << std::endl;
            double cur_val = 0;
            if (j==k){ // diagonal
                cur_val = Dsp_inv(j, k);
            }
            // iterate downward in L, same column as k
            for (int l_indx=index; l_indx < nnz; l_indx++){ 
                if (Cols(l_indx) != k){
                    break;
                }
                int l = Rows(l_indx);
                // std::cout << " (l, k): [" << l << ", " << Cols(l_indx) <<"]" << std::endl; 
                // std::cout << " (j, l): [" << j << ", " << l <<"]" << std::endl; 
                if (l > j){
                    cur_val = cur_val - X_inv.coeff(l, j) * Lsp.coeff(l, k);
                }else{
                    cur_val = cur_val - X_inv.coeff(j, l) * Lsp.coeff(l, k);
                }
            }
            X_inv.coeffRef(j, k) = cur_val;
            // std::cout << "X_inv(j,k):" << X_inv.coeff(j, k) << std::endl;
        }
        SpMat lower_tri = X_inv.triangularView<Eigen::StrictlyLower>().transpose();
        X_inv = X_inv + lower_tri;
    }

private:
    // IO related
    std::string _sep = "\n----------------------------------------\n";
};