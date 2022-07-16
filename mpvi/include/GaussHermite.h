/**
 * @file GaussHermite.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Class to calculate the approximated integrations using Gauss-Hermite quadrature
 * @version 0.1
 * @date 2022-05-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <boost/math/special_functions/factorials.hpp>
using namespace Eigen;
using namespace std;

namespace{
    template <typename Function>
class GaussHermite{
public:
    /**
     * @brief Default constructor.
     * 
     */
    GaussHermite(){}
    /**
     * @brief Construct a new Gauss Hermite object
     * 
     * @param p degree of GH polynomial
     * @param dim dimension of the integrand
     * @param mean mean 
     * @param P covariance matrix
     * @param func the integrand function
     */
    GaussHermite(const int& p, const int& dim, const VectorXd& mean, const MatrixXd& P, const Function& func):
        p_{p},
        dim_{dim},
        mean_{mean},
        P_{P},
        f_{func},
        W_{VectorXd::Zero(p_)},
        sigmapts_{VectorXd::Zero(p_)}{}

    /**
     * Sigmapoints as the root of Hermite polynomials.
     * */
    void getSigmaPts(){
        VectorXd a{VectorXd::Ones(p_-1)};
        VectorXd c{VectorXd::LinSpaced(p_-1, 1, p_-1)};
        VectorXd c_over_a = (c.array() / a.array()).matrix();
        MatrixXd L{MatrixXd::Zero(p_, p_)};
        for (int i=0; i<p_-1; i++){
            L(i+1, i) = c_over_a(i);
            L(i, i+1) = a(i);
        }

        sigmapts_ = L.eigenvalues().real();
    }

    /**
     * Define the Hermite polynomial of degree deg, evaluate at x.
     * */
    double HermitePolynomial(const int& deg, const double& x){
        if (deg == 0) return 1;
        if (deg == 1) return x;
        else{
            double H0 = 1;
            double H1 = x;
            double H2 = x*x - 1;
            for (int i=3; i<deg+1; i++){
                H0 = H1;
                H1 = H2;
                H2 = x*H1 - (i-1)*H0;
            }
            return H2;
        }
    }

    /**
     * Compute the weights in the Gauss-Hermite cubature method.
     * */
    VectorXd getWeights(){
        getSigmaPts();
        VectorXd W(p_);
        int cnt = 0;
        for (double i_pt : sigmapts_){
            W(cnt) = boost::math::factorial<double>(p_) / p_ / p_ / HermitePolynomial(p_-1, i_pt) / HermitePolynomial(p_-1, i_pt);
            cnt += 1;
        }
        W_ = W;
        return W;
    }

    /**
     * Compute the approximated integration using Gauss-Hermite.
     * */
    MatrixXd Integrate(){

        getWeights();
        LLT<MatrixXd> lltP(P_);
        MatrixXd sig{lltP.matrixL()};

        VectorXd pt_0 = VectorXd::Zero(dim_);
        
        MatrixXd res{MatrixXd::Zero(f_(pt_0).rows(), f_(pt_0).cols())};
        
        if (dim_ == 1){
            for (int i=0; i<p_; i++){
                res += W_(i)*f_(sig*sigmapts_(i) + mean_);
            }
        }

        else if (dim_ == 2) {
            for (int i = 0; i < p_; i++) {
                for (int j = 0; j < p_; j++) {
                    VectorXd pt_ij = VectorXd::Zero(dim_);
                    pt_ij << sigmapts_(i), sigmapts_(j);

                    VectorXd pt_ij_t = sig * pt_ij + mean_;
                    res += W_(i) * W_(j) * f_(pt_ij_t);

                }
            }
        }
        else if (dim_ == 3){
            for (int i=0; i<p_; i++){
                for(int j=0; j<p_; j++){
                    for(int k=0; k<p_; k++){
                        VectorXd pt_ijk = VectorXd::Zero(dim_);
                        pt_ijk << sigmapts_(i), sigmapts_(j), sigmapts_(k);

                        VectorXd pt_ijk_t = sig * pt_ijk + mean_;
                        res += W_(i) * W_(j) * W_(k) * f_(pt_ijk_t);
                    }
                }
            }
        }
        // else if (dim_ == 4){
        //     for (int i=0; i<p_; i++){
        //         for(int j=0; j<p_; j++){
        //             for(int k=0; k<p_; k++){
        //                 for (int l=0; l<p_; l++){
        //                     VectorXd pt_ijkl(4);
        //                     pt_ijkl << sigmapts_(i), sigmapts_(j), sigmapts_(k), sigmapts_(l);
        //                     VectorXd pt_ijkl_t = sig * pt_ijkl + mean_;
        //                     res += W_(i) * W_(j) * W_(k) * W_(l) * f_(pt_ijkl_t);
        //                 }
        //             }
        //         }
        //     }

        // }
        
        return res;
    }

    /**
     * Update member variables
     * */
    inline void update_mean(const VectorXd& mean){
        mean_ = mean;
    }

    inline void update_P(const MatrixXd& P){
        P_ = P;
    }

    inline void set_p(const int& p){
        p_ = p;
    }

    inline void update_integrand(const Function& fun){
        f_ = fun;
    }

    inline void update_dimension(const int& dim){
        dim_ = dim;
    }

private:
    int p_;
    int dim_;
    VectorXd mean_;
    MatrixXd P_;
    Function f_;
    VectorXd W_;
    VectorXd sigmapts_;
};

}
