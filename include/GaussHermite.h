/**
 * Class to calculate the approximated integrations using Gauss-Hermite quadrature
 * Author: Hongzhe Yu
 * Date: 05/11/2022
 * */

#include <Eigen/Dense>
#include <iostream>
#include <boost/math/special_functions/factorials.hpp>
using namespace Eigen;
using namespace std;

template <typename Function>
class GaussHermite{
public:
    GaussHermite(const int& p, const int& dim, const VectorXd& mean, const MatrixXd& P, const Function& func):
        p_{p},
        dim_{dim},
        mean_{mean},
        P_{P},
        f_{func},
        W_{VectorXd::Zero(p_)},
        sigmapts_{VectorXd::Zero(p_)}{}

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

    MatrixXd Integrate(){

        getWeights();
        LLT<MatrixXd> lltP(P_);
        MatrixXd sig{lltP.matrixL()};

        MatrixXd sigmpts_h = sigmapts_.transpose().replicate(dim_, 1);

        MatrixXd pts{sig * sigmpts_h};
        pts.colwise() += mean_;

        VectorXd pt_0 = VectorXd::Zero(dim_);
        pt_0 << pts(0, 0), pts(1, 0);

        MatrixXd res{MatrixXd::Zero(f_(pt_0).rows(), f_(pt_0).cols())};

        if (dim_ == 1){
            for (int i=0; i<p_; i++){
                auto res1 =f_(pts.col(i));
                cout << res1 << endl;
                res += W_(i)*f_(pts.col(i));
            }
        }

        else if (dim_ == 2) {
            for (int i = 0; i < p_; i++) {
                for (int j = 0; j < p_; j++) {
                    VectorXd pt_ij = VectorXd::Zero(dim_);
                    pt_ij << pts(0, i), pts(1, j);
                    res += W_(i) * W_(j) * f_(pt_ij);
                }
            }
        }
        else if (dim_ == 3){
            for (int i=0; i<p_; i++){
                for(int j=0; j<p_; j++){
                    for(int k=0; k<p_; k++){
                        VectorXd pt_ijk = VectorXd::Zero(dim_);
                        pt_ijk << pts(0, i), pts(1, j), pts(2, k);
                        res += W_(i) * W_(j) * W_(k) * f_(pt_ijk);
                    }
                }
            }
        }

        return res;
    }

    /**
     * Update member variables
     * */
    void update_mean(const VectorXd& mean){
        mean_ = mean;
    }

    void update_P(const MatrixXd& P){
        P_ = P;
    }

    void set_p(int p){
        p_ = p;
    }

    void update_integrand(const Function& fun){
        f_ = fun;
    }

    void update_dimension(const int dim){
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
