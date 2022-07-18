/**
 * @file GaussHermite-impl.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Calculate the approximated integrations using Gauss-Hermite quadrature
 * @version 0.1
 * @date 2022-05-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "GaussHermite.h"

namespace vimp{
    template <typename Function>
    void GaussHermite<Function>::computeSigmaPts(){
        VectorXd a{VectorXd::Ones(this->_deg-1)};
        VectorXd c{VectorXd::LinSpaced(this->_deg-1, 1, this->_deg-1)};
        VectorXd c_over_a = (c.array() / a.array()).matrix();
        MatrixXd L{MatrixXd::Zero(this->_deg, this->_deg)};
        for (int i=0; i<this->_deg-1; i++){
            L(i+1, i) = c_over_a(i);
            L(i, i+1) = a(i);
        }

        this->_sigmapts = L.eigenvalues().real();
    }

    template <typename Function>
    double GaussHermite<Function>::HermitePolynomial(const int& deg, const double& x) const{
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

    template <typename Function>
    void GaussHermite<Function>::computeWeights(){
        this->computeSigmaPts();
        VectorXd W(this->_deg);
        int cnt = 0;
        for (double i_pt : this->_sigmapts){
            W(cnt) = boost::math::factorial<double>(this->_deg) / this->_deg / this->_deg / HermitePolynomial(this->_deg-1, i_pt) / HermitePolynomial(this->_deg-1, i_pt);
            cnt += 1;
        }
        this->_W = W;
    }

    template <typename Function>
    MatrixXd GaussHermite<Function>::Integrate(){

        this->computeWeights();
        LLT<MatrixXd> lltP(this->_P);
        MatrixXd sig{lltP.matrixL()};

        VectorXd pt_0 = VectorXd::Zero(this->_dim);
        
        MatrixXd res{MatrixXd::Zero(this->_f(pt_0).rows(), this->_f(pt_0).cols())};
        
        if (this->_dim == 1){
            for (int i=0; i<this->_deg; i++){
                res += this->_W(i)*this->_f(sig*this->_sigmapts(i) + this->_mean);
            }
        }

        else if (this->_dim == 2) {
            for (int i = 0; i < this->_deg; i++) {
                for (int j = 0; j < this->_deg; j++) {
                    VectorXd pt_ij = VectorXd::Zero(2);
                    pt_ij << this->_sigmapts(i), this->_sigmapts(j);

                    VectorXd pt_ij_t = sig * pt_ij + this->_mean;
                    res += this->_W(i) * this->_W(j) * this->_f(pt_ij_t);

                }
            }
        }
        else if (this->_dim == 3){
            for (int i=0; i<this->_deg; i++){
                for(int j=0; j<this->_deg; j++){
                    for(int k=0; k<this->_deg; k++){
                        VectorXd pt_ijk = VectorXd::Zero(3);
                        pt_ijk << this->_sigmapts(i), this->_sigmapts(j), this->_sigmapts(k);

                        VectorXd pt_ijk_t = sig * pt_ijk + this->_mean;
                        res += this->_W(i) * this->_W(j) * this->_W(k) * this->_f(pt_ijk_t);
                    }
                }
            }
        }
        else if (this->_dim == 4){
            for (int i=0; i<this->_deg; i++){
                for(int j=0; j<this->_deg; j++){
                    for(int k=0; k<this->_deg; k++){
                        for (int l=0; l<this->_deg; l++){
                            VectorXd pt_ijkl = VectorXd::Zero(4);
                            pt_ijkl << this->_sigmapts(i), this->_sigmapts(j), this->_sigmapts(k), this->_sigmapts(l);
                            VectorXd pt_ijkl_t = sig * pt_ijkl + this->_mean;
                            res += this->_W(i) * this->_W(j) * this->_W(k) * this->_W(l) * this->_f(pt_ijkl_t);
                        }
                    }
                }
            }

        }
        
        return res;
    }

}
