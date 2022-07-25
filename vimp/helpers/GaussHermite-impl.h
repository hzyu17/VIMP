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

// #include "GaussHermite.h"
using namespace Eigen;


namespace vimp{
    template <typename Function>
    void GaussHermite<Function>::computeSigmaPts(){
        VectorXd a{VectorXd::Ones(_deg-1)};
        VectorXd c{VectorXd::LinSpaced(_deg-1, 1, _deg-1)};
        VectorXd c_over_a = (c.array() / a.array()).matrix();
        MatrixXd L{MatrixXd::Zero(_deg, _deg)};
        for (int i=0; i<_deg-1; i++){
            L(i+1, i) = c_over_a(i);
            L(i, i+1) = a(i);
        }

        _sigmapts = L.eigenvalues().real();
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
        computeSigmaPts();
        VectorXd W(_deg);
        int cnt = 0;
        for (double i_pt : _sigmapts){
            W(cnt) = boost::math::factorial<double>(_deg) / _deg / _deg / HermitePolynomial(_deg-1, i_pt) / HermitePolynomial(_deg-1, i_pt);
            cnt += 1;
        }
        _W = W;
    }

    template <typename Function>
    MatrixXd GaussHermite<Function>::Integrate(){

        computeWeights();
        LLT<MatrixXd> lltP(_P);
        MatrixXd sig{lltP.matrixL()};

        VectorXd pt_0 = VectorXd::Zero(_dim);

        try{
            MatrixXd res{MatrixXd::Zero((*_f)(pt_0).rows(), (*_f)(pt_0).cols())};
        }
        catch (int n){
            cout << "exception " << endl;
        }
        
        
        if (_dim == 1){
            for (int i=0; i<_deg; i++){
                res += _W(i)*(*_f)(sig*_sigmapts(i) + _mean);
            }
        }

        else if (_dim == 2) {
            for (int i = 0; i < _deg; i++) {
                for (int j = 0; j < _deg; j++) {
                    VectorXd pt_ij = VectorXd::Zero(2);
                    pt_ij << _sigmapts(i), _sigmapts(j);

                    VectorXd pt_ij_t = sig * pt_ij + _mean;
                    res += _W(i) * _W(j) * (*_f)(pt_ij_t);

                }
            }
        }
        else if (_dim == 3){
            for (int i=0; i<_deg; i++){
                for(int j=0; j<_deg; j++){
                    for(int k=0; k<_deg; k++){
                        VectorXd pt_ijk = VectorXd::Zero(3);
                        pt_ijk << _sigmapts(i), _sigmapts(j), _sigmapts(k);

                        VectorXd pt_ijk_t = sig * pt_ijk + _mean;
                        res += _W(i) * _W(j) * _W(k) * (*_f)(pt_ijk_t);
                    }
                }
            }
        }
        else if (_dim == 4){
            for (int i=0; i<_deg; i++){
                for(int j=0; j<_deg; j++){
                    for(int k=0; k<_deg; k++){
                        for (int l=0; l<_deg; l++){
                            VectorXd pt_ijkl = VectorXd::Zero(4);
                            pt_ijkl << _sigmapts(i), _sigmapts(j), _sigmapts(k), _sigmapts(l);
                            VectorXd pt_ijkl_t = sig * pt_ijkl + _mean;
                            res += _W(i) * _W(j) * _W(k) * _W(l) * (*_f)(pt_ijkl_t);
                        }
                    }
                }
            }

        }
        
        return res;
    }

}
