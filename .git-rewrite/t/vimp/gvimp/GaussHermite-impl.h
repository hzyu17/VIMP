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

using namespace Eigen;

namespace vimp{
    template <typename Function>
    void GaussHermite<Function>::permute_replacing(
                            const std::vector<int>& vec, 
                            const int& dimension, 
                            std::vector<int>& res, 
                            int index, 
                            std::vector<std::vector<int>>& v_res)
    {
        for (int i=0; i<vec.size(); i++){
            res[index] = vec[i];

            if (index == dimension-1){
                v_res.emplace_back(res);
                
            }else{
                // recursive
                permute_replacing(vec, dimension, res, index+1, v_res);
            }
        }
        return;
    }

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
        computeSigmaPts();

        VectorXd W(_deg);
        int cnt = 0;
        for (double i_pt : _sigmapts){
            W(cnt) = boost::math::factorial<double>(_deg) / _deg / _deg / HermitePolynomial(_deg-1, i_pt) / HermitePolynomial(_deg-1, i_pt);
            cnt += 1;
        }
        this->_W = W;
    }

    template <typename Function>
    void GaussHermite<Function>::update_integrand(const Function& function){
        _f = function;
    }

    template <typename Function>
    MatrixXd GaussHermite<Function>::Integrate(){
        return Integrate(_f);
    }

    template <typename Function>
    MatrixXd GaussHermite<Function>::Integrate(const Function& function){
                
        computeWeights();

        LLT<MatrixXd> lltP(_P);
        MatrixXd sig{lltP.matrixL()};

        VectorXd pt_0(_dim);
        pt_0.setZero();

        // Borrow space for the integration result
        MatrixXd res{MatrixXd::Zero(function(pt_0).rows(), function(pt_0).cols())}; 

        /// Compute permutations
        std::vector<int> range_deg;
        for (int i=0; i<_deg; i++){
            range_deg.emplace_back(i);
        }
        std::vector<int> permutation(_dim);
        std::vector<std::vector<int>> v_permutations;

        permute_replacing(range_deg, _dim, permutation, 0, v_permutations);

        for (std::vector<int>& i_v: v_permutations){
            VectorXd pt_ij(_dim);
            double weights = 1.0;
            int cnt = 0;
            for(int& j : i_v){
                pt_ij(cnt) = _sigmapts(j);
                weights = weights*_W(j);
                cnt += 1;
            }
            pt_ij = sig * pt_ij + _mean;
            res += weights * function(pt_ij);

        }

        return res;
    }

}
