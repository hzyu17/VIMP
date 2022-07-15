/**
 * @file test_unary_factror.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the different behaviors of the class inherited from gtsam::NoiseModel
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/GaussianPriorUnaryTranslation.h"
#include <gtsam/inference/Symbol.h>
#include <iostream>

using namespace std;
using namespace gpmp2;
using namespace gtsam;
using namespace Eigen;

using UnaryFactorTranslation2D = UnaryFactorTranslation<gtsam::Vector2>;


/// phi(x) = -log(p(x,z))
double errorWrapperPrior(const VectorXd& theta, const UnaryFactorTranslation2D& prior_factor) {

    /**
     * Prior factor
     * */
    gtsam::Vector2 position;
    position = theta.segment<2>(0);

    VectorXd vec_prior_err = prior_factor.evaluateError(position);

    MatrixXd K{prior_factor.get_Qc()};

    return vec_prior_err.transpose() * K.inverse() * vec_prior_err;

}

using Function1 = std::function<double(const VectorXd&, const UnaryFactorTranslation2D&)>;
using Function2 = std::function<MatrixXd(const VectorXd&)>;

class B{
    public:
        B(const Function2& func):_func2{func}{}

        Function2 _func2;

        void func_2_in_class_b(const VectorXd& x){
            MatrixXd res = _func2(x);
            cout << "res class b" << endl << res << endl;
        }

};

class A{
    public:
        A(const Function1& func, const UnaryFactorTranslation2D& fac, const VectorXd& vec): 
        _func1{std::move(func)},
        _factor{fac},
        _vec{vec},
        _func2{[this](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1,1,_func1(x-_vec, _factor))};}},
        _class_b{_func2}{}

        Function1 _func1;
        UnaryFactorTranslation2D _factor;

        Function2 _func2;
        VectorXd _vec;
        B _class_b; 

        void test_function2(const VectorXd& x){
            MatrixXd res = _func2(x);
            cout << "res" << endl << res << endl;

            _class_b.func_2_in_class_b(x);
        }

        void set_vec(const VectorXd& new_vec){
            _vec = new_vec;
        }
        
};

int main(){
    int dim_conf = 2;

    /// Noise model definition
    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(dim_conf, 2.0);
    cout << "Qc" << endl << getQc(Qc_model) << endl;
    SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(dim_conf, 0.5);
    cout << "K_0" << endl << getQc(K_0) << endl;

    gtsam::Vector2 conf{Vector2d::Zero()};
    conf << 1, 0;

    UnaryFactorTranslation2D factor{gtsam::symbol('x', 0), conf, Qc_model};

    Vector2d q{Vector2d::Zero()};
    q << 2, 0;
    double err = errorWrapperPrior(q, factor);
    cout << "err" << endl << err << endl;

    using vecA = vector<shared_ptr<A>>;

    vecA vec_A;
    for (int i=0; i<5; i++){
        shared_ptr<A> pa(new A{errorWrapperPrior, factor, conf});
        vecA.emplace_back(pa);
    }
    
    a.test_function2(q);
    Vector2d q1{Vector2d::Zero()};
    q1 << 5, 0;

    a.set_vec(q1);
    a.test_function2(q);

    return 0;
}