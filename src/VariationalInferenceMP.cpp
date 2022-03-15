//
// Created by hongzhe on 2/28/22.
//
// Variational Inference for motion planning

#include "../include/SparseMatrixHelper.h"
#include "../include/MVGsampler.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/obstacle/ObstacleCost.h>
#include <Eigen/Core>

#include <iostream>
using namespace gtsam;
using namespace gpmp2;
using namespace std;
using namespace GaussianSampler;

typedef SparseMatrix<double> SpMatrix;
typedef Triplet<double> T;

inline float errorWrapper(const ObstaclePlanarSDFFactorGPPointRobot& factor,
                          const Eigen::MatrixXd& obs_sigma,
                          const gtsam::Vector& conf1, const gtsam::Vector& vel1,
                          const gtsam::Vector& conf2, const gtsam::Vector& vel2) {
    gtsam::Vector err_vector{factor.evaluateError(conf1, vel1, conf2, vel2)};
    return err_vector.transpose() * obs_sigma.inverse() * err_vector;
}

// convert sdf vector to hinge loss err vector
inline gtsam::Vector convertSDFtoErr(const gtsam::Vector& sdf, double eps) {
    gtsam::Vector err_ori = 0.0 - sdf.array() + eps;
    return (err_ori.array() > 0.0).select(err_ori, gtsam::Vector::Zero(err_ori.rows()));  // (R < s ? P : Q)
}


int main()
{
    //sdf
    gtsam::Matrix field, map_ground_truth;
    PlanarSDF sdf;

    map_ground_truth = (gtsam::Matrix(7, 7) <<
                                     0,     0,     0,     0,     0,     0,     0,
            0,     0,     0,     0,     0,     0,     0,
            0,     0,     1,     1,     1,     0,     0,
            0,     0,     1,     1,     1,     0,     0,
            0,     0,     1,     1,     1,     0,     0,
            0,     0,     0,     0,     0,     0,     0,
            0,     0,     0,     0,     0,     0,     0).finished();
    field = (gtsam::Matrix(7, 7) <<
                          2.8284,    2.2361,    2.0000,    2.0000,    2.0000,    2.2361,    2.8284,
            2.2361,    1.4142,    1.0000,    1.0000,    1.0000,    1.4142,    2.2361,
            2.0000,    1.0000,   -1.0000,   -1.0000,   -1.0000,    1.0000,    2.0000,
            2.0000,    1.0000,   -1.0000,   -2.0000,   -1.0000,    1.0000,    2.0000,
            2.0000,    1.0000,   -1.0000,   -1.0000,   -1.0000,    1.0000,    2.0000,
            2.2361,    1.4142,    1.0000,    1.0000,    1.0000,    1.4142,    2.2361,
            2.8284,    2.2361,    2.0000,    2.0000,    2.0000,    2.2361,    2.8284).finished();

    Point2 origin(0, 0);
    double cell_size = 1.0;

    sdf = PlanarSDF(origin, cell_size, field);

    // 2D point robot
    const int ndof = 2, nlinks = 1, nspheres = 1, nsupptd_states = 2, N=1;
    const int ndim = 2*ndof * nlinks * nsupptd_states;
    PointRobot pR(ndof,nlinks);

    double r = 1.5;
    BodySphereVector body_spheres;
    body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));

    int n_spheres = body_spheres.size();

    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(2, 1.0);

    double delta_t = 0.1, tau = 0.025;
    double obs_eps = 0.2, obs_sigma = 1.0;

    PointRobotModel pRModel(pR, body_spheres);
    ObstaclePlanarSDFFactorGPPointRobot factor(0, 0, 0, 0, pRModel, sdf, obs_sigma, obs_eps,
                                               Qc_model, delta_t, tau);

    // start and end configurations
    gtsam::Vector2 q_s, q_e, qdot_s, qdot_e;
//    gtsam::Matrix H1_act, H2_act, H3_act, H4_act;

    // origin zero  and stationary case
    q_s    = gtsam::Vector2(0, 0);
    q_e    = gtsam::Vector2(17, 14);
    qdot_s = gtsam::Vector2(0, 0);
    qdot_e = gtsam::Vector2(0, 0);

    // this q and last q
    gtsam::Vector2 q_1, q_2, qdot_1, qdot_2;
//    gtsam::Matrix H1_act, H2_act, H3_act, H4_act;

    // origin zero  and stationary case
    q_1    = gtsam::Vector2(0, 0);
    q_2    = gtsam::Vector2(17, 14);
    qdot_1 = gtsam::Vector2(0, 0);
    qdot_2 = gtsam::Vector2(0, 0);

    // the collection of supported states
    gtsam::Vector mu = concatVectors({q_s, qdot_s});

    // error vector
    gtsam::Vector err_act(n_spheres*(nsupptd_states-1));

    // adding middle points
    for (int i=0; i < nsupptd_states - 1; i++){
        q_1 =q_s+i*(q_e-q_s)/(nsupptd_states - 1);
        qdot_1 = qdot_s+i*(qdot_e-qdot_s)/(nsupptd_states - 1);

        q_2 = q_s+(i+1)*(q_e-q_s)/(nsupptd_states - 1);
        qdot_2 = qdot_s+(i+1)*(qdot_e-qdot_s)/(nsupptd_states - 1);

        mu = concatVectors({mu, q_2, qdot_2});
        auto err = factor.evaluateError(q_1, qdot_1, q_2, qdot_2);
        cout << "nspheres " << endl << nspheres << endl;
        cout << "gtsam::Vector{factor.evaluateError(q_1, qdot_1, q_2, qdot_2)} size "
             << err.size()
             << endl;
        err_act(seq(i*n_spheres, (i+1)*nspheres-1)) = err;

    }

    cout << " mu " << endl << mu << endl;
    cout << " err_act " << endl << err_act << endl;

    // optimization variables
    const int nnz = 3*ndim-2; //number of nonzeros in the tri-diag precision matrix

    SpMatrix invSigma(ndim, ndim);
    invSigma.reserve(nnz);

    // filling sparse matrix
    vector< T > tripletList;
    tripletList.reserve(nnz);

    for (int i=0; i<ndim; i++)
    {
        tripletList.emplace_back(i,i,1);
    }
    invSigma.setFromTriplets(tripletList.begin(), tripletList.end());

    // Derivatives
    gtsam::Vector d_mu(ndim);
    SpMatrix d_invSigma(ndim, ndim);

    // parameters
    double step_size_mu = 1;
    double step_size_Sigma = 1;

    // Gaussian sampler
    int nn = 10000; // number of samples
    GaussianSamplerSparsePrecision sampler_sparse_precision {mu, invSigma};
    MatrixXd samples(ndim, nn);

    //generate samples
    samples = sampler_sparse_precision(nn);

    gtsam::Vector losses(nn);
    gtsam::Vector Vdmu = gtsam::Vector::Zero(ndim);
    gtsam::Matrix Vddmu = gtsam::Matrix::Zero(ndim, ndim);
    double accum_loss = 0;
    gtsam::Vector hinge_loss(nspheres * (nsupptd_states-1));

    for (int i=0; i<100; i++)
    {
        Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr_solver(invSigma);
        // Calulating the expectations
        auto colwise = samples.colwise();

        std::for_each(colwise.begin(), colwise.end(), [&](auto const& sample){
            for (int j=0; j<nsupptd_states-1; j++){
                q_1 = gtsam::Vector{sample(seq(j*4,j*4+1))};
                q_2 = gtsam::Vector{sample(seq(j*4+2,j*4+3))};
                qdot_1 = gtsam::Vector{sample(seq(j*4+4,j*4+5))};
                qdot_2 = gtsam::Vector{sample(seq(j*4+6,j*4+7))};
                hinge_loss(seq(i*n_spheres, (i+1)*nspheres-1)) = gtsam::Vector{factor.evaluateError(q_1, qdot_1, q_2, qdot_2)};
            }

            gtsam::Matrix Obs_invSigma;
            Obs_invSigma = gtsam::Matrix::Identity(hinge_loss.size(), hinge_loss.size()) / obs_sigma;

            double loss = (hinge_loss.transpose() * Obs_invSigma * hinge_loss);
//            if (hinge_loss.sum() > 0){
//                cout << "positive loss " << endl;
//            }

            accum_loss += loss;
//            cout << "sample " << endl << sample << endl;
//            cout << "mu " << endl << mu << endl;
            cout << "loss " << endl << loss << endl;
            Vdmu = Vdmu + (sample - mu) * loss;
            Vddmu = Vddmu + (sample - mu) * (sample - mu).transpose() * loss;
        });

        cout << "accum_loss " << accum_loss << endl;

        Vdmu = invSigma * (Vdmu / nn);
        Vddmu = invSigma * (Vddmu / nn) * invSigma;
        gtsam::Matrix tmp{invSigma * (accum_loss / nn)};
        Vddmu = Vddmu - tmp;

        // Update mu and precision matrix
        d_mu = qr_solver.solve(-Vdmu);
        d_invSigma = -invSigma + Vddmu;

//        cout << "d_invSigma " << endl << d_invSigma << endl;
//        cout << "invSigma " << endl << invSigma << endl;

        mu = mu + step_size_mu * d_mu;
        invSigma = invSigma + step_size_Sigma * d_invSigma;

//        cout << "invSigma " << endl << invSigma << endl;

        // Zero grad
        d_mu.setZero();
        d_invSigma.setZero();
        Vdmu.setZero();
        Vddmu.setZero();
        samples.setZero();
        hinge_loss.setZero();

        // Truncate the precision matrix
        Sparsehelper::tridiagonalizer tridiagzer(ndim);
        invSigma = tridiagzer(invSigma);
//        cout << "invSigma tridiagonalized " << invSigma << endl;

        // Update the sampler parameters
        sampler_sparse_precision.updateMean(mu);
        sampler_sparse_precision.updatePrecisionMatrix(invSigma);

        // Sampling
        samples = sampler_sparse_precision(nn);

        cout << "samples" << endl << samples << endl;
    }

}

