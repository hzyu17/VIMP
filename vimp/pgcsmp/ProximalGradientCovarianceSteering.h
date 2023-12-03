/**
 * @file ProximalGradientCS.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Proximal gradient algorithm for nonlinear covariance steering.
 * @version 0.1
 * @date 2023-03-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "helpers/ExperimentParams.h"
#include "LinearCovarianceSteering.h"
#include <memory>
#include <Eigen/QR>
#include "helpers/DataRecorder.h"

using namespace Eigen;


namespace vimp{
    // return type of one step: (Kt, dt, At, at, zt, Sigt) 
    using StepResult = std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D, Matrix3D, Matrix3D>;  
    
    // history of the nonimals: (zts, Sigts)
    using NominalHistory = std::tuple<std::vector<Matrix3D>, std::vector<Matrix3D>>;

    // return type of linear covariance steering: (Kt, dt, At, at)
    using LinearCSResult = std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D>;

    class ProxGradCovSteer
    {
    public:
        ProxGradCovSteer(){};

        virtual ~ProxGradCovSteer() {}

        ProxGradCovSteer(const MatrixXd &A0,
                         const VectorXd &a0,
                         const MatrixXd &B,
                         const PGCSParams& params):
                         _ei(),
                        _nx(params.nx()),
                        _nu(params.nu()),
                        _nt(params.nt()),
                        _eta(params.step_size()),
                        _Akt(_ei.replicate3d(A0, params.nt())),
                        _akt(_ei.replicate3d(a0, params.nt())),
                        _Bt(_ei.replicate3d(B, params.nt())),
                        _total_time(params.total_time()),
                        _eps(params.eps()),
                        _deltt(params.total_time() / (params.nt() - 1)),
                        _z0(params.m0()),
                        _Sig0(params.Sig0()),
                        _zT(params.mT()),
                        _SigT(params.SigT()),
                        _max_iter(params.max_iter()),
                        _stop_err(params.stop_err()),
                        _backtrack_ratio(params.backtrack_ratio()),
                        _max_n_backtrack(params.max_n_backtrack()),
                        _Qkt(Matrix3D(params.nx(), params.nx(), params.nt())),
                        _Qt(Matrix3D(params.nx(), params.nx(), params.nt())),
                        _rkt(Matrix3D(params.nx(), 1, params.nt())),
                        _hAkt(Matrix3D(params.nx(), params.nx(), params.nt())),
                        _hakt(Matrix3D(params.nx(), 1, params.nt())),
                        _nTrt(Matrix3D(params.nx(), 1, params.nt())),
                        _pinvBBTt(Matrix3D(params.nx(), params.nx(), params.nt())),
                        _zkt(_ei.replicate3d(params.m0(), params.nt())),
                        _Sigkt(_ei.replicate3d(params.Sig0(), params.nt())),
                        _Kt(params.nu(), params.nx(), params.nt()),
                        _dt(params.nu(), 1, params.nt()),
                        _linear_cs(_Akt, _Bt, _akt, params.nx(), params.nu(), params.total_time(), params.nt(), params.eps(), _Qkt, _rkt, params.m0(), params.Sig0(), params.mT(), params.SigT()),
                        _recorder(_Akt, _Bt, _akt, _Qkt, _rkt, _Kt, _dt, _zkt, _Sigkt)
                        {

                            // Initialize the final time covariance
                            _ei.compress3d(_SigT, _Sigkt, _nt - 1);

                            // compute pinvBBT
                            MatrixXd Bi(_nx, _nu), BiT(_nu, _nx), pinvBBTi(_nx, _nx);
                            for (int i = 0; i < _nt; i++)
                            {
                                Bi = Bt_i(i);
                                BiT = Bi.transpose();
                                pinvBBTi = (Bi * BiT).completeOrthogonalDecomposition().pseudoInverse();
                                _ei.compress3d(pinvBBTi, _pinvBBTt, i);
                            }

                        }
        
        ProxGradCovSteer(const MatrixXd &A0,
                         const VectorXd &a0,
                         const MatrixXd &B,
                         double sig,
                         int nt,
                         double eta,
                         double eps,
                         const VectorXd &z0,
                         const MatrixXd &Sig0,
                         const VectorXd &zT,
                         const MatrixXd &SigT,
                         double stop_err,
                         int max_iteration = 30) : 
                         _ei(),
                        _nx(A0.rows()),
                        _nu(B.cols()),
                        _nt(nt),
                        _eta(eta),
                        _Akt(_ei.replicate3d(A0, nt)),
                        _akt(_ei.replicate3d(a0, nt)),
                        _Bt(_ei.replicate3d(B, nt)),
                        _total_time(sig),
                        _eps(eps),
                        _deltt(sig / (nt - 1)),
                        _Qkt(Matrix3D(_nx, _nx, nt)),
                        _Qt(Matrix3D(_nx, _nx, nt)),
                        _rkt(Matrix3D(_nx, 1, nt)),
                        _hAkt(Matrix3D(_nx, _nx, nt)),
                        _hakt(Matrix3D(_nx, 1, nt)),
                        _nTrt(Matrix3D(_nx, 1, nt)),
                        _pinvBBTt(Matrix3D(_nx, _nx, nt)),
                        _zkt(_ei.replicate3d(z0, nt)),
                        _Sigkt(_ei.replicate3d(Sig0, nt)),
                        _z0(z0),
                        _Sig0(Sig0),
                        _zT(zT),
                        _SigT(SigT),
                        _Kt(_nu, _nx, nt),
                        _dt(_nu, 1, nt),
                        _max_iter(max_iteration),
                        _stop_err(stop_err),
                        _linear_cs(_Akt, _Bt, _akt, _nx, _nu, _total_time, nt, _eps, _Qkt, _rkt, _z0, _Sig0, _zT, _SigT),
                        _recorder(_Akt, _Bt, _akt, _Qkt, _rkt, _Kt, _dt, _zkt, _Sigkt)
        {
            // Initialize the final time covariance
            _ei.compress3d(_SigT, _Sigkt, nt - 1);
            // compute pinvBBT
            MatrixXd Bi(_nx, _nu), BiT(_nu, _nx), pinvBBTi(_nx, _nx);
            for (int i = 0; i < nt; i++)
            {
                Bi = Bt_i(i);
                BiT = Bi.transpose();
                pinvBBTi = (Bi * BiT).completeOrthogonalDecomposition().pseudoInverse();
                _ei.compress3d(pinvBBTi, _pinvBBTt, i);
            }
        }

        /**
         * @brief The optimization process, including linearization,
         * sovling a linear CS, and push forward the mean and covariances.
         * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt)
         */
        virtual std::tuple<Matrix3D, Matrix3D, NominalHistory> optimize()
        {
            double err = 1;
            MatrixXd Ak_prev(_nx * _nx, _nt), ak_prev(_nx, _nt);
            Ak_prev = _Akt;
            ak_prev = _akt;
            int i_step = 1;
            NominalHistory hnom;
            std::vector<Matrix3D> hzt, hSigzt;
            while ((err > _stop_err) && (i_step <= _max_iter))
            {
                step(i_step);
                err = (Ak_prev - _Akt).norm() / _Akt.norm() / _nt + (ak_prev - _akt).norm() / _akt.norm() / _nt;
                Ak_prev = _Akt;
                ak_prev = _akt;
                i_step++;

                hzt.push_back(_zkt);
                hSigzt.push_back(_Sigkt);
            }

            hnom = make_tuple(hzt, hSigzt);

            return std::make_tuple(_Kt, _dt, hnom);
        }

        /**
         * @brief Backtracking to select step sizes in optimization.
         * @return std::tuple<MatrixXd, MatrixXd>  representing (Kt, dt)
         */
        virtual std::tuple<Matrix3D, Matrix3D, NominalHistory> backtrack(){}

        /**
         * @brief step with given matrices, return a total cost of this step.
         */
        virtual StepResult step(int indx, double step_size, 
                                const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at,
                                const Matrix3D& hAt, const Matrix3D& hat, 
                                const VectorXd& z0, const MatrixXd& Sig0){}

        /**
         * @brief step with given matrices, return a total cost of this step.
         */
        virtual StepResult step(int indx, double step_size, 
                                const Matrix3D& At, const Matrix3D& Bt, const Matrix3D& at, 
                                const VectorXd& z0, const MatrixXd& Sig0){}


        void update_from_step_res(const StepResult& res){
            // return type of one step: (Kt, dt, At, at, zt, Sigt) 
            _Kt = std::get<0>(res); _dt = std::get<1>(res);
            _Akt = std::get<2>(res); _akt = std::get<3>(res);
            _zkt = std::get<4>(res); _Sigkt = std::get<5>(res);
        }

        virtual void step(int indx) = 0;

        /**
         * @brief Qrk with given matrices.
         * return: (Qt, rt)
         */
        virtual std::tuple<Matrix3D, Matrix3D> update_Qrk(const Matrix3D& zt, const Matrix3D& Sigt, 
                                                        const Matrix3D& At,  const Matrix3D& at, 
                                                        const Matrix3D& Bt, const Matrix3D& hAt,
                                                        const Matrix3D& hat, const double step_size)
                                                        {}

        
        /**
         * @brief Qrk with given matrices for nonlinear dynamics.
         * return: (Qt, rt)
         */
        virtual std::tuple<Matrix3D, Matrix3D> update_Qrk_NL(const Matrix3D& zt, const Matrix3D& Sigt, 
                                                            const Matrix3D& At,  const Matrix3D& at, 
                                                            const Matrix3D& Bt, const Matrix3D& hAt,
                                                            const Matrix3D& hat, const Matrix3D& nTrt,
                                                            const double step_size)
                                                            {}


        /**
         * @brief Problem with a state cost V(Xt) differs only in the expressions Qk and rk.
         */
        virtual void update_Qrk(){ }

        /**
         * @brief solve linear CS with local matrix inputs. 
         * @return std::tuple<K, d, A, a> where A, a are close-loop linear system already containing the feedback control. 
         */
        LinearCSResult solve_linearCS_return(const MatrixXd &A, 
                                             const MatrixXd &B, 
                                             const MatrixXd &a, 
                                             const MatrixXd &Q, 
                                             const MatrixXd &r)
        {
            // solve for the linear covariance steering
            _linear_cs.update_params(A, B, a, Q, r);
            _linear_cs.solve();

            // retrieve (K, d)
            Matrix3D Kt(_nu, _nx, _nt), dt(_nu, 1, _nt), At(_nx, _nx, _nt), at(_nx, 1, _nt);
            Kt = _linear_cs.Kt();
            dt = _linear_cs.dt();

            MatrixXd Ai(_nx, _nx), ai(_nx, 1), Bi(_nx, _nu), Aprior_i(_nx, _nx), aprior_i(_nx, 1), Ki(_nu, _nx), di(_nx, 1);
            for (int i = 0; i < _nt; i++)
            {
                Aprior_i = _ei.decomp3d(A, _nx, _nx, i);
                aprior_i = _ei.decomp3d(a, _nx, 1, i);

                Bi = Bt_i(i);
                Ki = _ei.decomp3d(Kt, _nu, _nx, i);
                di = _ei.decomp3d(dt, _nu, 1, i);

                Ai = Aprior_i + Bi * Ki;
                ai = aprior_i + Bi * di;

                _ei.compress3d(Ai, At, i);
                _ei.compress3d(ai, at, i);
            }
            return std::make_tuple(Kt, dt, At, at);
        }

        void solve_linearCS(const MatrixXd &A, const MatrixXd &B, const MatrixXd &a, const MatrixXd &Q, const MatrixXd &r)
        {
            LinearCSResult KtdtAtat;
            KtdtAtat = solve_linearCS_return(A, B, a, Q, r);
            _Kt = std::get<0>(KtdtAtat); _dt = std::get<1>(KtdtAtat);
            _Akt = std::get<2>(KtdtAtat); _akt = std::get<3>(KtdtAtat);
        }

        std::tuple<Matrix3D, Matrix3D> propagate_nominal(const Matrix3D& At, 
                                                         const Matrix3D& at, 
                                                         const Matrix3D& Bt, 
                                                         const VectorXd& z0, 
                                                         const MatrixXd& Sig0)
        {
            // The i_th matrices
            Eigen::VectorXd zi(_nx), znew(_nx), zt_next(_nx), ai(_nx), ai_next(_nx);
            Eigen::MatrixXd Ai(_nx, _nx), Bi(_nx, _nu), AiT(_nx, _nx), BiT(_nu, _nx); 
            Eigen::MatrixXd Si(_nx, _nx), Snew(_nx, _nx), Si_next(_nx, _nx);
            Eigen::MatrixXd Ai_next(_nx, _nx), Bi_next(_nx, _nu), AiT_next(_nx, _nx), BiT_next(_nu, _nx);
            Matrix3D zt_new(_nx, 1, _nt), Sigt_new(_nx, _nx, _nt);

            zt_new.setZero();
            Sigt_new.setZero();

            _ei.compress3d(z0, zt_new, 0);
            _ei.compress3d(Sig0, Sigt_new, 0);

            for (int i = 0; i < _nt - 1; i++)
            {
                Ai = _ei.decomp3d(At, _nx, _nx, i);
                ai = _ei.decomp3d(at, _nx, 1, i);
                Bi = _ei.decomp3d(Bt, _nx, _nu, i);
                AiT = Ai.transpose();
                BiT = Bi.transpose();

                zi = _ei.decomp3d(zt_new, _nx, 1, i);
                Si = _ei.decomp3d(Sigt_new, _nx, _nx, i);
                
                // Heun's method
                zt_next = zi + _deltt * (Ai * zi + ai);
                Si_next = Si + _deltt * (Ai * Si + Si * AiT + _eps * (Bi * BiT));

                Ai_next = _ei.decomp3d(At, _nx, _nx, i+1);
                ai_next = _ei.decomp3d(at, _nx, 1, i+1);
                Bi_next = _ei.decomp3d(Bt, _nx, _nu, i+1);
                AiT_next = Ai_next.transpose();
                BiT_next = Bi_next.transpose();

                VectorXd ztnew_i = VectorXd::Zero(_nx);
                MatrixXd Snew_i = MatrixXd::Zero(_nx, _nx);

                ztnew_i = zi + _deltt*((Ai*zi + ai) + (Ai_next*zt_next + ai_next)) / 2.0;
                Snew_i = Si + _deltt*((Ai*Si + Si*AiT + _eps*(Bi*BiT)) + (Ai_next*Si_next + Si_next*AiT_next + _eps*(Bi_next*BiT_next))) / 2.0;

                _ei.compress3d(ztnew_i, zt_new, i + 1);
                _ei.compress3d(Snew_i, Sigt_new, i + 1);

            }

            return std::make_tuple(zt_new, Sigt_new);
        }

        void propagate_nominal()
        {   
            std::tuple<Matrix3D, Matrix3D> ztSigt;
            ztSigt = propagate_nominal(_Akt, _akt, _Bt, _z0, _Sig0);
            _zkt = std::get<0>(ztSigt); _Sigkt = std::get<1>(ztSigt);
        }


        inline Matrix3D zkt() { return _zkt; }

        inline Matrix3D Sigkt() { return _Sigkt; }

        inline Matrix3D Akt() { return _Akt; }

        inline Matrix3D akt() { return _akt; }

        inline Matrix3D hAkt() { return _hAkt; }

        inline Matrix3D hakt() { return _hakt; }

        inline Matrix3D Qkt() { return _Qkt; }

        inline Matrix3D rkt() { return _rkt; }

        /**
         * @brief get the matrices at specific time point i.
         */

        inline MatrixXd Bt_i(int i){ return _ei.decomp3d(_Bt, _nx, _nu, i);}

        /**
         * @brief replicating a fixed state cost
         */
        void repliacteQt(MatrixXd Q0) { _Qt = _ei.replicate3d(Q0, _nt); }

    protected:
        EigenWrapper _ei;
        int _nx, _nu, _nt;
        double _eta, _total_time, _eps, _deltt, _stop_err, _backtrack_ratio;
        int _max_iter, _max_n_backtrack;

        // All the variables are time variant (3d matrices)
        // iteration variables
        Matrix3D _Akt, _Bt, _akt, _pinvBBTt;
        Matrix3D _Qkt, _Qt; // Qk is the Q in each iteration, and Qt is the quadratic state cost matrix.
        Matrix3D _rkt;

        // linearizations
        Matrix3D _hAkt, _hakt, _nTrt;

        // boundary conditions
        Matrix3D _Sigkt, _zkt;
        MatrixXd _Sig0, _SigT;
        VectorXd _z0, _zT;

        // Final result
        Matrix3D _Kt, _dt;

        // Data recorder for iteration plot
        PGCSDataRecorder _recorder;

        // Dynamics class
        LinearCovarianceSteering _linear_cs;

    };
}