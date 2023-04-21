/**
 * @file DataBuffer.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Data buffer class which is used for backtracking and cost analysis.
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../helpers/eigen_wrapper.h"

using namespace Eigen;

namespace vimp
{
    /**
     * @brief Sturture to store the matrices for backtracking and cost analysis.
     */
    class DataBuffer{
    public:
        DataBuffer(){}
        DataBuffer(const Matrix3D& Akt, 
                    const Matrix3D& Bt, 
                    const Matrix3D& akt,
                    const Matrix3D& pinvBBTt,
                    const Matrix3D& Qkt,
                    const Matrix3D& rkt,
                    const Matrix3D& hAkt,
                    const Matrix3D& hakt,
                    const Matrix3D& nTrt)
                    {
                        update(Akt, Bt, akt, pinvBBTt, Qkt, rkt, hAkt, hakt, nTrt);
                    }
                    
        void update(const Matrix3D& Akt, 
                    const Matrix3D& Bt, 
                    const Matrix3D& akt,
                    const Matrix3D& pinvBBTt,
                    const Matrix3D& Qkt,
                    const Matrix3D& rkt,
                    const Matrix3D& hAkt,
                    const Matrix3D& hakt,
                    const Matrix3D& nTrt){
                        _Akt = Akt;
                        _Bt = Bt;
                        _akt = akt;
                        _pinvBBTt = pinvBBTt;
                        _Qkt = Qkt;
                        _rkt = rkt;
                        _hAkt = hAkt;
                        _hakt = hakt;
                        _nTrt = nTrt;
                    }

        void update_forward(){
            _Akt = _Akt_next;
            _Bt = _Bt_next;
            _akt = _akt_next;
            _pinvBBTt = _pinvBBTt_next;
            _Qkt = _Qkt_next;
            _rkt = _rkt_next;
            _hAkt = _hAkt_next;
            _hakt = _hakt_next;
            _nTrt = _nTrt_next;
        }

        void update_Akt_i_next(const MatrixXd& Ai, int i){ _ei.compress3d(Ai, _Akt_next, i); }

        void update_akt_i_next(const MatrixXd& ai, int i){ _ei.compress3d(ai, _akt_next, i); }

        void update_Bt_i_next(const MatrixXd& Bi, int i){ _ei.compress3d(Bi, _Bt_next, i); }

        void update_hAkt_i_next(const MatrixXd& hAi, int i){ _ei.compress3d(hAi, _hAkt_next, i); }

        void update_hakt_i_next(const MatrixXd& hai, int i){ _ei.compress3d(hai, _hakt_next, i); }

        void update_nTrt_i_next(const MatrixXd& nTri, int i){ _ei.compress3d(nTri, _nTrt_next, i); }

        void update_Qkt_i_next(const MatrixXd& Qki, int i){ _ei.compress3d(Qki, _Qkt_next, i); }

        void update_rkt_i_next(const MatrixXd& rki, int i){ _ei.compress3d(rki, _rkt_next, i); }

    private:
        EigenWrapper _ei;

        // 3D matrices
        Matrix3D _Akt, _Bt, _akt, _pinvBBTt;
        Matrix3D _Qkt, _rkt;

        // linearizations
        Matrix3D _hAkt, _hakt, _nTrt;

        // Next time matrices
        // 3D matrices
        Matrix3D _Akt_next, _Bt_next, _akt_next, _pinvBBTt_next;
        Matrix3D _Qkt_next, _Qt_next; // Qk is the Q in each iteration, and Qt is the quadratic state cost matrix.
        Matrix3D _rkt_next;

        // linearizations
        Matrix3D _hAkt_next, _hakt_next, _nTrt_next;
    };

} // namespace vimp