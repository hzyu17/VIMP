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

#include "helpers/EigenWrapper.h"
#include <vector>

using namespace Eigen;
using namespace std;

namespace vimp
{
    /**
     * @brief Sturture to store the matrices for backtracking and cost analysis.
     */
    class DataBuffer{
    public:
        DataBuffer(){}
        DataBuffer(const Matrix3D& At, 
                    const Matrix3D& Bt, 
                    const Matrix3D& at,
                    const Matrix3D& pinvBBTt,
                    const Matrix3D& Qt,
                    const Matrix3D& rt,
                    const Matrix3D& hAt,
                    const Matrix3D& hat,
                    const Matrix3D& nTrt,
                    const Matrix3D& zt,
                    const Matrix3D& Sigt)
                    {
                        update(At, Bt, at, pinvBBTt, Qt, rt, hAt, hat, nTrt, zt, Sigt);
                    }
                    
        void update(const Matrix3D& At, 
                    const Matrix3D& Bt, 
                    const Matrix3D& at,
                    const Matrix3D& pinvBBTt,
                    const Matrix3D& Qt,
                    const Matrix3D& rt,
                    const Matrix3D& hAt,
                    const Matrix3D& hat,
                    const Matrix3D& nTrt,
                    const Matrix3D& zt,
                    const Matrix3D& Sigt){
                        _At = At;
                        _Bt = Bt;
                        _at = at;
                        _pinvBBTt = pinvBBTt;
                        _Qt = Qt;
                        _rt = rt;
                        _hAt = hAt;
                        _hat = hat;
                        _nTrt = nTrt;
                        _zt = zt;
                        _Sigt = Sigt;
                    }

        void update_nominal(const Matrix3D& zt, const Matrix3D& Sigt){
            _zt = zt;
            _Sigt = Sigt;
        }


    private:
        EigenWrapper _ei;

        // 3D matrices
        Matrix3D _At, _Bt, _at, _pinvBBTt;
        Matrix3D _Qt, _rt;

        // linearizations
        Matrix3D _hAt, _hat, _nTrt;

        // nominals
        Matrix3D _zt, _Sigt;

    };

    /**
     * @brief A class which record the iterations of data.
     */
    class DataRecorder{
    public:
        DataRecorder(){}
        DataRecorder(const Matrix3D& Akt, 
                    const Matrix3D& Bt, 
                    const Matrix3D& akt,
                    const Matrix3D& Qkt,
                    const Matrix3D& rkt,
                    const Matrix3D& Kkt,
                    const Matrix3D& dkt,
                    const Matrix3D& zkt,
                    const Matrix3D& Sigkt)
                    {
                        add_iteration(Akt, Bt, akt, Qkt, rkt, Kkt, dkt, zkt, Sigkt);
                    }
                    
        void add_iteration(const Matrix3D& Akt, 
                        const Matrix3D& Bt, 
                        const Matrix3D& akt,
                        const Matrix3D& Qkt,
                        const Matrix3D& rkt,
                        const Matrix3D& Kkt,
                        const Matrix3D& dkt,
                        const Matrix3D& zkt,
                        const Matrix3D& Sigkt){
                            
                        _Akt.push_back(Akt);
                        _Bt.push_back(Bt);
                        _akt.push_back(akt);
                        _Qkt.push_back(Qkt);
                        _rkt.push_back(rkt);
                        _Kkt.push_back(Kkt),
                        _dkt.push_back(dkt);
                        _zkt.push_back(zkt);
                        _Sigkt.push_back(Sigkt);
                    }

        void write_records(){
            
        }


    private:
        EigenWrapper _ei;

        // 3D matrices
        vector<Matrix3D> _Akt, _Bt, _akt;
        vector<Matrix3D> _Qkt, _rkt;
        vector<Matrix3D> _Kkt, _dkt;

        // linearizations
        vector<Matrix3D> _hAkt, _hakt;

        // nominal means and covariances
        vector<Matrix3D> _zkt, _Sigkt;

    };

} // namespace vimp