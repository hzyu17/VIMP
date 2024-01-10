
#pragma once

#ifndef PGCS_DATARECORDER_H
#define PGCS_DATARECORDER_H

#include <vector>
#include "helpers/EigenWrapper.h"

namespace vimp{

class PGCSDataRecorder{
public:
    PGCSDataRecorder(){}
    PGCSDataRecorder(const Matrix3D& Akt, 
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
                    _Kkt.push_back(Kkt);
                    _dkt.push_back(dkt);
                    _zkt.push_back(zkt);
                    _Sigkt.push_back(Sigkt);
                }


private:
    EigenWrapper _ei;

    // 3D matrices
    std::vector<Matrix3D> _Akt, _Bt, _akt;
    std::vector<Matrix3D> _Qkt, _rkt;
    std::vector<Matrix3D> _Kkt, _dkt;

    // linearizations
    std::vector<Matrix3D> _hAkt, _hakt;

    // nominal means and covariances
    std::vector<Matrix3D> _zkt, _Sigkt;

};

}

#endif // PGCS_DATARECORDER_H