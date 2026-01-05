/**
 * @file PGCSDataRecorder.h
 * @brief Records iteration data for Proximal Gaussian Covariance Steering.
 */

#pragma once

#include <vector>
#include "GaussianVI/helpers/EigenWrapper.h"

namespace vimp {

using gvi::Matrix3D;

/**
 * @brief Stores per-iteration data from PGCS optimization.
 * 
 * Records system matrices, cost parameters, and controller gains
 * at each iteration for analysis or debugging.
 */
class PGCSDataRecorder {
public:
    PGCSDataRecorder() = default;

    PGCSDataRecorder(const Matrix3D& Akt, const Matrix3D& Bt, const Matrix3D& akt,
                     const Matrix3D& Qkt, const Matrix3D& rkt,
                     const Matrix3D& Kkt, const Matrix3D& dkt,
                     const Matrix3D& zkt, const Matrix3D& Sigkt) {
        add_iteration(Akt, Bt, akt, Qkt, rkt, Kkt, dkt, zkt, Sigkt);
    }

    void add_iteration(const Matrix3D& Akt, const Matrix3D& Bt, const Matrix3D& akt,
                       const Matrix3D& Qkt, const Matrix3D& rkt,
                       const Matrix3D& Kkt, const Matrix3D& dkt,
                       const Matrix3D& zkt, const Matrix3D& Sigkt) {
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

    // ==================== Accessors ====================

    size_t num_iterations() const { return _Akt.size(); }

    // Dynamics matrices
    const std::vector<Matrix3D>& Akt()   const { return _Akt; }
    const std::vector<Matrix3D>& Bt()    const { return _Bt; }
    const std::vector<Matrix3D>& akt()   const { return _akt; }

    // Cost matrices
    const std::vector<Matrix3D>& Qkt()   const { return _Qkt; }
    const std::vector<Matrix3D>& rkt()   const { return _rkt; }

    // Controller gains
    const std::vector<Matrix3D>& Kkt()   const { return _Kkt; }
    const std::vector<Matrix3D>& dkt()   const { return _dkt; }

    // Nominal trajectory
    const std::vector<Matrix3D>& zkt()   const { return _zkt; }
    const std::vector<Matrix3D>& Sigkt() const { return _Sigkt; }

    // Per-iteration access
    const Matrix3D& Akt(size_t iter)   const { return _Akt[iter]; }
    const Matrix3D& Bt(size_t iter)    const { return _Bt[iter]; }
    const Matrix3D& akt(size_t iter)   const { return _akt[iter]; }
    const Matrix3D& Qkt(size_t iter)   const { return _Qkt[iter]; }
    const Matrix3D& rkt(size_t iter)   const { return _rkt[iter]; }
    const Matrix3D& Kkt(size_t iter)   const { return _Kkt[iter]; }
    const Matrix3D& dkt(size_t iter)   const { return _dkt[iter]; }
    const Matrix3D& zkt(size_t iter)   const { return _zkt[iter]; }
    const Matrix3D& Sigkt(size_t iter) const { return _Sigkt[iter]; }

private:
    // Dynamics: dx = (A*x + B*u + a)dt
    std::vector<Matrix3D> _Akt, _Bt, _akt;

    // Running cost: x'Qx + r'x
    std::vector<Matrix3D> _Qkt, _rkt;

    // Feedback controller: u = K*x + d
    std::vector<Matrix3D> _Kkt, _dkt;

    // Nominal mean and covariance trajectory
    std::vector<Matrix3D> _zkt, _Sigkt;
};

} // namespace vimp