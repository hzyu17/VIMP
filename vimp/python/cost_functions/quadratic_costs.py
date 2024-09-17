# Defines the quadratic loss functionals
# Hongzhe Yu
# 12/06/2023

from numba import njit
import numpy as np
from numba import float64

# @njit(float64(float64[:], float64[:,:], float64[:], float64), cache=True)
def xQrx(x, Q, r, constant):
     return x.T@Q@x/2.0 + r.T@x + constant

# @njit(float64(float64[:]), cache=True)
def uTu(u):
    return u.T@u / 2.0

def rt_xr(x_trj, xr_trj, Qt):
    """Compute the r(t) in the quadratic lqr cost, given a state trj and reference trj.
    Args:
        x_trj (numpy array): state trajectory
        xr_trj (numpy array): reference trajectory 
        Qt (numpy array): time varying quadratic Q matrix
    """
    nt, nx = x_trj.shape
    rt = np.zeros((nt, nx), dtype=np.float64)
    for i in range(nt):
        rt[i] = -Qt[i]@xr_trj[i]
    return rt

def xQrx_xr(x, xr, Q):
    """The cost with respect to a reference state.
    Args:
        x (numpy array): the inquired state
        xr (numpy array): the reference state
        Q (numpy array): the state quadratic penalty matrix
    """
    r = -Q@xr
    constant = np.transpose(xr)@Q@xr / 2.0
    return xQrx(x, Q, r, constant)

def xr_track(track_data, state):
    """ Find the point on the track that has the closest distance to the given state.

    Args:
        track_data (numpy array): the track data points in shape (N, 2)
        point2 (numpy array): the inquired 2-D position (2)
        
    Returns:
        xr: the point on the track that has the minimum dist to the inquired state.
    """
    nx = len(state)
    xr = np.zeros(nx)
    diff_track_xt = track_data - state[:2]
    dist_2_track = np.sqrt(np.sum(diff_track_xt * diff_track_xt, axis=1))
    min_index = np.argmin(dist_2_track)
    xr[0:2] = track_data[min_index]
    return xr

def xr_track_trj(track_data, nominal_trj):
    """ Find the reference trajectory the track that has the closest distance to the given state trajectory.
    Args:
        track_data (numpy array): the track data points in shape (N, 2)
        nominal_trj (numpy array): the nominal trajectory in shape (nt, nx)
    """
    nt, nx = nominal_trj.shape
    xr_trj = np.zeros((nt, nx), dtype=np.float64)
    for i in range(nt):
        xr_trj[i] = xr_track(track_data, nominal_trj[i])
    return xr_trj