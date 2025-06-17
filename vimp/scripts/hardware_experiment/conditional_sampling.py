import torch
import numpy as np

def conditional_sample(joint_mean, joint_cov, state_dim):
    # ---- inputs ---------------------------------------------------------
    # Assume each state is a column vector of size state_dim.
    # mean : (N*state_dim,)
    # cov  : (N*state_dim, N*state_dim)
    
    # Determine if we should return NumPy
    return_numpy = isinstance(joint_mean, np.ndarray) or isinstance(joint_cov, np.ndarray)

    # Convert NumPy input to torch tensors
    if isinstance(joint_mean, np.ndarray):
        joint_mean = torch.from_numpy(joint_mean).double()
    if isinstance(joint_cov, np.ndarray):
        joint_cov = torch.from_numpy(joint_cov).double()

    N = joint_mean.numel() // state_dim     # total number of states
    
    x1 = joint_mean[:state_dim]             # shape (state_dim,)
    xN = joint_mean[(N-1)*state_dim:]       # shape (state_dim,)

    # index helpers -------------------------------------------------------
    def block(i):             # block corresponding to x_i (1-based)
        start = (i-1)*state_dim
        return slice(start, start+state_dim)

    idx_z = torch.cat([torch.arange(block(k).start, block(k).stop) for k in range(2, N)])
    idx_y = torch.cat([torch.arange(block(1).start, block(1).stop),
                    torch.arange(block(N).start, block(N).stop)])

    # partition matrices and vectors -------------------------------------
    mu_z = joint_mean[idx_z]                        # (state_dim*(N-2),)
    mu_y = joint_mean[idx_y]                        # (2d,)
    Sigma_zz = joint_cov[idx_z][:, idx_z]           # (state_dim*(N-2), state_dim*(N-2))
    Sigma_zy = joint_cov[idx_z][:, idx_y]           # (state_dim*(N-2), 2d)
    Sigma_yy = joint_cov[idx_y][:, idx_y]           # (2d, 2d)

    y0 = torch.cat([x1, xN])                        # (2d,)

    # conditional mean / cov ------------------------------------------------
    # Solve with a linear system instead of explicit inverse for stability.
    K = torch.linalg.solve(Sigma_yy, (y0 - mu_y))          # Σ_yy^{-1}(y0-μ_y)
    cond_mean = mu_z + Sigma_zy @ K

    L = torch.linalg.solve(Sigma_yy, Sigma_zy.T)           # Σ_yy^{-1} Σ_yz
    cond_cov  = Sigma_zz - Sigma_zy @ L                    # Schur complement

    if return_numpy:
        return cond_mean.numpy(), cond_cov.numpy()
    else:
        return cond_mean, cond_cov