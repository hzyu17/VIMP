import torch
from torch.distributions.multivariate_normal import MultivariateNormal


def conditional_sample(joint_mean, joint_cov, state_dim):
    # ---- inputs ---------------------------------------------------------
    # Assume each state is a column vector of size d.
    # mean : (N*d,)
    # cov  : (N*d, N*d)
    
    N = joint_mean.size() // state_dim                  # total number of states
    
    x1_val = joint_mean[:state_dim]              # shape (d,)
    xN_val = joint_mean[(N-1)*state_dim:]              # shape (d,)
    # --------------------------------------------------------------------

    d = x1_val.numel()

    # index helpers -------------------------------------------------------
    def block(i):             # block corresponding to x_i (1-based)
        start = (i-1)*d
        return slice(start, start+d)

    idx_z = torch.cat([torch.arange(block(k).start, block(k).stop) for k in range(2, N)])
    idx_y = torch.cat([torch.arange(block(1).start, block(1).stop),
                    torch.arange(block(N).start, block(N).stop)])

    # partition matrices and vectors -------------------------------------
    mu_z = joint_mean[idx_z]                       # (d*(N-2),)
    mu_y = joint_mean[idx_y]                       # (2d,)
    Sigma_zz = joint_cov[idx_z][:, idx_z]          # (d*(N-2), d*(N-2))
    Sigma_zy = joint_cov[idx_z][:, idx_y]          # (d*(N-2), 2d)
    Sigma_yy = joint_cov[idx_y][:, idx_y]          # (2d, 2d)

    y0 = torch.cat([x1_val, xN_val])         # (2d,)

    # conditional mean / cov ------------------------------------------------
    # Solve with a linear system instead of explicit inverse for stability.
    K = torch.linalg.solve(Sigma_yy, (y0 - mu_y))          # Σ_yy^{-1}(y0-μ_y)
    cond_mean = mu_z + Sigma_zy @ K

    L = torch.linalg.solve(Sigma_yy, Sigma_zy.T)           # Σ_yy^{-1} Σ_yz
    cond_cov  = Sigma_zz - Sigma_zy @ L                    # Schur complement
    # ----------------------------------------------------------------------

    # sample ---------------------------------------------------------------
    mvn = MultivariateNormal(cond_mean, covariance_matrix=cond_cov)
    sample = mvn.sample()                                  # (d*(N-2),)
    x2_to_xNm1 = sample.reshape(N-2, d)                    # convenient shape
    
    return x2_to_xNm1

