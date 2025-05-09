import csv
import os
import numpy as np
import torch
from conditional_sampling import conditional_sample


this_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = this_dir + "/../../../matlab_helpers/GVIMP-examples/2d_pR/map2/case2"

mean_file = data_dir+"/mean.csv"
precision_file = data_dir+"/joint_precision.csv"


df_mean = np.loadtxt(mean_file, delimiter=",", dtype=np.float32, skiprows=0)    
mean = torch.from_numpy(df_mean)[:, -1].flatten()

df_joint_precision = np.loadtxt(precision_file, delimiter=",", dtype=np.float32, skiprows=0)
joint_precision = torch.from_numpy(df_joint_precision)[:,-1].reshape(60,60)
joint_cov = torch.linalg.inv(joint_precision)

samples = conditional_sample(mean, joint_cov, state_dim=4)

def plot_cov(Sigma, mu, k=3, ax=None):
    from matplotlib.patches import Ellipse
    eigvals, eigvecs = np.linalg.eigh(Sigma)   # eigh → guaranteed sorted λ1≤λ2
    order = eigvals.argsort()[::-1]            # make λ1 ≥ λ2 for nicer handling
    eigvals, eigvecs = eigvals[order], eigvecs[:,order]

    a, b = k*np.sqrt(eigvals)                  # semi-axis lengths
    theta = np.degrees(np.arctan2(*eigvecs[:,0][::-1]))  # principal angle in deg

    if ax is None:
        fig, ax = plt.subplots()
    ax.scatter(*mu, c='red', s=20, label='mean')
    ax.add_patch(Ellipse(mu, 2*a, 2*b, theta, fc='none', ec='blue', lw=2, label='3σ contour'))
    ax.set_aspect('equal')
    
    plt.show()


import matplotlib.pyplot as plt

marginal_cov_file = data_dir+"/cov.csv"
marginal_mean_file = data_dir+"/zk_sdf.csv"
df_marginal_cov = np.loadtxt(marginal_cov_file, delimiter=",", dtype=np.float32, skiprows=0)
df_marginal_mean = np.loadtxt(marginal_mean_file, delimiter=",", dtype=np.float32, skiprows=0)

marginal_mean = torch.from_numpy(df_marginal_mean)
marginal_covs = torch.from_numpy(df_marginal_cov)[:,-1].reshape(4,4,15)

fig, ax = plt.subplots(1, 1, figsize=(8, 8))
ax.plot(samples[:, 0], samples[:, 1], 'o', markersize=2, label='Conditional Samples')

for i in range(marginal_covs.shape[2]):
    Sigma = marginal_covs[:2, :2, i]
    mu = marginal_mean[:2, i]
    plot_cov(Sigma, mu, k=3, ax=ax)

plt.show()