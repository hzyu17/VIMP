# Plot planar sdf and optimization iteration results.

from cvxpy import length
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from sklearn import neural_network
sns.set_theme(style="dark")

# read the sdf matrix
def read_sdf(filename):
    return np.genfromtxt(filename, delimiter=',')

# read the results of mean and covariances from the optimization iterations.
# returns vectors of 2d means and 2d marginal covariance matrices.
# @param: f_res_mean: filename for mean data
# @param: f_res_cov: filename for covariance data 
# @param: dim_conf: dimension of a robot configuration. e.g., 4 for planar point robot 
# @output: vector (n_iters) of vector(n_states) of 2d vectors and matrices
# e.g., [[mean1, mean2, mean3], [mean1, mean2, mean3]]
#  
def read_2d_positions(f_res_mean, f_res_cov, dim_conf):
    means = np.genfromtxt(f_res_mean, delimiter=',')
    covs = np.genfromtxt(f_res_cov, delimiter=',')

    # number of states
    n_states = means.size // dim_conf 

    n_iters, tot_dim_conf = means.shape

    # vector of 2d means and covariances
    vec_means_2d = []
    vec_covs_2d = []

    for i in range(n_iters):
        i_mean = means[i, :]
        i_cov = covs[i*tot_dim_conf:(i+1)*tot_dim_conf, i*tot_dim_conf:(i+1)*tot_dim_conf]

        i_vec_means_2d = []
        i_vec_covs_2d = []

        for j in range(n_states):
            i_vec_means_2d.append(i_mean[j*dim_conf:j*dim_conf+2])
            i_vec_covs_2d.append(i_cov[j*dim_conf:j*dim_conf+2, j*dim_conf:j*dim_conf+2])

        vec_means_2d.append(i_vec_means_2d)
        vec_covs_2d.append(i_vec_covs_2d)

    return vec_means_2d, vec_covs_2d


# plot one 2d gaussian distribution, given mean and covariance
def plot_gaussian(mean, cov, c=".95"):
    # Simulate data from a bivariate Gaussian
    n = 1000
    rng = np.random.RandomState(0)
    x, y = rng.multivariate_normal(mean, cov, n).T
    sns.scatterplot(x=x, y=y, s=4, color=c, alpha=0.5)
    sns.histplot(x=x, y=y, bins=10, pthresh=.1, cmap="mako", alpha=0.5)
    sns.kdeplot(x=x, y=y, levels=3, color="k", linewidths=1)


if __name__ == '__main__':

    # plot sdf
    sdf_file = '../mpvi/data/map_ground_truth.csv'
    sdf = read_sdf(sdf_file)
    
    # # Draw a combo histogram and scatterplot with density contours
    # f, ax = plt.subplots(figsize=(6, 6))
    # for _ in range(3):
    #     mean = np.random.randn(2)
    #     cov = [(2, .4), (.4, .2)]
    #     plot_gaussian(mean, cov)

    # Read the means and covariances result data
    f_means = '../mpvi/data/mean.csv'
    f_covs = '../mpvi/data/cov.csv'
    dim_conf = 4 # for point robot
    vec_means_2d, vec_covs_2d = read_2d_positions(f_means, f_covs, dim_conf)

    # plotting
    niters = len(vec_means_2d)
    cnt = 1;
    f, ax = plt.subplots(figsize=(10, 10))
    for i_iter_mean, i_iter_cov in zip(vec_means_2d, vec_covs_2d):
        # each iteration data, differentiated using different colors.
        for j_mean, j_cov in zip(i_iter_mean, i_iter_cov):
            print("j_mean")
            print(j_mean)
            print("j_cov")
            print(j_cov)
            plot_gaussian(j_mean, j_cov, c=str(0.95/niters*cnt))
        cnt += 1

    plt.show()
