# Plot planar sdf and optimization iteration results.

from turtle import color
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
# from gpmp2_python.gpmp2_python.utils.plot_utils import plotSignedDistanceField2D

sns.set_theme(style="dark")

def read_sdf(filename):
    #% read_sdf: read the sdf matrix
    #% @filename: csv file
    return np.genfromtxt(filename, delimiter=',')


def read_2d_positions(f_res_mean, f_res_cov, dim_conf, n_sample_iters=5):
    #% read_2d_positions: read the results of mean and covariances from the optimization iterations.
   
    #% @f_res_mean: filename for mean data
    #% @f_res_cov: filename for covariance data 
    #% @dim_conf: dimension of a robot configuration. e.g., 4 for planar point robot 
    #% @n_sample_iters number of iterations to collect
    #% e.g., [[mean1, mean2, mean3]_0, ..., [mean1, mean2, mean3]_n_sample_iters]

    #% Returns vectors of 2d means and 2d marginal covariance, and the indexes.

    means = np.genfromtxt(f_res_mean, delimiter=',')
    covs = np.genfromtxt(f_res_cov, delimiter=',')

    # number of states
    n_states = means[0].size // dim_conf 
    n_iters, tot_dim_conf = means.shape
    step_size = n_iters // n_sample_iters

    # vector of 2d means and covariances
    vec_means_2d = []
    vec_covs_2d = []    
    vec_index = []

    for i_step in range(n_sample_iters):
        i = i_step * step_size
        i_mean = means[i, :]
        i_cov = covs[i*tot_dim_conf:(i+1)*tot_dim_conf, 0:tot_dim_conf]

        i_vec_means_2d = []
        i_vec_covs_2d = []

        for j in range(n_states):
            i_vec_means_2d.append(i_mean[j*dim_conf:j*dim_conf+2])
            i_vec_covs_2d.append(i_cov[j*dim_conf:j*dim_conf+2, j*dim_conf:j*dim_conf+2])

        vec_means_2d.append(i_vec_means_2d)
        vec_covs_2d.append(i_vec_covs_2d)

        vec_index.append(i)

    return vec_means_2d, vec_covs_2d, vec_index


def plot_gaussian(mean, cov, c=".95", axis=None):
    # %plot_gaussian: plot one 2d gaussian distribution, given mean and covariance
    # %
    # %   Usage: plot_gaussian(mean, cov, c=".95", axis=None)
    # %   @mean     mean vector
    # %   @cov      covariance matrix
    # %   @c        color
    # %   @axis     plt Axes

    if axis is None:
        axis = plt.subplots(figsize=(20, 20))
    # Simulate data from a bivariate Gaussian
    n = 1000
    rng = np.random.RandomState(0)
    x, y = rng.multivariate_normal(mean, cov, n).T
    # sns.scatterplot(x=x, y=y, s=4, color=c, alpha=0.5, ax=axis)
    sns.histplot(x=x, y=y, bins=60, pthresh=.1, cmap="mako", alpha=0.2, ax=axis)
    # sns.kdeplot(x=x, y=y, levels=5, color="k", linewidths=1, fill=True, ax=axis)

    axis.scatter(mean[0], mean[1], s=20, c='red')

    return axis


def plotSignedDistanceField2D(
    figure, axis, field, origin_x, origin_y, cell_size, epsilon_dist=0
):
    # %PLOTSIGNEDDISTANCEFIELD2D plot 2D SignedDistanceField
    # %
    # %   Usage: PLOTSIGNEDDISTANCEFIELD2D(field, origin_x, origin_y, cell_size, epsilon_dist)
    # %   @field                  field matrix
    # %   @origin_x, origin_y     origin (down-left) corner of the map
    # %   @cell_size              cell size
    # %   @epsilon_dist           optional plot obstacle safety distance, default = 0

    # get X-Y coordinates
    grid_rows = field.shape[0]
    grid_cols = field.shape[1]
    grid_corner_x = origin_x + (grid_cols - 1) * cell_size
    grid_corner_y = origin_y + (grid_rows - 1) * cell_size

    grid_X = np.linspace(origin_x, grid_corner_x, num=grid_cols)
    grid_Y = np.linspace(origin_y, grid_corner_y, num=grid_rows)

    z_min = np.amin(field)
    z_max = np.amax(field)
    c = axis.pcolor(grid_X, grid_Y, field, cmap="YlOrRd", vmin=z_min, vmax=z_max)
    figure.colorbar(c, ax=axis)  # add colorbar

    # set(gca,'YDir','normal')
    axis.invert_yaxis()  # TODO: check this again! same as set(gca,'YDir','normal')

    axis.axis("equal")
    axis.axis(
        [
            origin_x - cell_size / 2,
            grid_corner_x + cell_size / 2,
            origin_y - cell_size / 2,
            grid_corner_y + cell_size / 2,
        ]
    )

    # colorbar
    axis.set_xlabel("X/m")
    axis.set_ylabel("Y/m")
    axis.set_title("Signed Distance Field")

    return figure, axis


def plotResult2DSdf(sdf_file, means_file, covs_file, dim_conf = 4, niter_viz = 3):

    #% plotResult2DSdf: plot the sdf and optimizing results in 2d case
    #% usage:           plotResult2DSdf(sdf_file, means_file, covs_file, dim_conf, niter_viz)
    #% @sdf_file:       sdf file name
    #% @means_file:     result data files for mean vectors
    #% @covs_file:      result data files for covariance matrices
    #% @dim_conf:       configuration variable dimension
    #$ @niter_viz:      number of iterations to plot

    # read sdf
    sdf = read_sdf(sdf_file)

    # Read the means and covariances result data
    vec_means_2d, vec_covs_2d, vec_index = read_2d_positions(means_file, covs_file, dim_conf, niter_viz)

    ## Plotting
    niters = len(vec_means_2d)
    cnt = 1;
    # for each iteration
    f, ax1 = plt.subplots(1, niter_viz, figsize=(40, 6))
    for i_iter_mean, i_iter_cov in zip(vec_means_2d, vec_covs_2d):
        f, ax1[cnt-1] = plotSignedDistanceField2D(f, ax1[cnt-1], sdf, 0, 0, 1)

        # for each state
        for j_mean, j_cov in zip(i_iter_mean, i_iter_cov):
            ax1[cnt-1] = plot_gaussian(j_mean, j_cov, c=str(0.95/niters*cnt), axis=ax1[cnt-1])
        
        ax1[cnt-1].set_xlim(-2, 8)
        ax1[cnt-1].set_ylim(-2.5, 7.5, auto=False)
        ttl = 'iteration ' + str(vec_index[cnt-1])
        ax1[cnt-1].set_title(ttl) 

        cnt += 1

    plt.show()



if __name__ == '__main__':
    # read sdf
    sdf_file = '../mpvi/data/2d_pR/map_ground_truth.csv'
  
    # result data files
    means_file = '../mpvi/data/2d_pR/mean.csv'
    covs_file = '../mpvi/data/2d_pR/cov.csv'

    # configuration variable dimension
    dim_conf = 4 # point robot

    # number of iterations to plot
    niter_viz = 5

    plotResult2DSdf(sdf_file, means_file, covs_file, dim_conf, niter_viz)

    

    
