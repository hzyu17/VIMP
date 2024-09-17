## Plot sampled nonlinear dynamics trajectories for a time-varyting 2d data
# Hongzhe Yu, 12/27/2023

import os, sys

file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(file_path)
sys.path.append(current_dir)

from plot_tube import *

def plot_timevarying_trj(trajectory, tf, time_step, fig, color='b'):
    """Plot a time dependent 2d trajectory.

    Args:
        trajectory (nparray): should have the shape: (nt, 2)
        tf (float64): time span
        time_step (int): the step skipped when plotting
        fig (plotly.graph_objects): fig = go.Figure(layout=layout)
        color (str): color of the plot
    Returns:
        _type_: _description_
    """
    
    nt, _ = trajectory.shape
    ts = np.linspace(0, tf, nt)
    
    fig.add_trace(
        go.Scatter3d(
            x=ts[::time_step],
            y=trajectory[::time_step,0],
            z=trajectory[::time_step,1],
            mode='lines',
            line={'width': 1.8, 'color': color}
        )
    )
    return fig


# trajectory_samples should have the shape: (nt, 2, n_samples)
def plot_samples_2d(trajectory_samples, tf, fig, 
                    x0, Sig0, xT, SigT, 
                    time_step=1, subsample=200, 
                    xlabel=r'X', ylabel=r'Y', camera_eye=dict(x=2.5, y=0.5, z=1.0), ylims=None, zlims=None):
    nt, nx, nsamples = trajectory_samples.shape
    ts = np.linspace(0, tf, nt)
    
    ymin = 10
    ymax = -10
    zmin = 10
    zmax = -10
    
    for i_s in range(0, nsamples, subsample):
        xtrj_sample_i = trajectory_samples[:,:,i_s]
        
        iymin = np.min(xtrj_sample_i[:,0])
        iymax = np.max(xtrj_sample_i[:,0])
        
        izmin = np.min(xtrj_sample_i[:,1])
        izmax = np.max(xtrj_sample_i[:,1])
        
        if iymin < ymin: ymin = iymin
        if izmin < zmin: zmin = izmin
        
        if iymax > ymax: ymax = iymax
        if izmax > zmax: zmax = izmax
            
        if ylims is None:
            ylims = [ymin-1, ymax+1]
            
        if zlims is None:
            zlims = [zmin-1, zmax+1]
        
        
        fig.add_trace(
            go.Scatter3d(
                x=ts[::time_step],
                y=xtrj_sample_i[::time_step,0],
                z=xtrj_sample_i[::time_step,1],
                mode='lines',
                line={'width': 0.8}
            )
        )
        
    # plot start and goal covariance
    n_ellip_pts = 50
    samples_index = np.array([0, 1], dtype=np.int64)
    samples_t = np.array([0.0, tf], dtype=np.float64)
    samples_t = np.repeat(samples_t[:, np.newaxis], n_ellip_pts, 1)

    trj_mean = np.zeros((2, 2), dtype=np.float64)
    trj_cov = np.zeros((2, 2, 2), dtype=np.float64)

    trj_mean[0] = x0
    trj_mean[1] = xT
    trj_cov[0] = Sig0
    trj_cov[1] = SigT

    ellip_pts = t_ellips_pts(samples_index, trj_mean, trj_cov, n_ellip_pts)

    for i in range(2):
        if (i==0):
            color = 'red'
        else:
            color = 'green'

        fig.add_trace(
            go.Scatter3d(
                x=samples_t[i, :],
                y=ellip_pts[i,0,:],
                z=ellip_pts[i,1,:],
                mode='lines',
                line={'width': 4, 'color': color}
            )
        )
    
    fig.update_layout(
        scene = dict(
            xaxis = dict(nticks=5, 
                         range=[0, tf],
                         backgroundcolor="rgba(200, 200, 230, 0.95)",
                         gridcolor="white",
                         showbackground=True,
                         zerolinecolor="white",),
            yaxis = dict(nticks=5,
                        range=ylims,
                        backgroundcolor="rgba(230, 200,230, 0.95)",
                        gridcolor="white",
                        showbackground=True,
                        zerolinecolor="white",),
            zaxis = dict(nticks=5,
                         range=zlims,
                        backgroundcolor="rgba(230, 230,200, 0.95)",
                        gridcolor="white",
                        showbackground=True,
                        zerolinecolor="white",),
            xaxis_title=r'TIME (s)',
            yaxis_title=xlabel,
            zaxis_title=ylabel)

        )

    # name = 'eye = (x:2, y:2, z:0.1)'
    camera = dict(
        eye=camera_eye
    )

    fig.update_layout(scene_camera=camera)
    # fig.update_scenes(xaxis_visible=False, yaxis_visible=False,zaxis_visible=False )
    fig.update_layout(showlegend=False)
    
    return fig
