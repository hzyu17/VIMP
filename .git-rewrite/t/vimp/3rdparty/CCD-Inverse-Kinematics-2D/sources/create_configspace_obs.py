import numpy as np 
import matplotlib.pyplot as plt 
import math

# Robot Link Length Parameter
link = [0.5, 0.5]
# Robot Initial Joint Values (degree)
angle = [0, 0]
# Target End of Effector Position
target = [0, 0, 0] 

# Create figure to plot
fig = plt.figure() 
ax = fig.add_subplot(1,1,1)

# Draw Axis
def draw_axis(ax, scale=1.0, A=np.eye(4), style='-', draw_2d = False):
    xaxis = np.array([[0, 0, 0, 1], 
                      [scale, 0, 0, 1]]).T
    yaxis = np.array([[0, 0, 0, 1], 
                      [0, scale, 0, 1]]).T
    zaxis = np.array([[0, 0, 0, 1], 
                      [0, 0, scale, 1]]).T
    
    xc = A.dot( xaxis )
    yc = A.dot( yaxis )
    zc = A.dot( zaxis )
    
    if draw_2d:
        ax.plot(xc[0,:], xc[1,:], 'r' + style)
        ax.plot(yc[0,:], yc[1,:], 'g' + style)
    else:
        ax.plot(xc[0,:], xc[1,:], xc[2,:], 'r' + style)
        ax.plot(yc[0,:], yc[1,:], yc[2,:], 'g' + style)
        ax.plot(zc[0,:], zc[1,:], zc[2,:], 'b' + style)

def rotateZ(theta):
    rz = np.array([[math.cos(theta), - math.sin(theta), 0, 0],
                   [math.sin(theta), math.cos(theta), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    return rz

def translate(dx, dy, dz):
    t = np.array([[1, 0, 0, dx],
                  [0, 1, 0, dy],
                  [0, 0, 1, dz],
                  [0, 0, 0, 1]])
    return t

# Forward Kinematics
# Input initial angles and length of links
# Output positions each points
def FK(angle, link):
    n_links = len(link)
    P = []
    P.append(np.eye(4))
    for i in range(0, n_links):
        R = rotateZ(angle[i]/180*math.pi)
        T = translate(link[i], 0, 0)
        P.append(P[-1].dot(R).dot(T))
    return P

def IK(target, angle, link, max_iter = 10000, err_min = 0.01):
    solved = False
    err_end_to_target = math.inf
    
    for loop in range(max_iter):
        for i in range(len(link)-1, -1, -1):
            P = FK(angle, link)
            end_to_target = target - P[-1][:3, 3]
            err_end_to_target = math.sqrt(end_to_target[0] ** 2 + end_to_target[1] ** 2)
            if err_end_to_target < err_min:
                solved = True
            else:
                # Calculate distance between i-joint position to end effector position
                # P[i] is position of current joint
                # P[-1] is position of end effector
                cur_to_end = P[-1][:3, 3] - P[i][:3, 3]
                cur_to_end_mag = math.sqrt(cur_to_end[0] ** 2 + cur_to_end[1] ** 2)
                cur_to_target = target - P[i][:3, 3]
                cur_to_target_mag = math.sqrt(cur_to_target[0] ** 2 + cur_to_target[1] ** 2)

                end_target_mag = cur_to_end_mag * cur_to_target_mag

                if end_target_mag <= 0.0001:    
                    cos_rot_ang = 1
                    sin_rot_ang = 0
                else:
                    cos_rot_ang = (cur_to_end[0] * cur_to_target[0] + cur_to_end[1] * cur_to_target[1]) / end_target_mag
                    sin_rot_ang = (cur_to_end[0] * cur_to_target[1] - cur_to_end[1] * cur_to_target[0]) / end_target_mag

                rot_ang = math.acos(max(-1, min(1,cos_rot_ang)))

                # if sin_rot_ang > 0.0:
                #     rot_ang = -rot_ang

                # Update current joint angle values
                angle[i] = angle[i] + (rot_ang * 180 / math.pi)
                
                if angle[i] <= -360:
                    angle[i] = angle[i] + 360
                if angle[i] > 0:
                    angle[i] = angle[i] - 360

                # if angle[i] >= 360:
                #     angle[i] = angle[i] - 360
                # if angle[i] < 0:
                #     angle[i] = 360 + angle[i]
                  
        if solved:
            break
            
    return angle, err_end_to_target, solved, loop

import pandas as pd
import os
full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
root_vimp = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(full_path))))
map_file = root_vimp+"/maps/2dArm/map.csv"


def read_map(file_name=map_file):
    global target, link, angle, ax
    
    df = pd.read_csv(file_name, header=None)
    cell_map = df.values

    rows, cols = cell_map.shape
    
    cell_size = 0.01
    origin_x = -1
    origin_y = -1
    
    # find the obstacle elements
    ones = np.where(cell_map == 1)
    
    grid_corner_x = origin_x + (cols-1)*cell_size;
    grid_corner_y = origin_y + (rows-1)*cell_size;
    
    grid_X = np.linspace(origin_x, grid_corner_x, (cols-1));
    grid_Y = np.linspace(origin_y, grid_corner_y, (rows-1));
    
    y_obs = grid_Y[ones[0]]
    x_obs = grid_X[ones[1]]        
    
    cell_config = np.linspace(-np.pi, np.pi, 300)
    map_config = np.zeros([300, 300])
    
    for x_coord, y_coord in zip(x_obs, y_obs):
        target[0] = x_coord
        target[1] = y_coord
        angle, err, solved, iteration = IK(target, angle, link, max_iter=1000)

        angle[0] = angle[0]*np.pi/180.0
        angle[1] = angle[1]*np.pi/180.0
        
        # if angle[0] < -2*np.pi:
        #     angle[0] = angle[0] + 2*np.pi
        
        # if angle[1] < -2*np.pi:
        #     angle[1] = angle[1] + 2*np.pi
            
        # if angle[0] < -np.pi:
        #     angle[0] = angle[0] + 2*np.pi
            
        # if angle[1] < -np.pi:
        #     angle[1] = angle[1] + 2*np.pi  
        
        # if angle[0] > 2*np.pi:
        #     angle[0] = angle[0] - 2*np.pi
        
        # if angle[1] > 2*np.pi:
        #     angle[1] = angle[1] - 2*np.pi
            
        # if angle[0] > np.pi:
        #     angle[0] = angle[0] - 2*np.pi
            
        # if angle[1] > np.pi:
        #     angle[1] = angle[1] - 2*np.pi  
        
        indx_x = next(x[0] for x in enumerate(cell_config) if x[1] > angle[0])
        indx_y = next(x[0] for x in enumerate(cell_config) if x[1] > angle[1])
                
        map_config[indx_y, indx_x] = 1
        
        if solved:
            print("\nIK solved\n")
            print("Iteration :", iteration)
            print("Angle :", angle)
    
    # save configuration space obstacles into csv file
    np.savetxt(root_vimp+"/maps/2dArm/config_obs.csv", map_config, delimiter=",")
    
    
def onclick():
    return 
    
    
def main():
    
    ## =====================================================================
    # Read a sdf map and use the IK to map it into the configuration space.
    ## =====================================================================
    
    read_map(map_file)
    
    ## ===========================================================================
    # Show an example configuration space plan in the work space to verify the FK.
    ## ===========================================================================
    
    # fig.canvas.mpl_connect('button_press_event', onclick)
    # fig.suptitle("Cyclic Coordinate Descent - Inverse Kinematics", fontsize=12)

    # print("Target Position : ", target)
    # plt.cla()
    # ax.set_xlim(-1, 1.5)
    # ax.set_ylim(-1, 1.5)
    
    # df = pd.read_csv(root_vimp+"/../matlab_helpers/PGCS-examples/2d_Arm/map1/case1/zk_sdf.csv", header=None)
    # trj = df.values
    # nt = trj.shape[1]
    
    # for i_t in range(nt):
    #     angle[0] = trj[0, i_t] * 180.0 / np.pi
    #     angle[1] = trj[1, i_t] * 180.0 / np.pi
    #     P = FK(angle, link)
    #     for i in range(len(link)):
    #         start_point = P[i]
    #         end_point = P[i+1]
    #         ax.plot([start_point[0,3], end_point[0,3]], [start_point[1,3], end_point[1,3]], linewidth=5)

    # plt.show()
    
    

    return
    
if __name__ == "__main__":
    main()
