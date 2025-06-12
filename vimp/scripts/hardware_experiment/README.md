## Generate baseline trajectories
1. run the ros simulator and RViz 
```
cd ~/ws_moveit/
source devel/setup.bash
roslaunch panda_moveit_config demo.launch
```
2. Generate obstacles for RViz: in two separate terminals, run:
```
python vimp/scripts/hardware_experiment/planningscene_pose_listener.py
python vimp/scripts/hardware_experiment/fake_bodyframe_publisher.py
```
3. Hit ENTER key to send an obstacle's pose, observe it in RViz
4. Run the baselines:
```
python vimp/scripts/hardware_experiment/moveit_baselines.py
``` 

The result baseline plans will be saved to corresponding yaml in the Data/ folder with the planner id in the name. e.g., 'RRT_plan_trj.yaml'. To read the planned trajectory and display in RViz, use function 'read_plan_json_to_numpy' defined in moveit_baselines.py

## Add artificial random noise and compute collision‐checking distances
1. Run the pose listener for the sdf
```
python vimp/scripts/hardware_experiment/sdf_pose_lcmlistener.py
```
2. Generate noisy obstacles based on the pose in config file
```
python vimp/scripts/hardware_experiment/fake_bodyframe_publisher.py
```
3. SDF with random obstacles and the signed distance checking of baseline trajectories in the SDF will be generated automatically. The random poses will be saved in Data/poses.yaml, and the signed distances will be saved in `Data/disturbed_results.yaml`


## Real world noisy poses and signed distance checking
1. Compute the noimial pose of the obstacle. Put the box in the workspace at still, and run 
```
python vimp/scripts/hardware_experiment/apriltag_d435.py
```
after running the script, hit ENTER key for multiple (~10) times, the poses will be saved in `Data/box_poses.yaml`. After that, run the following script to compute the mean of the default (nominal) pose:
```
python vimp/scripts/hardware_experiment/pose_helpers.py
``` 
The script will output the nominal pose. Use this value to update the pose in `config/config.yaml`.

2. Do planning using the default pose for both GVIMP and for baselines
```
python vimp/scripts/hardware_experiment/moveit_baselines.py
``` 
3. Add noise by moving the obstacles in the workspace, and record the noisy poses:
```
python vimp/scripts/hardware_experiment/apriltag_d435.py
```
Note: Before running, modify the output filename in `apriltag_d435.py` to avoid overwriting `Data/box_poses.yaml`.

4. Record the noisy pose and compute the collision‐checking distances of baselines, GVIMP, and resampled GVIMP
```
python vimp/scripts/hardware_experiment/sdf_pose_lcmlistener.py
python vimp/scripts/hardware_experiment/hardware_exp_pose_publisher.py
```

5. Visualize the statistical results:
```
python vimp/scripts/hardware_experiment/plot_results.py
```