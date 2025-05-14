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

## Add random noise and the collision checking distances of the baseline trajectories
1. Run the pose listener for the sdf
```
python vimp/scripts/hardware_experiment/sdf_pose_lcmlistener.py
```
2. Generate random obstacles
```
python vimp/scripts/hardware_experiment/fake_bodyframe_publisher.py
```
3. Hit ENTER once in a while every time the sdf listener has saved the current results.
