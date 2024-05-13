# ROS_Chess
# Project állapot
- Környezet
- Robot szimuláció 
- Robot kézivezérlő
  

# Futtatás
1. ROS_Chess git projekten belül carkin_ws megnyitása terminálon belül ezt érdemes minden új terminál ablaknál megtenni. Git alapértelmezett beállítás esetén: 
```console
cd ~/Documents/GitHub/ROS_Chess/catkin_ws
```
2. Minden terminál ablakon belül be kell tölteni a projectet (ha valami nem működik akkor valószínűleg ennek a lépésnek a hiánya okozza):
```console
source devel/setup.bash
```
3. Környezet és vezérlő indítása
3.1. Robot szimuláció, Gazebo környezet, Rosmaster indítása és vezérlő paramétereinek előkészítése 
```console
roslaunch ros_chess spawn_robot.launch
```
3.2. Robot vezérlő indítása 
```console
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
4. Fontosabb egyéb program:
4.1. PID hangoláshoz:
```console
rosrun rqt_reconfigure rqt_reconfigure
```
4.2. rqt minden máshoz (rosmaster-nek futnia kell):
```console
rqt
```


