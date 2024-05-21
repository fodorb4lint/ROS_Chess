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
# Kamera
1. A kameránkat úgy helyeztük el, hogy az 1. csukló alatti ```Gripper_base_link``` részhez rögzítettük és felülről látja be a táblánkat, emiatt nem volt szükség több, különböző kamerára. Első lépésként a ```chess_robot.xacro``` fájlba helyeztük be a camera_link -et:
```console
<!-- Camera -->
  <joint type="fixed" name="camera_joint">
    <origin xyz="1.3 0 1.5" rpy="0 0.2 -3.14"/>
    <child link="camera_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" />
  </joint>

  <link name='camera_link'>
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </collision>

    <visual name='camera_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </visual>

  </link>

  <gazebo reference="camera_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <joint type="fixed" name="camera_optical_joint">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <child link="camera_link_optical"/>
    <parent link="camera_link"/>
  </joint>

  <link name="camera_link_optical">
  </link>
```

Itt meghatároztuk, hogy a pozíciónk ``` <origin xyz="1.3 0 1.5" rpy="0 0.2 -3.14"/>``` legyen 
Ezt követően a chess_robot.gazebo -ba behelyezzük a kameránknak a plug-in-ját, amivel tudunk szimulálni:
```console
<!-- Camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <visualize>true</visualize>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.01</near>
          <far>25.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>head_camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link_optical</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
```
