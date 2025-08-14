


```
pip install 'opencv-python<4.7.0' opencv-contrib-python 'numpy<2'
```

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.8 


ros2 launch orbbec_camera femto_bolt.launch.py \
    color_width:=1920 color_height:=1080 color_fps:=15 \
    depth_width:=1024 depth_height:=1024 depth_fps:=15 \
    ir_width:=1024 ir_height:=1024 ir_fps:=15 \
    depth_registration:=true 

```

[INFO] [1755151648.543721086] [eih_calib]: Computing calibration result
[INFO] [1755151648.545920088] [eih_calib]: Computed results tool0_to_camera_color_optical_frame
[INFO] [1755151648.547069483] [eih_calib]: matrix: 
[[ 9.99924106e-01  1.21668125e-02  1.93660639e-03  2.64398014e-02]
 [-1.23121119e-02  9.81247774e-01  1.92357004e-01 -1.29796681e-01]
 [ 4.40080902e-04 -1.92366250e-01  9.81323103e-01  1.26690008e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1755151648.548013782] [eih_calib]: pos: [ 0.0264398  -0.12979668  0.12669001], quat (xyzw): [-9.66349172e-02  3.75897778e-04 -6.14862453e-03  9.95300832e-01]
[INFO] [1755151648.549671694] [eih_calib]: Computed results tool0_to_camera_link
[INFO] [1755151648.550952128] [eih_calib]: matrix: 
[[ 1.93660639e-03 -9.99924106e-01 -1.21668125e-02 -6.06275518e-03]
 [ 1.92357004e-01  1.23121119e-02 -9.81247774e-01 -1.29986142e-01]
 [ 9.81323103e-01 -4.40080902e-04  1.92366250e-01  1.29006459e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]