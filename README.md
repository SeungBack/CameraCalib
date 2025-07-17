


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


[INFO] [1752740153.047527967] [eih_calib]: matrix: 
[[ 6.06634282e-03 -9.99499605e-01 -3.10441363e-02 -2.20001083e-03]
 [ 1.75717111e-01  3.16271219e-02 -9.83932529e-01 -1.27009401e-01]
 [ 9.84422011e-01  5.13886088e-04  1.75821044e-01  1.14070881e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1752740153.048566440] [eih_calib]: pos: [-0.00220001 -0.1270094   0.11407088], quat (xyzw): [ 0.4468272  -0.46090664  0.53341531  0.55079817]