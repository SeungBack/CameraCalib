


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

[INFO] [1755675668.351463076] [eih_calib]: Computing calibration result
[INFO] [1755675668.353670342] [eih_calib]: Computed results tool0_to_camera_color_optical_frame
[INFO] [1755675668.354235341] [eih_calib]: matrix: 
[[ 9.99898236e-01  1.42572907e-02  4.96610934e-04  2.61380646e-02]
 [-1.40455928e-02  9.77763021e-01  2.09241954e-01 -1.22725987e-01]
 [ 2.49765555e-03 -2.09227636e-01  9.77863773e-01  1.29264840e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1755675668.354623264] [eih_calib]: pos: [ 0.02613806 -0.12272599  0.12926484], quat (xyzw): [-1.05203900e-01 -5.03065701e-04 -7.11538852e-03  9.94425089e-01]
[INFO] [1755675668.355451405] [eih_calib]: Computed results tool0_to_camera_link
[INFO] [1755675668.355895765] [eih_calib]: matrix: 
[[ 4.96610934e-04 -9.99898236e-01 -1.42572907e-02 -6.36892838e-03]
 [ 2.09241954e-01  1.40455928e-02 -9.77763021e-01 -1.22818832e-01]
 [ 9.77863773e-01 -2.49765555e-03  2.09227636e-01  1.31524222e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[INFO] [1755675668.356388547] [eih_calib]: pos: [-0.00636893 -0.12281883  0.13152422], quat (xyzw): [ 0.44080137 -0.44841982  0.54650833  0.55312066]
[INFO] [1755675668.377393706] [eih_calib]: Publishing the hand-eye calibration result as a static TF (tool0 -> camera_link)