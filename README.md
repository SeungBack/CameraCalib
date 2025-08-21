


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

INFO] [1755744815.981136530] [eih_calib]: Computed results tool0_to_camera_color_optical_frame

[INFO] [1755744815.983487638] [eih_calib]: matrix: 
[[ 0.99980927  0.01814379 -0.00722695  0.03037563]
 [-0.01629535  0.97896299  0.20338615 -0.11792636]
 [ 0.01076511 -0.20322959  0.97907193  0.12387442]
 [ 0.          0.          0.          1.        ]]
 
[INFO] [1755744815.984757808] [eih_calib]: pos: [ 0.03037563 -0.11792636  0.12387442], quat (xyzw): [-0.10219387 -0.00452191 -0.00865552  0.99471657]