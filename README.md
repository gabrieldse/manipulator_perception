# Commands

```bash
# launch camera node
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /root/dev_ws/src/manipulator_h_vision/config/params.yaml

# launch minimal image subscriber
ros2 run manipulator_h_vision track_circle

# to help visualize camera
/root/dev_ws/src/manipulator_h_vision/config/rviz_cam.rviz
```