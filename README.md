# Commands

```bash
# launch camera node
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /root/dev_ws/src/manipulator_h_vision/config/params.yaml

# launch minimal image subscriber
ros2 run manipulator_h_vision track_circle

# to help visualize camera
rviz2 -d /root/dev_ws/src/manipulator_h_vision/config/rviz_cam.rviz
```


The camera launch gets first the params.yaml file and from it the camera_info.yaml file that it uses to calibrate itself.

references to the [usb_cam](https://github.com/ros-drivers/usb_cam) package