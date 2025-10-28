# pick-place-bt
Using behaviortree cpp to pick and place object


## Commands

```bash
# for robot
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=cobotta sim:=false ip_address:=172.16.6.103 send_format:=0 recv_format:=2

ros2 launch bcap_service bcap_service.launch.py model:=cobotta ip_address:=172.16.6.103

ros2 launch denso_robot_bringup denso_gripper_bringup.launch.py

ros2 launch pick_place_bt move_group_server.launch.py


# for camera and apriltag pose detection
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_tf:=false publish_map_tf:=false enable_gnss:=false

ros2 launch pick_place_bt camera_arm_static_tf.launch.py

ros2 launch pick_place_bt april_tag_detector.launch.yaml

ros2 launch pick_place_bt tag_pose_server.launch.py

# for behaviortree
ros2 run pick_place_bt gripper_bt_node

```
