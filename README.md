# pick-place-bt
Using behaviortree cpp to pick and place object

## Demo


https://github.com/user-attachments/assets/6d7853a4-4656-401a-aa3b-2115b5c069af



## Commands

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm

ros2 run pick_place_bt gripper_bt_node

ros2 launch pick_place_bt move_group_server.launch.py
```
