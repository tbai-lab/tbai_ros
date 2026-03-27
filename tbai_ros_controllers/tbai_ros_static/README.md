# tbai_ros_static package

## Example
```bash
# Start ROS and relevant nodes
roslaunch tbai_ros_static simple.launch

# Change controllers
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'SIT'"
rostopic pub /anymal_d/change_controller std_msgs/String "data: 'STAND'"
```

https://github.com/lnotspotl/tbai/assets/82883398/a31bd182-6b55-44ca-8e13-18466e9ce282

