# obstacle_server_cpp
Obstacle server is a node fusing data about obstacles from various sources. The idea is to provide reliable and aggregated data to planners.

This implementation is split into serveral threads using timers:

- serverPublish() - periodically sending obstacle data to other nodes,
- transformListener() - periodically listening to /tf for transform map->laser.

## Parameters

- `/delay_measure` -- when True, timing evaluation is performed.
- `~frame_id` -- name of the top frame, which we take reference from.
- `~child_frame_id` -- name of the child frame, containing the position of the car.


## Example

### ROS1

```sh
rosparam set /delay_measure true
rosrun obstacle_server_cpp obstacle_server_cpp_node _frame_id:=/map _child_frame_id:=/base_link
```

### ROS2

```sh
ros2 run obstacle_server_cpp obstacle_server_cpp_node --ros-args -p delay_measure:=true -p frame_id:=/map -p child_frame_id:=base_link
```
