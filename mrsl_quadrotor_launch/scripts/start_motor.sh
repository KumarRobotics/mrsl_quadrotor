rostopic pub -1 controllers_manager/line_tracker_min_jerk/goal geometry_msgs/Vector3 "{x: 0.0, y: 0.0, z: 0}"

rosservice call controllers_manager/transition line_tracker/LineTrackerMinJerk

