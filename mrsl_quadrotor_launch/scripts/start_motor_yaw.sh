rostopic pub -1 controllers_manager/line_tracker_yaw/goal quadrotor_msgs/FlatOutputs  "{x: 0.0, y: 0.0, z: 0, yaw: 0}"

rosservice call controllers_manager/transition line_tracker/LineTrackerYaw

