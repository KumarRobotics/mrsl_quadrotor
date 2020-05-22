rostopic pub -1 controllers_manager/line_tracker_waypoints/goal kr_mav_msgs/FlatOutputs  "{x: 0.0, y: 0.0, z: 0, yaw: 0}"

rosservice call controllers_manager/transition line_tracker/LineTrackerWaypoints

#rostopic pub -1 controllers_manager/line_tracker_waypints/waypoints nav_msgs/Path  "poses: {{pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, {pose: {position: {x: 2.0, y: 0.0, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"
