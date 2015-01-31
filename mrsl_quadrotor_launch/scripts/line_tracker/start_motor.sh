rostopic pub -1 /KHexTango/controllers_manager/line_tracker_distance/goal geometry_msgs/Vector3 "{x: 0.0, y: 0.0, z: 0}"

rosservice call /KHexTango/controllers_manager/transition line_tracker/LineTrackerDistance

