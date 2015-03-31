rostopic pub -1 /romeo/controllers_manager/line_tracker_distance/goal geometry_msgs/Vector3 "{x: 0.0, y: 0.0, z: 0}"

rosservice call /romeo/controllers_manager/transition line_tracker/LineTrackerDistance

