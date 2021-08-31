define kinect ranger
(
  sensor
  (
    range_max [ 0.2 5.0 ]
    fov 360
    samples 360
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]

)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  size [ 0.2 0.2 0.2 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  gripper_return 1
  obstacle_return 1
  
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  #camera_sensor(pose [ -0.1 0.0 -0.11 1.0 ])
)

