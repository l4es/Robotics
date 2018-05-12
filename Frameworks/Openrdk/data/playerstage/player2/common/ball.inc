define ball position
(
	name ["ball"]

	color "orange"

  # position properties
  drive "omni"
  pose [ 0 0 0 ]

	# actual size
  size [0.1 0.1]

  # the pioneer's center of rotation is offset from its center of area
  origin [0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 0
	gui_outline 0

  # estimated mass in KG
  mass 0.1

  polygons 1
	polygon[0].points 12
	polygon[0].point[0] [  0.050   0.000 ]
	polygon[0].point[1] [  0.043   0.025 ]
	polygon[0].point[2] [  0.025   0.043 ]
	polygon[0].point[3] [  0.000   0.050 ]
	polygon[0].point[4] [ -0.025   0.043 ]
	polygon[0].point[5] [ -0.043   0.025 ]
	polygon[0].point[6] [ -0.050   0.000 ]
	polygon[0].point[7] [ -0.043  -0.025 ]
	polygon[0].point[8] [ -0.025  -0.043 ]
	polygon[0].point[9] [  0.000  -0.050 ]
	polygon[0].point[10] [  0.025  -0.043 ]
	polygon[0].point[11] [  0.043  -0.025 ]
)



