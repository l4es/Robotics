define ball position
(
	name ["ball"]

	color "orange"

	# position properties
	drive "omni"
	pose [ 0 0 0 0 ]

	# actual size
	size [0.1 0.1 0.1]

	# the pioneer's center of rotation is offset from its center of area
	origin [0 0 0 0]

	# draw a nose on the robot so we can see which way it points
	gui_nose 0
	gui_outline 0

	# estimated mass in KG
	mass 0.1
)



