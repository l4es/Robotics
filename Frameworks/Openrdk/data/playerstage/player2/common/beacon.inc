define beacon model
(
	name ["landmark"]
	pose [0 0 0]
	size [0.1 0.1]
	origin [0 0 0]
	velocity [0 0 0]

	# determine how the model appears in various sensors
	
	obstacle_return 0
	laser_return 1
	ranger_return 1
	blob_return 1
	fiducial_return 0
	gripper_return 0

	fiducial_key 0

	# GUI properties
	gui_nose 0
	gui_grid 0
	gui_movemask 1
	gui_outline 0

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
