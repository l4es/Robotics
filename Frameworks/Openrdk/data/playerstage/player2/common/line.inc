define line model
(
	name ["line"]
	pose [0 0 0]
	size [0.05 0.05]
	origin [0 0 0]
	velocity [0 0 0]

	color "gray"

	# determine how the model appears in various sensors

	obstacle_return 0
	laser_return 0
	ranger_return 0
	blob_return 0
	fiducial_return 1
	gripper_return 0

	fiducial_key 0

	# GUI properties
	gui_nose 0
	gui_grid 0
	gui_movemask 0
	gui_outline 0
)

define corner model
(
	name ["landmark"]
	size [ 0.05 0.05 ]
	origin [0 0 0]
	velocity [0 0 0]

	color "magenta"

	# determine how the model appears in various sensors

	obstacle_return 0
	laser_return 0
	ranger_return 0
	blob_return 1
	fiducial_return 1
	gripper_return 0

	fiducial_key 1

	# GUI properties
	gui_nose 0
	gui_grid 0
	gui_movemask 1
	gui_outline 0
)
