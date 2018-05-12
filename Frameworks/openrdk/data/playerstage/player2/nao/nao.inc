include "../common/finder.inc"
include "../common/pioneer.inc"
include "../common/urg.inc"
include "../common/camera.inc"

# create a position model - it can drive around like a robot
define nao pioneer2dx
(
	color "green"
	size [ 0.2 0.4 ]
	origin [ 0 0 0 ]
	gui_nose 1
	mass 5

	bitmap "nao.png"
	name "nao"


	drive "diff"
	pose [ 0 0 0]
	localization "gps"
	localization_origin [ 0 0 0 ]

	urg_laser(
		blob_return 0
		gui_outline 0
		polygons 0
		size [0.01 0.01 ]
	)
	camera()

	featurefinder
	(
		name "cornersdetector"
		fiducial_key 1
		color "green"
	)

	obstacle_return 1
	blob_return 1
	laser_return 1
	gui_outline 0
	polygons 0
)


