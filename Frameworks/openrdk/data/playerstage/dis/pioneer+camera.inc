include "../common/pioneer.inc"
include "../common/sick.inc"
include "../common/urg.inc"
include "../common/camera.inc"

# create a robot
define erratic pioneer2dx
(
	color "red"
	#sick_laser()
	urg_laser()

	# needed by fakelocalize driver
	localization "gps"
	localization_origin [ 0 0 0 ]

	obstacle_return 1
	laser_return 1
	blob_return 1
)

define erraticcam erratic
(
	camera()
)

define erraticomnicam erratic
(
	cameraomni()
)
