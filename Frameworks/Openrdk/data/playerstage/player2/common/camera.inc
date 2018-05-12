
define commonblobfinder blobfinder
(
	name "camera"
	channel_count 12
	channels [ "blue" "red" "gray" "violet" "pink" "yellow" "gold" "lime green" "lawn green" "magenta" "orange" "cyan"]
	range_max 6.0
	
	# model properties
	size [0.005 0.005]

	# camera resolution
	image [320 240]
	color "green"
	blob_return 0
	laser_return 0
	obstacle_return 0
)

define commoncamera ptz
(
	name "ptz"
	size [0.025 0.025]
	blob_return 0
	laser_return 0
	obstacle_return 0
)

define camera commoncamera
(
	ptz [ 0 0 60 ]
	name "ptz"
	commonblobfinder
	(
		name "imageprocessor"
		range_max 6.0
	)
)

define cameraomni commoncamera
(
	ptz [ 0 0 360 ]
	name ""
	commonblobfinder
	(
		name "imageprocessor"
		range_max 4.0
	)
)

define fixedcamera commoncamera
(
	ptz [0 0 70]
	commonblobfinder
	(
		name "imageprocessor"
	)
	size [0.2 0.1]
)

define fixedwidecamera commoncamera
(
	ptz [0 0 90]
	commonblobfinder
	(
		name "imageprocessor"
	)
	size [0.2 0.1]
)
