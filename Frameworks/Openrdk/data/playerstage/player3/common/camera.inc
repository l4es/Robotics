
define commoncamera blobfinder
(
	name "camera"
	colors_count 12
	colors [ "blue" "red" "gray" "violet" "pink" "yellow" "gold" "lime green" "lawn green" "magenta" "orange" "cyan"]
	range 6.0
	
	# model properties
	size [0 0 0]

	# camera resolution
	image [80 60]
	color "green"
	fov 1.570795 # 3.14159/3.0
	pan 0.0
)

define camera commoncamera
(
	name "imageprocessor"
	range 6.0
)

define cameraomni commoncamera
(
	name "imageprocessor"
	range 6.0
	fov 6.28318 # 3.14159/2.0
)

define fixedcamera commoncamera
(
	name "imageprocessor"
)

define fixedwidecamera commoncamera
(
	name "imageprocessor"
	range 21.0
)
