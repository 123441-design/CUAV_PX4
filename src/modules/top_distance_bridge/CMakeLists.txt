############################################################################
# Dedicated TELEM1 four-laser MAVLink receiver
############################################################################

px4_add_module(
	MODULE modules__top_distance_bridge
	MAIN top_distance_bridge
	COMPILE_FLAGS
		-Wno-address-of-packed-member
		-Wno-cast-align
	INCLUDES
		${CMAKE_BINARY_DIR}/mavlink
		${CMAKE_BINARY_DIR}/mavlink/${CONFIG_MAVLINK_DIALECT}
	SRCS
		TopDistanceBridge.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		mavlink_c_generate
		px4_work_queue
	)
