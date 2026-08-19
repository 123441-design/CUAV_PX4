px4_add_module(
	MODULE modules__custom_action_control
	MAIN custom_action_control
	SRCS
		CustomActionControl.cpp
		CustomActionControl.hpp
	MODULE_CONFIG
		custom_action_control_params.yaml
	DEPENDS
		px4_work_queue
	)
