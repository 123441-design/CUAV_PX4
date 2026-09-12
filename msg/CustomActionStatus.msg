# State and control ownership for project CUSTOM actions.

uint64 timestamp

uint8 STATE_INACTIVE = 0
uint8 STATE_SEARCH_TOP = 1
uint8 STATE_TOP_APPROACH = 2
uint8 STATE_CONTACT_VERIFY = 3
uint8 STATE_CONTACT_PRESS = 4
uint8 STATE_CONTACT_PRESS_WAIT = 5
uint8 STATE_MOTOR_TRIM_WAIT = 6

uint8 OWNER_LEGACY = 0
uint8 OWNER_CUSTOM = 1
uint8 OWNER_HANDOVER = 2
uint8 OWNER_COMMANDER = 3

uint8 ACTION_NONE = 0
uint8 ACTION_SEARCH_TOP = 1

uint8 REASON_NONE = 0
uint8 REASON_DIRECTION = 1
uint8 REASON_MAX_DISTANCE = 2
uint8 REASON_TIMEOUT = 3
uint8 REASON_SENSOR_TIMEOUT = 4
uint8 REASON_ESTIMATOR = 5
uint8 REASON_HEADING_RESET = 6
uint8 REASON_LAND = 7
uint8 REASON_TOP_DISTANCE = 8
uint8 REASON_ACTUATOR_TIMEOUT = 9
uint8 REASON_MOTOR_TRIM_MISMATCH = 10

uint8 TRIM_SOURCE_NONE = 0
uint8 TRIM_SOURCE_SEARCH_LOOSE = 1
uint8 TRIM_SOURCE_SEARCH_STRICT = 2
uint8 TRIM_SOURCE_APPROACH_STRICT = 3

uint8 TRIM_CANDIDATE_SEARCH_LOOSE = 1
uint8 TRIM_CANDIDATE_SEARCH_STRICT = 2
uint8 TRIM_CANDIDATE_APPROACH_STRICT = 4

uint8 state
bool active
uint8 action
uint8 control_owner
uint16 handover_id
uint8 reason

# Contact-pressure telemetry. Gains are normalized ratios: 0.30 means +30%.
float32 pressure_target_gain
float32 pressure_applied_gain
float32 pressure_progress
float32 pressure_time_s
uint8 trim_source
uint8 trim_candidate_mask
uint8 limiting_motor
bool pressure_limited
bool pressure_ramp_complete
