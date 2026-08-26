# State and control ownership for project CUSTOM actions.

uint64 timestamp

uint8 STATE_INACTIVE = 0
uint8 STATE_SEARCH_TOP = 1
uint8 STATE_TOP_APPROACH = 2
uint8 STATE_CONTACT_VERIFY = 3
uint8 STATE_CONTACT_PRESS = 4

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

uint8 state
bool active
uint8 action
uint8 control_owner
uint16 handover_id
uint8 reason
