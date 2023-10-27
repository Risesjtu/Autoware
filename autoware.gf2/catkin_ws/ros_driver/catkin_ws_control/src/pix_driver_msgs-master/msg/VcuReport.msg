Header header
# [] [0|1] 0-off 1-on
int8 brake_light_actual
# [] [0|0] 0-off 1-left 2-right 3-hazard_warning
int8 turn_light_actual
# [] [0|255]
int32 chassis_errcode
# [] [0|7] 0-throttle 1-speed
int8 drive_mode_sts
# [] [0|7] 0-standard 1-non_direction 2-sync_direction
int8 steer_mode_sts
# [] [0|0] 0-manual_remote 1-auto 2-emergency 3-standby
int8 vehicle_mode_state
# [] [0|0] 0-no_event 1-crash_event
int8 frontcrash_state
# [] [0|0] 0-no_event 1-crash_event
int8 backcrash_state
# [] [0|0] 0-inactive 1-active
int8 aeb_state
# [m/s^2] [-10|10]
float32 acc
# [m/s] [-32.768|32.767]
float32 speed