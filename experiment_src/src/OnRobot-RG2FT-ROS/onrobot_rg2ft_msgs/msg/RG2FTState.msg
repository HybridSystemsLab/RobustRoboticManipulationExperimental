uint8 status_l # Reads low (0x0000) when there is no error with the left finger sensor.
int16 fx_l # Left finger sensor's force value along the X axis (in the sensor coordinate system) in 1/10N.
int16 fy_l # Left finger sensor's force value along the Y axis (in the sensor coordinate system) in 1/10N.
int16 fz_l # Left finger sensor's force value along the Z axis (in the sensor coordinate system) in 1/10N.
int16 tx_l # Left finger sensor's torque value about the X axis (in the sensor coordinate system) in 1/100 Nm.
int16 ty_l # Left finger sensor's torque value about the Y axis (in the sensor coordinate system) in 1/100 Nm.
int16 tz_l # Left finger sensor's torque value about the Z axis (in the sensor coordinate system) in 1/100 Nm.
uint8 status_r # Same as the left above.
int16 fx_r # Same as the left above.
int16 fy_r # Same as the left above.
int16 fz_r # Same as the left above.
int16 tx_r # Same as the left above.
int16 ty_r # Same as the left above.
int16 tz_r # Same as the left above.
uint8 proximity_status_l # Reads low (0x0000) when there is no error with the left proximity sensor.
int16 proximity_value_l # Reads the current distance from the left proximity sensor in 1/10 mm.
uint8 proximity_status_r # Same as the left above.
int16 proximity_value_r # Same as the left above.
int16 actual_gripper_width # Indicates the current width between the gripper fingers in 1/10 millimeters.
uint8 gripper_busy # High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
uint8 grip_detected # High (1) when an internal or external grip is detected.