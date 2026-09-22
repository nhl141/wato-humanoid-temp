int8 motor_id

# position: degrees in the MOTOR frame (apply hardware_mapping.yaml's direction/zero_offset
# to get the command frame). velocity: servo mode -> ERPM, MIT mode -> the drive's velocity
# units (GL II: "r/s") scaled by mit_profiles.yaml.
float32 position
float32 velocity
float32 current

# MIT mode only (N.m, decoded from the drive's 12-bit torque field). 0 for servo-mode
# feedback, which reports current instead.
float32 torque

int8 temperature

# Servo mode: the DBC's FbkErrorCode (0=no fault ... 7=motor lock-up).
# MIT mode (GL II): the drive's status nibble -- 0=Disable, 1=Enable, 8=over-voltage,
# 9=under-voltage, 10=over-current, 11=MOSFET over-temp, 12=winding over-temp,
# 13=communication loss, 14=overload.
int8 error_code
