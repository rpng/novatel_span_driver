# message 1462

# IMU Status
# 0 = bad
# 1 = good!
uint8 imu_status

# Table 14 in the SPAN on OEM6 manual:
# http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf
# Northrop Grumman Litef ISA-100C = 26
uint8 imu_type

uint16 gps_week
float64 gps_week_seconds

uint32 imu_status_data

int32 accel_z
int32 accel_neg_y
int32 accel_x

int32 gyro_z
int32 gyro_neg_y
int32 gyro_x

