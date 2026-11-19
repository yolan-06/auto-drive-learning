Header Header

float32 cur_speed  # 当前速度(km/h)
float32 cur_steer # 当前转向角度(°)

float32 left_wheel_speed
float32 right_wheel_speed

bool brake          # 刹车状态
bool is_autodrive   # 自动驾驶按钮状态

uint8 wheel_direction # 车轮转向方向：左或者右
uint8 shift_level # 当前档位
uint8 drive_mode # 驾驶模式
