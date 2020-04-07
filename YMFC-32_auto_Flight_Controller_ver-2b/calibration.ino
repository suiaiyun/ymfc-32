/**
 * 校准罗盘
 */
void calibrate_compass(void) {
  /* 设置compass_calibration_on变量以禁用原始罗盘值的调整 */
  compass_calibration_on = 1;  
  /* 红色led将指示指南针校准已激活 */                                              
  red_led(HIGH);
  /* 熄灭绿色LED灯 */                                                             
  green_led(LOW);
  /* 保持在这个循环中，直到射器的俯仰杆被拉低 */
  while (channel_2 < 1900) {
    /* 向地面站发送遥测数据 */
    send_telemetry_data();
    /* 模拟一个250Hz的程序循环 */
    delayMicroseconds(3700);
    /* 读取原始罗盘值 */
    read_compass();
    /* 检测并存储最大和最小罗盘值 */
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
  }
  /* 重置指南针校准变量 */
  compass_calibration_on = 0;

  /* 存储电子罗盘校准值 */
  for (error = 0; error < 6; error ++) {
    EEPROM.write(0x10 + error, compass_cal_values[error]);
  }

  /* 初始化指南针并设置正确的寄存器 */
  setup_compass();
  /* 读取并计算罗盘数据 */
  read_compass();
  /* 设置初始罗盘航向 */
  angle_yaw = actual_compass_heading;

  red_led(LOW);
  for (error = 0; error < 15; error ++) {
    green_led(HIGH);
    delay(50);
    green_led(LOW);
    delay(50);
  }

  error = 0;
  /* 设置下一个循环的计时器 */
  loop_timer = micros();
}

/**
 * 校准水平仪
 */
void calibrate_level(void) {
  level_calibration_on = 1;

  while (channel_2 < 1100) {
    /* 向地面站发送遥测数据 */
    send_telemetry_data();
    delay(10);
  }
  red_led(HIGH);
  green_led(LOW);

  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  for (error = 0; error < 64; error ++) {
    /* 向地面站发送遥测数据 */
    send_telemetry_data();
    gyro_signalen();
    acc_pitch_cal_value += acc_y;
    acc_roll_cal_value += acc_x;
    if (acc_y > 500 || acc_y < -500)error = 80;
    if (acc_x > 500 || acc_x < -500)error = 80;
    delayMicroseconds(3700);
  }

  acc_pitch_cal_value /= 64;
  acc_roll_cal_value /= 64;

  red_led(LOW);
  if (error < 80) {
    EEPROM.write(0x16, acc_pitch_cal_value);
    EEPROM.write(0x17, acc_roll_cal_value);
    for (error = 0; error < 15; error ++) {
      green_led(HIGH);
      delay(50);
      green_led(LOW);
      delay(50);
    }
    error = 0;
  }
  else error = 3;
  level_calibration_on = 0;
  gyro_signalen();
  /** 
   *  加速度计角度计算
   *  计算加速度计总矢量
   */
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); 

  /**
   * 防止asin函数产生NaN
   * 计算俯仰角
   */
  if (abs(acc_y) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  /**
   * 防止asin函数产生NaN
   * 计算横滚角
   */
  if (abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }
  /* 将陀螺仪俯仰角设置为四旋翼机启动时的加速度计俯仰角 */
  angle_pitch = angle_pitch_acc;
  angle_roll = angle_roll_acc;
  /* 设置下一个循环的计时器 */
  loop_timer = micros();
}
