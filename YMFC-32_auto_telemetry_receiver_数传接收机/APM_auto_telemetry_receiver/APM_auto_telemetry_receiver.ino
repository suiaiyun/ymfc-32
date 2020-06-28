#include "mavlink/mavlink.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>

// 定义蜂鸣器引脚
#define BUZZER_PIN  2  
#define ANALOG_PIN  0

uint8_t receive_buffer[50], receive_buffer_counter, receive_byte_previous, receive_start_detect;
uint8_t check_byte, temp_byte;
uint8_t error, alarm_sound, flight_mode;
uint8_t telemetry_lost;
uint8_t alarm_silence;
uint8_t start, flight_timer_start;
uint8_t hours,minutes, seconds;
uint8_t heading_lock;
uint8_t number_used_sats;
uint8_t fix_type, max_speed_from_eeprom, speed_kmph, max_speed, speed_loop_counter;
uint16_t speed_buffer[5];

int8_t page, previous_page;

uint32_t last_receive, next_sound, flight_timer, flight_timer_previous, flight_timer_from_start, flight_time_from_eeprom;
uint32_t hours_flight_time, minutes_flight_time, seconds_flight_time;
int32_t l_lat_gps, l_lon_gps, l_lat_gps_previous, l_lon_gps_previous;
float lat_distance, lon_distance;

int16_t temperature, button_push, button_store, roll_angle, pitch_angle;
int16_t altitude_meters, max_altitude_meters, max_altitude_from_eeprom;
uint16_t key_press_timer;
int16_t takeoff_throttle;
uint16_t actual_compass_heading;

float battery_voltage, adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

byte led;


//SoftwareSerial TelemetrySerial(11, 12); // RX, TX

// 初始化lcd引脚
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

/**
 * 中断程序
 */
ISR(TIMER2_COMPA_vect) {
  if(button_store == -1)button_store = analogRead(ANALOG_PIN);
  if(button_store > 1000)button_store = -1;
}

void setup() {
  // 设置一个中断来检查模拟输入A0上的键输入
  TCCR2A = 0;                                                               // 设置TCCR2A寄存器为零
  TCCR2B = 0;                                                               // 设置TCCR2B寄存器为零
  TIMSK2 |= (1 << OCIE2A);                                                  // 在TIMSK2寄存器中设置中断启用位OCIE2A
  TCCR2B |= (1 << CS22|1 << CS21|1 << CS20);                                // 将TCCRB寄存器中的CS20、CS21和CS22位设置为1024
  OCR2A = 249;                                                              // 比较寄存器设置为249=>16ms/（1s/（16.000.000MHz/1024））-1=249
  TCCR2A |= (1 << WGM21);                                                   // 将计数器2设置为CTC（比较时清除计时器）模式

  Serial.begin(57600);                                                       //Set the serial output to 9600bps.
  pinMode(BUZZER_PIN, OUTPUT);                                              //Set input 2 as output for the buzzer.

  // 配置LCD屏幕宽度
  lcd.begin(16, 2);
  // 打印欢迎信息
  lcd.setCursor(4, 0);
  lcd.print("APM v2.8");
  lcd.setCursor(3, 1);
  lcd.print("telemetry");
  // 蜂鸣器提示音
  digitalWrite(BUZZER_PIN, HIGH);
  delay(10);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(10);
  digitalWrite(BUZZER_PIN, LOW);
  delay(2500);

  button_store = -1;
  lcd.clear();
  telemetry_lost = 2;
  flight_time_from_eeprom = (uint32_t)EEPROM.read(0x00)<< 24 | (uint32_t)EEPROM.read(0x01)<< 16 | (uint32_t)EEPROM.read(0x02)<< 8 | (uint32_t)EEPROM.read(0x03);
  max_altitude_from_eeprom = EEPROM.read(0x04)<< 8 | EEPROM.read(0x05);
  max_speed_from_eeprom = (uint32_t)EEPROM.read(0x06);
  max_speed = max_speed_from_eeprom;
}

// 主循环
void loop() {
  if(key_press_timer > 0){
    key_press_timer = 0;
    button_store = -1;
  }

  if(button_store != -1){
    while(analogRead(ANALOG_PIN) < 1000){
      delay(10);
      if(key_press_timer < 200)key_press_timer ++;
      if(key_press_timer == 200){
        digitalWrite(BUZZER_PIN, HIGH);
        delay(10);
        digitalWrite(BUZZER_PIN, LOW);
        delay(50);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(10);
        digitalWrite(BUZZER_PIN, LOW);
        delay(500);

      }
    }
    if(key_press_timer < 200){
      digitalWrite(BUZZER_PIN, HIGH);
      delay(10);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  if((telemetry_lost == 1 || alarm_sound == 1) && button_store != -1){
    if(alarm_sound == 1)alarm_sound = 2;
    if(telemetry_lost == 1)telemetry_lost = 2;
    page = 0;
    lcd.clear();
    button_store = -1;
  }

  if(button_store != -1 && key_press_timer < 200){
    if(button_store < 350 && button_store > 250) page--; // Down键
    if(button_store < 150 && button_store > 100) page++; // Up键
    if(button_store < 800 && button_store > 700) page=0; // Select键
    if(page < 0)page = 0;
    if(page > 6)page = 7;
    button_store = -1;
  }

  if(start > 1 && flight_timer_start == 0){
    flight_time_from_eeprom = (uint32_t)EEPROM.read(0x00)<< 24 | (uint32_t)EEPROM.read(0x01)<< 16 | (uint32_t)EEPROM.read(0x02)<< 8 | (uint32_t)EEPROM.read(0x03);
    flight_timer_from_start = millis();
    flight_timer = millis() - flight_timer_previous;
    flight_timer_start = 1;
  }

  if(start == 0 && flight_timer_start == 1){
    if(max_altitude_meters > max_altitude_from_eeprom){
      max_altitude_from_eeprom = max_altitude_meters;
      EEPROM.write(0x04, max_altitude_from_eeprom >> 8);
      EEPROM.write(0x05, max_altitude_from_eeprom);
    }

    if(max_speed > max_speed_from_eeprom){
      max_speed_from_eeprom = max_speed;
      EEPROM.write(0x06, max_speed_from_eeprom);
    }

    flight_time_from_eeprom += (millis() - flight_timer_from_start)/1000;
    EEPROM.write(0x00, flight_time_from_eeprom >> 24);
    EEPROM.write(0x01, flight_time_from_eeprom >> 16);
    EEPROM.write(0x02, flight_time_from_eeprom >> 8);
    EEPROM.write(0x03, flight_time_from_eeprom);

    flight_timer_previous = millis() - flight_timer;
    flight_timer_start = 0;
  }

  while (Serial.available()) {
    // 将接收的数据加载到接收的缓冲区数组中     
    receive_buffer[receive_buffer_counter] = Serial.read();
    // 在接收到的数据流中搜索开始签名
    if(receive_byte_previous == 'J' && receive_buffer[receive_buffer_counter] == 'B'){
      // 如果找到起始签名，则重置接收缓冲区计数器
      receive_buffer_counter = 0;
      // 累加接收开始检测计数器，以检查是否接收到完整的数据流
      receive_start_detect ++;
      // 如果检测到两个启动签名，则可能有完整的数据集可用
      if(receive_start_detect >= 2)get_data();
    } else {
      // 如果未检测到启动签名, 为下一个循环安全当前接收的字节
      receive_byte_previous = receive_buffer[receive_buffer_counter];
      // 累加接收缓冲区计数器变量
      receive_buffer_counter ++;
      // 当接收缓冲区计数器变量大于48时，将其重置
      if(receive_buffer_counter > 48)receive_buffer_counter = 0;
    }
  }

  if(start > 1){
    minutes = (millis() - flight_timer)/60000;
    seconds = ((millis() - flight_timer)-minutes*60000)/1000;
  }

  if(page != previous_page){
    previous_page = page;
    lcd.clear();
  }

  if(page == 0){
    if(flight_mode <= 3){
      lcd.setCursor(0, 0);
      lcd.print("M");
      lcd.print(flight_mode);
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("R");
      lcd.print(flight_mode - 4);
    }

    lcd.setCursor(5, 0);
    if(battery_voltage < 10)lcd.print("0");
    lcd.print(battery_voltage,1);
    lcd.print("V ");

    lcd.setCursor(11, 0);
    if(altitude_meters < 0)lcd.print("-");
    else lcd.print("+");
    if(altitude_meters < 100)lcd.print("0");
    if(altitude_meters < 10)lcd.print("0");
    lcd.print(abs(altitude_meters));
    lcd.print("m");

    lcd.setCursor(2, 0);
    lcd.print("E");
    lcd.print(error);

    lcd.setCursor(0, 1);
    if(fix_type == 3)lcd.print("S");
    else lcd.print("s");
    if(number_used_sats < 10)lcd.print("0");
    lcd.print(number_used_sats);

    lcd.setCursor(5, 1);
    if(minutes < 10)lcd.print("0");
    lcd.print(minutes);
    lcd.print(":");
    if(seconds < 10)lcd.print("0");
    lcd.print(seconds);

    lcd.setCursor(11, 1);
    lcd.print("H");
    if(actual_compass_heading < 100)lcd.print("0");
    if(actual_compass_heading < 10)lcd.print("0");
    lcd.print(actual_compass_heading);
    if(heading_lock)lcd.print("L");
    else lcd.print((char)223);
  }

  if(page == 1){
    if(key_press_timer == 200){
      button_store = -1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset max spd?");
      lcd.setCursor(0, 1);
      lcd.print("Select = yes");
      while(button_store == -1)delay(10);
      if(button_store < 800 && button_store > 700){
        while(analogRead(ANALOG_PIN) < 1000)delay(10);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Max spd is reset");
        EEPROM.write(0x06, 0x00);
        max_speed_from_eeprom = EEPROM.read(0x06);
        max_speed = max_speed_from_eeprom;
        delay(2000);
      }
      lcd.clear();
    }
    else{
      lcd.setCursor(0, 0);
      lcd.print("Speed     Max");
      lcd.setCursor(0, 1);
      if(speed_kmph < 100)lcd.print("0");
      if(speed_kmph < 10)lcd.print("0");
      lcd.print(speed_kmph);
      lcd.print("kph    ");
      if(max_speed < 100)lcd.print("0");
      if(max_speed < 10)lcd.print("0");
      lcd.print(max_speed);
      lcd.print("kph");
    }
  }

  if(page == 2){
    lcd.setCursor(0, 0);
    lcd.print("Lat:");
    lcd.print(l_lat_gps);
    lcd.setCursor(0, 1);
    lcd.print("Lon:");
    lcd.print(l_lon_gps);
  }

  if(page == 3){
    if(key_press_timer == 200){
      button_store = -1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset max alt?");
      lcd.setCursor(0, 1);
      lcd.print("Select = yes");
      while(button_store == -1)delay(10);
      if(button_store < 800 && button_store > 700){
        while(analogRead(ANALOG_PIN) < 1000)delay(10);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Max alt is reset");
        EEPROM.write(0x04, 0x00);
        EEPROM.write(0x05, 0x00);
        max_altitude_from_eeprom = EEPROM.read(0x04)<< 8 | EEPROM.read(0x05);
        delay(2000);
      }
      lcd.clear();
    }
    else{
      lcd.setCursor(0, 0);
      lcd.print("Max altitude:");
      lcd.setCursor(0, 1);
      if(max_altitude_meters < 100)lcd.print("0");
      if(max_altitude_meters < 10)lcd.print("0");
      lcd.print(max_altitude_meters);
      lcd.print("m     mem");
      if(max_altitude_from_eeprom < 100)lcd.print("0");
      if(max_altitude_from_eeprom < 10)lcd.print("0");
      lcd.print(max_altitude_from_eeprom);
      lcd.print("m");
    }
  }

  if(page == 4){
    lcd.setCursor(0, 0);
    lcd.print("roll: ");
    if(roll_angle >= 0)lcd.print("+");
    else lcd.print("-");
    if(roll_angle < 10 && roll_angle > -10)lcd.print("0");
    lcd.print(abs(roll_angle));
    lcd.setCursor(0, 1);
    lcd.print("pitch:");
    if(pitch_angle >= 0)lcd.print("+");
    else lcd.print("-");
    if(pitch_angle < 10 && pitch_angle > -10)lcd.print("0");
    lcd.print(abs(pitch_angle));
  }

  if(page == 5){
    if(key_press_timer == 200){
      button_store = -1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset timer?");
      lcd.setCursor(0, 1);
      lcd.print("Select = yes");
      while(button_store == -1)delay(10);
      if(button_store < 800 && button_store > 700){
        while(analogRead(ANALOG_PIN) < 1000)delay(10);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Timer is reset");
        EEPROM.write(0x00, 0x00);
        EEPROM.write(0x01, 0x00);
        EEPROM.write(0x02, 0x00);
        EEPROM.write(0x03, 0x00);
        flight_time_from_eeprom = (uint32_t)EEPROM.read(0x00)<< 24 | (uint32_t)EEPROM.read(0x01)<< 16 | (uint32_t)EEPROM.read(0x02)<< 8 | EEPROM.read(0x03);
        delay(2000);
      }
      //telemetry_lost = 2;
      lcd.clear();
    }
    else{
      lcd.setCursor(0, 0);
      lcd.print("Tot flight time");
      lcd.setCursor(0, 1);
      hours_flight_time = flight_time_from_eeprom/3600;
      minutes_flight_time = (flight_time_from_eeprom - (hours_flight_time*3600))/60;
      seconds_flight_time = flight_time_from_eeprom - (hours_flight_time*3600) - (minutes_flight_time*60);
      if(hours_flight_time < 10)lcd.print("0");
      lcd.print(hours_flight_time);
      lcd.print(":");
      if(minutes_flight_time < 10)lcd.print("0");
      lcd.print(minutes_flight_time);
      lcd.print(":");
      if(seconds_flight_time < 10)lcd.print("0");
      lcd.print(seconds_flight_time);
    }
  }

  if(page == 6){
    lcd.setCursor(0, 0);
    lcd.print("1:");
    if(adjustable_setting_1 < 10)lcd.print("0");
    lcd.print(adjustable_setting_1);
    lcd.setCursor(8, 0);
    lcd.print("3:");
    if(adjustable_setting_3 < 10)lcd.print("0");
    lcd.print(adjustable_setting_3);
    lcd.setCursor(0, 1);
    lcd.print("2:");
    if(adjustable_setting_2 < 10)lcd.print("0");
    lcd.print(adjustable_setting_2);

  }

  if(page == 7){
    lcd.setCursor(0, 0);
    lcd.print("Take-off thr:");
    lcd.setCursor(0, 1);
    lcd.print(takeoff_throttle);      
  }

  if(page == 100){
    lcd.setCursor(0, 0);
    lcd.print(" Lost telemetry");
    lcd.setCursor(0, 1);
    lcd.print("   connection!"); 
    delay(1000);    
  }
  if(page > 100){
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(error);
    lcd.setCursor(0, 1);
    if(error == 1)lcd.print("Battery LOW!");
    if(error == 5)lcd.print("Loop time exc.");
    if(error == 6)lcd.print("Take-off error");
    if(error == 10)lcd.print("Take-off thr.");

  }

  if(last_receive + 3000 < millis() && receive_start_detect && telemetry_lost == 0 && key_press_timer < 200) {
    telemetry_lost = 1;
    lcd.clear();
    receive_start_detect = 0;
    page = 100;
  }

  if(error && alarm_sound == 0){
    alarm_sound = 1;
    page = 100 + error;
  }

  if(error == 0 && alarm_sound){
    alarm_sound = 0;
    page = 0;
  }

  if((telemetry_lost == 1 || alarm_sound == 1) && next_sound < millis()){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(10);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(10);
    digitalWrite(BUZZER_PIN, LOW);
    next_sound = millis() + 1000;
  }
}

/**
 * 当接收到两个起始签名时，如果它们之间的
 * 接收数据是有效的数据流，则开始解析
 */
void get_data(void)
{
  // 重置检查字节变量
  check_byte = 0;
  // 计算校验字节
  for(temp_byte = 0; temp_byte <= 30; temp_byte++)
    check_byte ^= receive_buffer[temp_byte];
  // 第32个字节存放的是校验值
  if(check_byte == receive_buffer[31]){
    // 如果遥测信号丢失, 重新设置遥测丢失信号，因为接收到有效的数据流
    if(telemetry_lost > 0){
      telemetry_lost = 0;
      page = 0;
    }
    // 记录最后一次接收数据的时间
    last_receive = millis();
    // 将接收启动检测变量重置为 1
    receive_start_detect = 1;
    
    // 在下面的行中，从有效的数据流还原不同的变量。变量的名称与YMFC-32飞行控制器程序中的相同
    error = receive_buffer[0];
    flight_mode = receive_buffer[1];
    battery_voltage = (float)receive_buffer[2]/10.0;
    temperature = receive_buffer[3] | receive_buffer[4] << 8;
    roll_angle = receive_buffer[5] - 100;
    pitch_angle = receive_buffer[6] - 100;
    start = receive_buffer[7];
    altitude_meters = (receive_buffer[8] | receive_buffer[9] << 8) - 1000;
    if(altitude_meters > max_altitude_meters)max_altitude_meters = altitude_meters;
    takeoff_throttle = receive_buffer[10] | receive_buffer[11] << 8;
    actual_compass_heading = receive_buffer[12] | receive_buffer[13] << 8;
    heading_lock = receive_buffer[14];
    number_used_sats = receive_buffer[15];
    fix_type = receive_buffer[16];
    l_lat_gps = (int32_t)receive_buffer[17] | (int32_t)receive_buffer[18] << 8 | (int32_t)receive_buffer[19] << 16 | (int32_t)receive_buffer[20] << 24;
    l_lon_gps = (int32_t)receive_buffer[21] | (int32_t)receive_buffer[22] << 8 | (int32_t)receive_buffer[23] << 16 | (int32_t)receive_buffer[24] << 24;
    adjustable_setting_1 = (float)(receive_buffer[25] | receive_buffer[26] << 8)/100.0;
    adjustable_setting_2 = (float)(receive_buffer[27] | receive_buffer[28] << 8)/100.0;
    adjustable_setting_3 = (float)(receive_buffer[29] | receive_buffer[30] << 8)/100.0;

    if (number_used_sats >= 4)
    {
      lat_distance = abs(l_lat_gps - l_lat_gps_previous);
      lat_distance *= cos(((float)l_lat_gps/1000000.0) * 0.017453);

      lon_distance = abs(l_lon_gps - l_lon_gps_previous);
      lon_distance = sqrt((lon_distance * lon_distance) + (lat_distance * lat_distance));
      lon_distance /= 1.2626263;
      if(start == 0)lon_distance = 0;

      if(lon_distance < 250){
        speed_loop_counter ++;
        speed_buffer[3] = speed_buffer[2];
        speed_buffer[2] = speed_buffer[1];
        speed_buffer[1] = speed_buffer[0];
        speed_buffer[0] = lon_distance;

        if(speed_loop_counter == 3){
          speed_buffer[4] = speed_buffer[3] + speed_buffer[2] + speed_buffer[1] + speed_buffer[0];
          speed_buffer[4] /= 4;

          speed_loop_counter = 0;
          speed_kmph = speed_buffer[4];
          if(max_speed < speed_kmph)max_speed = speed_kmph;
        }

      }
      //Serial.println(speed_kmph);

      l_lat_gps_previous = l_lat_gps;
      l_lon_gps_previous = l_lon_gps;
    }
  }
}
