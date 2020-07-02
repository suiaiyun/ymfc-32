/* MAVLInk_DroneLights
 *  by Juan Pedro López
 *  
 * This program was developed to connect an Arduino board with a Pixhawk via MAVLink 
 *   with the objective of controlling a group of WS2812B LED lights on board of a quad
 * 
 * The current version of the program is working properly.
 * 
 * TO DO:
 *  - Move STREAMS request to RC_CHANNELS to use values in logic
 *  - Add RC_CHANNLES_RAW messages monitoring: move #30 to RC_CHANNELS_RAW (#35)
 *      http://mavlink.org/messages/common#RC_CHANNELS_RAW
 *  - Look for message on low battery:
 *      To be tested: http://mavlink.org/messages/common#PARAM_REQUEST_READ
 *      To be checked: http://mavlink.org/messages/common#SYS_STATUS
 *  - Potential implementation of other alarms, like high intensity
 *      
 * You can restrict the maximum package size with this parameter in mavlink_types.h:

    #ifndef MAVLINK_MAX_PAYLOAD_LEN_
    // it is possible to override this, but be careful! Defa_
    #define **MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length_
    #endif_
 */


/**
 * In case we need a second serial port for debugging
 * Comment this line if no serial debugging is needed
 */
//#define SOFT_SERIAL_DEBUGGING
#ifdef SOFT_SERIAL_DEBUGGING
  /* Library to use serial debugging with a second board */
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial(12, 13); // RX, TX
#endif

// 定义蜂鸣器引脚
#define BUZZER_PIN   2  
#define ANALOG_PIN   0
#define BATTERY_COMPARE_VOLTAGE 10.8

#include <EEPROM.h>
#include <LiquidCrystal.h>

#include "mavlink.h"
//#include "common/mavlink_msg_request_data_stream.h"

// 初始化lcd引脚
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;


// Lights flashing adjustment
unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long next_interval = 0;      // next interval

//电池电压(V)
float battery_voltage;
//飞机姿态
int8_t roll_angle, pitch_angle;
//当前页码
int8_t page, previous_page;
int8_t flight_timer_start;
//错误模式
uint8_t error;
//可见卫星数量
uint8_t number_used_sats;
//当前模式
uint8_t flight_mode;
//定位类型
uint8_t fix_type;
//锁头模式标志
uint8_t heading_lock;
//飞行时间
uint8_t minutes, seconds, alarm_sound;
//解锁状态
uint8_t start;
uint8_t receive_start_detect;
uint8_t telemetry_lost;
//罗盘指向
int16_t actual_compass_heading;
int16_t button_store;
//高度
int16_t altitude_meters, max_altitude_meters, max_altitude_from_eeprom;
int32_t l_lat_gps, l_lon_gps;
uint16_t key_press_timer;
uint32_t next_sound, last_receive;

void setup() {
  
  // 设置一个中断来检查模拟输入A0上的键输入
  TCCR2A = 0;                                                               // 设置TCCR2A寄存器为零
  TCCR2B = 0;                                                               // 设置TCCR2B寄存器为零
  TIMSK2 |= (1 << OCIE2A);                                                  // 在TIMSK2寄存器中设置中断启用位OCIE2A
  TCCR2B |= (1 << CS22|1 << CS21|1 << CS20);                                // 将TCCRB寄存器中的CS20、CS21和CS22位设置为1024
  OCR2A = 249;                                                              // 比较寄存器设置为249=>16ms/（1s/（16.000.000MHz/1024））-1=249
  TCCR2A |= (1 << WGM21);                                                   // 将计数器2设置为CTC（比较时清除计时器）模式

  
  // MAVLink interface start
  Serial.begin(57600);
  Serial.println("MAVLink starting.");

#ifdef SOFT_SERIAL_DEBUGGING
  // [DEB] Soft serial port start
  mySerial.begin(57600);
  mySerial.println("Run debugging...");
#endif

  // 配置LCD屏幕宽度
  lcd.begin(16, 2);
  // 打印欢迎信息
  lcd.setCursor(4, 0);
  lcd.print("APM V2.8");
  lcd.setCursor(3, 1);
  lcd.print("telemetry");

  pinMode(BUZZER_PIN, OUTPUT);
  // 蜂鸣器提示音
  #ifndef SOFT_SERIAL_DEBUGGING
  digitalWrite(BUZZER_PIN, HIGH);
  delay(10);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(10);
  digitalWrite(BUZZER_PIN, LOW);
  delay(2500);
  #endif

  button_store = -1;
  telemetry_lost = 2;
  lcd.clear();
}

/**
 * 主循环
 */
void loop() {
  /**
   * Lights management 
   * Light pulses: 2 quick flashes per second. 100 ms each cycle
   */
  unsigned long currentMillis = millis();
  int i=0;

  /* Normal mode, lights on. */
  if (currentMillis - previousMillis >= next_interval) {
    /* Keep time last mode changed */
    previousMillis = currentMillis;

  }

  mavlink_loop();

  #ifndef SOFT_SERIAL_DEBUGGING
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
    if(page > 2)page = 3;
    button_store = -1;
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
  
  if(page == 1)
  {
    lcd.setCursor(0, 0);
    lcd.print("roll: ");
    if(roll_angle >= 0)lcd.print("+");
    else lcd.print("-");
    if(roll_angle < 10 && roll_angle > -10)lcd.print("0");
    lcd.print(abs(roll_angle));
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print("pitch:");
    if(pitch_angle >= 0)lcd.print("+");
    else lcd.print("-");
    if(pitch_angle < 10 && pitch_angle > -10)lcd.print("0");
    lcd.print(abs(pitch_angle));
    lcd.print(" ");
  }
  
  if(page == 2)
  {
    lcd.setCursor(0, 0);
    lcd.print("Lat:");
    lcd.print(l_lat_gps);
    lcd.setCursor(0, 1);
    lcd.print("Lon:");
    lcd.print(l_lon_gps);
  }
  
  if(page == 3)
  {
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
    else
    {
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
  
  if(page == 100)
  {
    lcd.setCursor(0, 0);
    lcd.print(" Lost telemetry");
    lcd.setCursor(0, 1);
    lcd.print("   connection!"); 
    // 发送数据请求
    Mav_Request_Data();
    delay(1000);    
  }
  
  if(page > 100)
  {
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(error);
    lcd.setCursor(0, 1);
    if(error == 1)lcd.print("Battery LOW!");
  }

  if(last_receive + 3000 < millis() && telemetry_lost == 0 && key_press_timer < 200) {
    telemetry_lost = 1;
    lcd.clear();
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
  #endif

/*
  if(start > 1 && flight_timer_start == 0){
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

    flight_timer_previous = millis() - flight_timer;
    flight_timer_start = 0;
  }
*/

}



/**
 * 中断程序
 */
ISR(TIMER2_COMPA_vect) {
  if(button_store == -1)button_store = analogRead(ANALOG_PIN);
  if(button_store > 1000)button_store = -1;
}
