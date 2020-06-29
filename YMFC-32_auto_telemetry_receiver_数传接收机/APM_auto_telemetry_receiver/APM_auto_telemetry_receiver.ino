#include "mavlink/ardupilotmega/mavlink.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>

// 定义蜂鸣器引脚
#define BUZZER_PIN   2  
#define ANALOG_PIN   0
#define TELEM_SERIAL Serial

// MAVLink 配置
#define SYSID           1
#define COMPID          158
#define TYPE            MAV_TYPE_QUADROTOR
#define SYSTEM_TYPE     MAV_TYPE_GENERIC
#define AUTOPILOT_TYPE  MAV_AUTOPILOT_INVALID
#define CUSTOM_MODE     0
#define SYSTEM_MODE     MAV_MODE_PREFLIGHT
#define SYSTEM_STATE    MAV_STATE_STANDBY
#define MAX_STREAMS     3
#define NUM_HBS         60
#define SERIAL_BAUDRATE 57600
#define BATTERY_VOLTAGE 10.5

unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000;
int num_hbs_pasados = NUM_HBS;

// MAVLink END


uint8_t flight_mode;
uint8_t telemetry_lost;
uint8_t alarm_silence;
uint8_t start;

uint8_t heading_lock;
int16_t button_store, roll_angle, pitch_angle;

uint32_t last_receive;
uint16_t l_lat_gps, l_lon_gps;
uint16_t actual_compass_heading;

// global 
float battery_voltage;
int16_t altitude_meters, max_altitude_meters, max_altitude_from_eeprom;
uint8_t error;
uint8_t number_used_sats, fix_type;
int8_t page, previous_page, flight_timer_start;
uint8_t hours, minutes, seconds, alarm_sound;
uint8_t receive_start_detect;
uint16_t key_press_timer;
uint32_t next_sound, flight_timer, flight_timer_previous, flight_timer_from_start;


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

  mavlink_setup();
  pinMode(BUZZER_PIN, OUTPUT);                                              //Set input 2 as output for the buzzer.

  // 配置LCD屏幕宽度
  lcd.begin(16, 2);
  // 打印欢迎信息
  lcd.setCursor(4, 0);
  lcd.print("APM V2.8");
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
  max_altitude_from_eeprom = EEPROM.read(0x04)<< 8 | EEPROM.read(0x05);

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
    if(page > 2)page = 3;
    button_store = -1;
  }

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

  mavlink_loop();

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
