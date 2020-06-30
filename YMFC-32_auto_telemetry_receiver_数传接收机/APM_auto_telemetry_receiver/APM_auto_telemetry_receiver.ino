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


// In case we need a second serial port for debugging
// Comment this line if no serial debugging is needed
//#define SOFT_SERIAL_DEBUGGING   
#ifdef SOFT_SERIAL_DEBUGGING
  // Library to use serial debugging with a second board
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
//罗盘指向
uint8_t actual_compass_heading;
uint8_t telemetry_lost;
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
 * 中断程序
 */
ISR(TIMER2_COMPA_vect) {
  if(button_store == -1)button_store = analogRead(ANALOG_PIN);
  if(button_store > 1000)button_store = -1;
}

/**
 * 主循环
 */
void loop() {
  // Lights management
  // Light pulses: 2 quick flashes per second. 100 ms each cycle
  unsigned long currentMillis = millis();
  int i=0;

  // Normal mode, lights on.
  if (currentMillis - previousMillis >= next_interval) {
    // Keep time last mode changed
    previousMillis = currentMillis;

  }

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
  else if(page == 1)
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
  else if(page == 2)
  {
    lcd.setCursor(0, 0);
    lcd.print("Lat:");
    lcd.print(l_lat_gps);
    lcd.setCursor(0, 1);
    lcd.print("Lon:");
    lcd.print(l_lon_gps);
  }
  else if(page == 3)
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
  else if(page == 100)
  {
    lcd.setCursor(0, 0);
    lcd.print(" Lost telemetry");
    lcd.setCursor(0, 1);
    lcd.print("   connection!"); 
    delay(1000);    
  }
  else if(page > 100)
  {
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

  if(page == 100){
    lcd.setCursor(0, 0);
    lcd.print(" Lost telemetry");
    lcd.setCursor(0, 1);
    lcd.print("   connection!"); 
    delay(1000);    
  } else if(page > 100){
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(error);
    lcd.setCursor(0, 1);
    if(error == 1)lcd.print("Battery LOW!");
    if(error == 5)lcd.print("Loop time exc.");
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

        
  // MAVLink
  /* The default UART header for your MCU */ 
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;

#ifdef SOFT_SERIAL_DEBUGGING
    Serial.write(buf,len);
    //mySerial.println("Ardu HB");
#else
    Serial.write(buf, len);
#endif

    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Request streams from Pixhawk
#ifdef SOFT_SERIAL_DEBUGGING
      mySerial.println("Streams requested!");
#endif
      Mav_Request_Data();
      num_hbs_pasados=0;
    }

  }

  // Check reception buffer
  comm_receive();
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02,0x05};

    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
#ifdef SOFT_SERIAL_DEBUGGING
    Serial.write(buf,len);
#else
    Serial.write(buf, len);
#endif
  }
  
  // Request: PARAM_REQUEST_LIST. Only for full log recording
  /*
   * Primitive: mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                   uint8_t target_system, uint8_t target_component)
   */
/*
  // Configure
  uint8_t system_id=2;
  uint8_t component_id=200;
  // mavlink_message_t* msg;
  uint8_t target_system=1;
  uint8_t target_component=0;

  // Pack
  mavlink_msg_param_request_list_pack(system_id, component_id, &msg,
    target_system, target_component);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send
#ifdef SOFT_SERIAL_DEBUGGING
    pxSerial.write(buf,len);
#else
    Serial.write(buf, len);
#endif
*/
}



void comm_receive() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  // Echo for manual debugging
  // Serial.println("---Start---");

#ifdef SOFT_SERIAL_DEBUGGING
  while(Serial.available()>0) {
    uint8_t c = Serial.read();
#else
  while(Serial.available()>0) {
    uint8_t c = Serial.read();
#endif

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
#ifdef SOFT_SERIAL_DEBUGGING
            //mySerial.println("PX HB");
#endif
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            battery_voltage = (float)sys_status.voltage_battery / 1000;
            if (battery_voltage < BATTERY_COMPARE_VOLTAGE) error = 1;
            
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("PX SYS STATUS: ");
            mySerial.print("[Bat (V): ");
            mySerial.print(battery_voltage);
            mySerial.print("], [Bat (A): ");
            mySerial.print(sys_status.current_battery);
            mySerial.print("], [Comms loss (%): ");
            mySerial.print(sys_status.drop_rate_comm);
            mySerial.println("]");
#endif
            
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
            //mavlink_message_t* msg;
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX PARAM_VALUE");
            mySerial.println(param_value.param_value);
            mySerial.println(param_value.param_count);
            mySerial.println(param_value.param_index);
            mySerial.println(param_value.param_id);
            mySerial.println(param_value.param_type);
            mySerial.println("------ Fin -------");
#endif
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX RAW IMU");
            mySerial.println(raw_imu.xacc);
#endif
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            roll_angle = (int8_t)(attitude.roll * 100);
            pitch_angle = (int8_t)(attitude.pitch * 100);
            
#ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX ATTITUDE");
            mySerial.println(attitude.roll);
            mySerial.println(attitude.pitch);
            mySerial.println(roll_angle);
            mySerial.println(pitch_angle);
#endif
            
          }
          break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
          mavlink_gps_raw_int_t gps_raw;
          mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
          l_lat_gps = gps_raw.lat;
          l_lon_gps = gps_raw.lon;
          number_used_sats = gps_raw.satellites_visible;
          fix_type = gps_raw.fix_type;
        
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX GPS RAW");
            mySerial.println(gps_raw.lat);
            mySerial.println(gps_raw.lon);
            mySerial.println(gps_raw.satellites_visible);
          #endif
        }
        break;

        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        {
          mavlink_nav_controller_output_t nav_controller;
          mavlink_msg_nav_controller_output_decode(&msg, &nav_controller);
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.println("PX NAV CONTROLLER");
            mySerial.println(nav_controller.nav_roll);
            mySerial.println(nav_controller.nav_pitch);
            mySerial.println(nav_controller.target_bearing);
          #endif 
        }
        break;
        
       default:
#ifdef SOFT_SERIAL_DEBUGGING
          mySerial.print("--- Otros: ");
          mySerial.print("[ID: ");
          mySerial.print(msg.msgid);
          mySerial.print("], [seq: ");
          mySerial.print(msg.seq);
          mySerial.println("]");
#endif
          break;
      }
    }
  }
}
