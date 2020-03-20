/**
 * STM32版遥测系统
 * 使用OLED128x64显示屏
 */
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>

#define STM32_Board_LED PC13
#define OLED_ADDRESS 0x78
#define LINE(x) (13*(x))
#define ROW(x) (7*(x))
#define SERIAL_PORT Serial2

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, PB6, PB7, U8X8_PIN_NONE); 

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
uint16_t page_counter = 0;

void setup() {
  SERIAL_PORT.begin(9600);
  pinMode(STM32_Board_LED, OUTPUT);
  digitalWrite(STM32_Board_LED, HIGH);

  oled.begin();
  oled.enableUTF8Print();
  oled.setFont(u8g2_font_unifont_t_symbols);
  oled.clearBuffer();

  oled.setCursor(35, LINE(1));
  oled.print("YMFC-32");
  oled.setCursor(25, LINE(3));
  oled.print("telemetry");
  oled.sendBuffer();
  delay(4500);

  oled.clear();

  button_store = -1;
  telemetry_lost = 2;
  flight_time_from_eeprom = 0;
  max_altitude_from_eeprom = 0;
  max_speed_from_eeprom = 0;
  max_speed = max_speed_from_eeprom;
  receive_buffer_counter = 0;
}

void loop() {
  if (key_press_timer > 0) {
    key_press_timer = 0;
    button_store = -1;
  }

  if(button_store != -1){
    while(analogRead(0) < 1000){
      delay(10);
      if(key_press_timer < 200)key_press_timer ++;
      if(key_press_timer == 200){
        digitalWrite(2, HIGH);
        delay(10);
        digitalWrite(2, LOW);
        delay(50);
        digitalWrite(2, HIGH);
        delay(10);
        digitalWrite(2, LOW);
        delay(500);
      }
    }
    if(key_press_timer < 200){
      digitalWrite(2, HIGH);
      delay(10);
      digitalWrite(2, LOW);
    }
  }

  if ((telemetry_lost == 1 || alarm_sound == 1) && button_store != -1) {
    if (alarm_sound == 1) {
      alarm_sound = 2;
    }
    if (telemetry_lost == 1) telemetry_lost = 2;
    page = 0;
    oled.clear();
    button_store = -1;
  }

  if (button_store != -1 && key_press_timer < 200) {
    if (button_store < 300 && button_store > 200) page--; // down
    if (button_store < 150 && button_store > 50) page++;  // up
    if (button_store < 700 && button_store > 600) page = 0; //select
    if (page < 0) page = 0;
    if (page > 6) page = 7;
    button_store = -1;
  }

  if (start > 1 && flight_timer_start == 0) {
    flight_time_from_eeprom = 0;
    flight_timer_from_start = millis();
    flight_timer = millis() - flight_timer_previous;
    flight_timer_start = 1;
  }

  if(start == 0 && flight_timer_start == 1){
    if(max_altitude_meters > max_altitude_from_eeprom){
      max_altitude_from_eeprom = max_altitude_meters;
//      EEPROM.write(0x04, max_altitude_from_eeprom >> 8);
//      EEPROM.write(0x05, max_altitude_from_eeprom);
    }

    if(max_speed > max_speed_from_eeprom){
      max_speed_from_eeprom = max_speed;
//      EEPROM.write(0x06, max_speed_from_eeprom);
    }

    flight_time_from_eeprom += (millis() - flight_timer_from_start)/1000;
//    EEPROM.write(0x00, flight_time_from_eeprom >> 24);
//    EEPROM.write(0x01, flight_time_from_eeprom >> 16);
//    EEPROM.write(0x02, flight_time_from_eeprom >> 8);
//    EEPROM.write(0x03, flight_time_from_eeprom);

    flight_timer_previous = millis() - flight_timer;
    flight_timer_start = 0;
  }

  while(SERIAL_PORT.available()) {                                              //If there are bytes available.      
    receive_buffer[receive_buffer_counter] = SERIAL_PORT.read();                //Load them in the received_buffer array.
    //Search for the start signature in the received data stream.
    if(receive_byte_previous == 'J' && receive_buffer[receive_buffer_counter] == 'B') {
      receive_buffer_counter = 0;                                           //Reset the receive_buffer_counter counter if the start signature if found.
      receive_start_detect ++;                                              //Increment the receive_start_detect to check for a full data stream reception.
      if(receive_start_detect >= 2)get_data();                              //If there are two start signatures detected there could be a complete data set available.
    } else {                                                                //If there is no start signature detected.
      receive_byte_previous = receive_buffer[receive_buffer_counter];       //Safe the current received byte for the next loop.
      receive_buffer_counter ++;                                            //Increment the receive_buffer_counter variable.
      if(receive_buffer_counter > 48)receive_buffer_counter = 0;            //Reset the receive_buffer_counter variable when it becomes larger than 38.
    }
    digitalWrite(STM32_Board_LED, !digitalRead(STM32_Board_LED));
  }

  if(start > 1){
    minutes = (millis() - flight_timer)/60000;
    seconds = ((millis() - flight_timer)-minutes*60000)/1000;
  }

  if(page != previous_page){
    previous_page = page;
    oled.clear();
  }

  if(page == 0) {
    page_counter++;
    if (page_counter % 200 == 0) oled.clear();
    
    if (page_counter < 200) {
      if(flight_mode <= 3){
        oled.setCursor(ROW(0), LINE(1));
        oled.print("M");
        oled.print(flight_mode);
      } else {
        oled.setCursor(ROW(0), LINE(1));
        oled.print("R");
        oled.print(flight_mode - 4);
      }
      oled.print("E");
      oled.print(error);
  
      oled.setCursor(ROW(5), LINE(1));
      if(battery_voltage < 10)
        oled.print("0");
      oled.print(battery_voltage, 1);
      oled.print("V");
  
      oled.setCursor(ROW(12)-3, LINE(1));
      if(altitude_meters < 0)oled.print("-");
      else oled.print("+");
      if(altitude_meters < 100)oled.print("0");
      if(altitude_meters < 10)oled.print("0");
      oled.print(abs(altitude_meters));
      oled.print("m");
  
      oled.setCursor(ROW(0), LINE(2));
      if(fix_type == 3) 
        oled.print("S");
      else 
        oled.print("s");
      if(number_used_sats < 10) 
        oled.print("0");
      oled.print(number_used_sats);
  
      oled.setCursor(ROW(5), LINE(2));
      if(minutes < 10)
        oled.print("0");
      oled.print(minutes);
      oled.print(":");
      if(seconds < 10)
        oled.print("0");
      oled.print(seconds);
  
      oled.setCursor(ROW(12)-3, LINE(2));
      oled.print("H");
      if(actual_compass_heading < 100)
        oled.print("0");
      if(actual_compass_heading < 10)
        oled.print("0");
      oled.print(actual_compass_heading);
      if(heading_lock)
        oled.print("L");
      else
        oled.print((char)223);
  
      oled.setCursor(ROW(0), LINE(3)+3);
      oled.print("Roll: ");
      if(roll_angle >= 0)
        oled.print("+");
      else 
        oled.print("-");
      if(roll_angle < 10 && roll_angle > -10)
        oled.print("0");
      oled.print(abs(roll_angle));
      oled.setCursor(ROW(0), LINE(4)+6);
      oled.print("Pitch: ");
      if(pitch_angle >= 0)
        oled.print("+");
      else 
        oled.print("-");
      if(pitch_angle < 10 && pitch_angle > -10)
        oled.print("0");
      oled.print(abs(pitch_angle));
    } 
    
    if (page_counter >= 200 && page_counter < 400) {
      oled.setCursor(ROW(0), LINE(1));
      oled.print("Max altitude:");
      oled.setCursor(ROW(0), LINE(2));
      if(max_altitude_meters < 100)oled.print("0");
      if(max_altitude_meters < 10)oled.print("0");
      oled.print(max_altitude_meters);
      oled.print("m  mem:");
      if(max_altitude_from_eeprom < 100)oled.print("0");
      if(max_altitude_from_eeprom < 10)oled.print("0");
      oled.print(max_altitude_from_eeprom);
      oled.print("m");
      
      oled.setCursor(ROW(0), LINE(3)+3);
      oled.print("Lat:");
      oled.print(l_lat_gps);
      oled.setCursor(ROW(0), LINE(4)+6);
      oled.print("Lon:");
      oled.print(l_lon_gps);
    }

    if (page_counter >= 400 && page_counter < 600) {
      oled.setCursor(ROW(0), LINE(1));
      oled.print("Take-off thr:");
      oled.print(takeoff_throttle); 

      oled.setCursor(ROW(0), LINE(2)+3);
      oled.print("1:");
      if(adjustable_setting_1 < 10)oled.print("0");
      oled.print(adjustable_setting_1);
      oled.setCursor(ROW(0), LINE(3)+6);
      oled.print("2:");
      if(adjustable_setting_2 < 10)oled.print("0");
      oled.print(adjustable_setting_2);
      oled.setCursor(ROW(0), LINE(4)+9);
      oled.print("3:");
      if(adjustable_setting_3 < 10)oled.print("0");
      oled.print(adjustable_setting_3);
    }

//    if (page_counter >= 600 && page_counter < 800) {
//      oled.setCursor(ROW(0), LINE(1));
//      oled.print("Tot flight time");
//      oled.setCursor(ROW(0), LINE(2));
//      hours_flight_time = flight_time_from_eeprom/3600;
//      minutes_flight_time = (flight_time_from_eeprom - (hours_flight_time*3600))/60;
//      seconds_flight_time = flight_time_from_eeprom - (hours_flight_time*3600) - (minutes_flight_time*60);
//      if(hours_flight_time < 10)oled.print("0");
//      oled.print(hours_flight_time);
//      oled.print(":");
//      if(minutes_flight_time < 10)oled.print("0");
//      oled.print(minutes_flight_time);
//      oled.print(":");
//      if(seconds_flight_time < 10)oled.print("0");
//      oled.print(seconds_flight_time);
//    }
    
    if (page_counter > 600){
      oled.clear();
      page_counter = 0;
    }

    oled.sendBuffer();
  }

//  if(page == 1){
//    if(key_press_timer == 200){
//      button_store = -1;
//      oled.clear();
//      oled.setCursor(0, 0);
//      oled.print("Reset max spd?");
//      oled.setCursor(0, 1);
//      oled.print("Select = yes");
//      while(button_store == -1)delay(10);
//      if(button_store < 700 && button_store > 600){
//        while(analogRead(0) < 1000)delay(10);
//        oled.clear();
//        oled.setCursor(0, 0);
//        oled.print("Max spd is reset");
//        EEPROM.write(0x06, 0x00);
//        max_speed_from_eeprom = EEPROM.read(0x06);
//        max_speed = max_speed_from_eeprom;
//        delay(2000);
//      }
//      oled.clear();
//    }
//    else{
//      oled.setCursor(0, 0);
//      oled.print("Speed     Max");
//      oled.setCursor(0, 1);
//      if(speed_kmph < 100)oled.print("0");
//      if(speed_kmph < 10)oled.print("0");
//      oled.print(speed_kmph);
//      oled.print("kph    ");
//      if(max_speed < 100)oled.print("0");
//      if(max_speed < 10)oled.print("0");
//      oled.print(max_speed);
//      oled.print("kph");
//    }
//    oled.sendBuffer();
//  }
//
//  if(page == 2){
//    oled.setCursor(0, 0);
//    oled.print("Lat:");
//    oled.print(l_lat_gps);
//    oled.setCursor(0, 1);
//    oled.print("Lon:");
//    oled.print(l_lon_gps);
//    oled.sendBuffer();
//  }
//
//  if(page == 3){
//    if(key_press_timer == 200){
//      button_store = -1;
//      oled.clear();
//      oled.setCursor(0, 0);
//      oled.print("Reset max alt?");
//      oled.setCursor(0, 1);
//      oled.print("Select = yes");
//      while(button_store == -1)delay(10);
//      if(button_store < 700 && button_store > 600){
//        while(analogRead(0) < 1000)delay(10);
//        oled.clear();
//        oled.setCursor(0, 0);
//        oled.print("Max alt is reset");
//        EEPROM.write(0x04, 0x00);
//        EEPROM.write(0x05, 0x00);
//        max_altitude_from_eeprom = EEPROM.read(0x04)<< 8 | EEPROM.read(0x05);
//        delay(2000);
//      }
//      oled.clear();
//    }
//    else{
//      oled.setCursor(0, 0);
//      oled.print("Max altitude:");
//      oled.setCursor(0, 1);
//      if(max_altitude_meters < 100)oled.print("0");
//      if(max_altitude_meters < 10)oled.print("0");
//      oled.print(max_altitude_meters);
//      oled.print("m     mem");
//      if(max_altitude_from_eeprom < 100)oled.print("0");
//      if(max_altitude_from_eeprom < 10)oled.print("0");
//      oled.print(max_altitude_from_eeprom);
//      oled.print("m");
//      oled.sendBuffer();
//    }
//  }
//
//  if(page == 4){
//    oled.setCursor(0, 0);
//    oled.print("roll: ");
//    if(roll_angle >= 0)oled.print("+");
//    else oled.print("-");
//    if(roll_angle < 10 && roll_angle > -10)oled.print("0");
//    oled.print(abs(roll_angle));
//    oled.setCursor(0, 1);
//    oled.print("pitch:");
//    if(pitch_angle >= 0)oled.print("+");
//    else oled.print("-");
//    if(pitch_angle < 10 && pitch_angle > -10)oled.print("0");
//    oled.print(abs(pitch_angle));
//    oled.sendBuffer();
//  }
//
//  if(page == 5){
//    if(key_press_timer == 200){
//      button_store = -1;
//      oled.clear();
//      oled.setCursor(0, 0);
//      oled.print("Reset timer?");
//      oled.setCursor(0, 1);
//      oled.print("Select = yes");
//      oled.sendBuffer();
//      while(button_store == -1)delay(10);
//      if(button_store < 700 && button_store > 600){
//        while(analogRead(0) < 1000)delay(10);
//        oled.clear();
//        oled.setCursor(0, 0);
//        oled.print("Timer is reset");
//        EEPROM.write(0x00, 0x00);
//        EEPROM.write(0x01, 0x00);
//        EEPROM.write(0x02, 0x00);
//        EEPROM.write(0x03, 0x00);
//        flight_time_from_eeprom = (uint32_t)EEPROM.read(0x00)<< 24 | (uint32_t)EEPROM.read(0x01)<< 16 | (uint32_t)EEPROM.read(0x02)<< 8 | EEPROM.read(0x03);
//        delay(2000);
//      }
//      //telemetry_lost = 2;
//      oled.clear();
//    }
//    else{
//      oled.setCursor(0, 0);
//      oled.print("Tot flight time");
//      oled.setCursor(0, 1);
//      hours_flight_time = flight_time_from_eeprom/3600;
//      minutes_flight_time = (flight_time_from_eeprom - (hours_flight_time*3600))/60;
//      seconds_flight_time = flight_time_from_eeprom - (hours_flight_time*3600) - (minutes_flight_time*60);
//      if(hours_flight_time < 10)oled.print("0");
//      oled.print(hours_flight_time);
//      oled.print(":");
//      if(minutes_flight_time < 10)oled.print("0");
//      oled.print(minutes_flight_time);
//      oled.print(":");
//      if(seconds_flight_time < 10)oled.print("0");
//      oled.print(seconds_flight_time);
//      oled.sendBuffer();
//    }
//  }
//
//  if(page == 6){
//    oled.setCursor(0, 0);
//    oled.print("1:");
//    if(adjustable_setting_1 < 10)oled.print("0");
//    oled.print(adjustable_setting_1);
//    oled.setCursor(8, 0);
//    oled.print("3:");
//    if(adjustable_setting_3 < 10)oled.print("0");
//    oled.print(adjustable_setting_3);
//    oled.setCursor(0, 1);
//    oled.print("2:");
//    if(adjustable_setting_2 < 10)oled.print("0");
//    oled.print(adjustable_setting_2);
//    oled.sendBuffer();
//  }
//
//  if(page == 7){
//    oled.setCursor(0, 0);
//    oled.print("Take-off thr:");
//    oled.setCursor(0, 1);
//    oled.print(takeoff_throttle);   
//    oled.sendBuffer();   
//  }

  if(page == 100){
    oled.setCursor(ROW(0), LINE(1));
    oled.print(" Lost telemetry");
    oled.setCursor(ROW(0), LINE(2));
    oled.print("   connection!"); 
    oled.sendBuffer();
    digitalWrite(STM32_Board_LED, HIGH);
//    delay(1000);  
  }
  if(page > 100) {
    oled.setCursor(ROW(0), LINE(1));
    oled.print("Error:");
    oled.print(error);
    oled.setCursor(ROW(0), LINE(2));
    if(error == 1)oled.print("Battery LOW!");
    if(error == 5)oled.print("Loop time exc.");
    if(error == 6)oled.print("Take-off error");
    if(error == 10)oled.print("Take-off thr.");
    oled.sendBuffer();
  }

  if(last_receive + 3000 < millis() && receive_start_detect && telemetry_lost == 0 && key_press_timer < 200){
    telemetry_lost = 1;
    oled.clear();
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
    digitalWrite(PC13, HIGH);
    delay(10);
    digitalWrite(PC13, LOW);
    delay(50);
    digitalWrite(PC13, HIGH);
    delay(10);
    digitalWrite(PC13, LOW);
    next_sound = millis() + 1000;
  }
}

void get_data(void) {
  check_byte = 0;                                                                         //Reset the check_byte variabel.
  for(temp_byte = 0; temp_byte <= 30; temp_byte++) {
    check_byte ^= receive_buffer[temp_byte];  //Calculate the check_byte.
  }
  if(check_byte == receive_buffer[31]) {                                                   //If the calculated check_byte and the received check_byte are the same.
    if(telemetry_lost > 0){                                                               //If the telemetry signal was lost.
      telemetry_lost = 0;                                                                 //Reset the telemetry lost signal because a valid data stream is received.
      page = 0;                                                                           //Start at page 1.
    }
    last_receive = millis();                                                              //Remember when this reception has arived.
    receive_start_detect = 1;                                                             //Reset the receive_start_detect variable to 1.
    //In the following lines the different variables are restored from the valid data stream.
    //The name of the variables are the same as in the YMFC-32 flight controller program.
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

    if(number_used_sats >= 4){
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
