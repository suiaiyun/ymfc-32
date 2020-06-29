void mavlink_setup()
{
  TELEM_SERIAL.begin(SERIAL_BAUDRATE);
}

void mavlink_loop()
{
  uint16_t len;
  unsigned long currentMillisMAVLink;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;
  
  mavlink_msg_heartbeat_pack(1, 0, &msg, TYPE, AUTOPILOT_TYPE, 
    SYSTEM_MODE, CUSTOM_MODE, SYSTEM_STATE);


  len = mavlink_msg_to_send_buffer(buf, &msg);

  currentMillisMAVLink = millis();

  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) 
  {
    previousMillisMAVLink = currentMillisMAVLink;

    TELEM_SERIAL.write(buf, len);
    num_hbs_pasados++;
    if (num_hbs_pasados >= NUM_HBS)
    {
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }
  }

  comm_receive();
}

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
void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  uint8_t maxStreams = 6;
  PROGMEM const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x02, 0x02};
  PROGMEM const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, 
                                                  MAV_DATA_STREAM_POSITION, 
                                                  MAV_DATA_STREAM_RAW_CONTROLLER,
                                                  MAV_DATA_STREAM_EXTRA2,
                                                  MAV_DATA_STREAM_EXTRA3,
                                                  MAV_DATA_STREAM_EXTRA1};

  for (int i = 0; i < maxStreams; i++)
  {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, 
      MAVStreams[i], MAVRates[i], 1);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    TELEM_SERIAL.write(buf, len);
  }
}

void comm_receive()
{
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t c;
  
  while (TELEM_SERIAL.available() > 0)
  {
    c = TELEM_SERIAL.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:
          flight_mode = 1;
        break;
        
        case MAVLINK_MSG_ID_SYS_STATUS:
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
          battery_voltage = ((uint16_t)(sys_status.voltage_battery * 10000)) / 100;
          if (BATTERY_VOLTAGE <= battery_voltage) error = 1;
        break;

        case MAVLINK_MSG_ID_PARAM_VALUE:
          mavlink_param_value_t param_value;
          mavlink_msg_param_value_decode(&msg, &param_value);
        break;

        // IMU 原始值
        case MAVLINK_MSG_ID_RAW_IMU:
          mavlink_raw_imu_t raw_imu;
          mavlink_msg_raw_imu_decode(&msg, &raw_imu);
          
        break;

        case MAVLINK_MSG_ID_ATTITUDE:
          mavlink_attitude_t attitude;
          mavlink_msg_attitude_decode(&msg, &attitude);

          roll_angle = attitude.roll * 100;
          pitch_angle = attitude.pitch * 100;
        break;

        case MAVLINK_MSG_ID_ALTITUDE:
          mavlink_altitude_t altitude;
          mavlink_msg_altitude_decode(&msg, &altitude);
          altitude_meters = altitude.altitude_relative;
        break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          mavlink_global_position_int_t global_position;
          mavlink_msg_global_position_int_decode(&msg, &global_position);
          l_lat_gps = global_position.lat;
          l_lon_gps = global_position.lon;
        break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
          mavlink_gps_raw_int_t gps_raw;
          mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
          number_used_sats = gps_raw.satellites_visible >=255 ? 0 : gps_raw.satellites_visible;
        break;

        default: 
        break;
      }
    }
  }
}
