void mavlink_loop()
{
  /* MAVLink */
  /* The default UART header for your MCU */ 
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  /**
   * Define the system type, in this case an airplane -> on-board controller
   * uint8_t system_type = MAV_TYPE_FIXED_WING;
   */
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  /**
   * 发送心跳包
   * mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
   */
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  /**
   * Send the message with the standard UART send function
   * uart0_send might be named differently depending on
   * the individual microcontroller / library in use.
   */
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    /* Guardamos la última vez que se cambió el modo */
    previousMillisMAVLink = currentMillisMAVLink;
    
    Serial.write(buf, len);

    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {

      #ifdef SOFT_SERIAL_DEBUGGING
        mySerial.println("Streams requested!");
      #endif
      
      /* Request streams from Pixhawk */
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
  const int  maxStreams = 4;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS,
                                          MAV_DATA_STREAM_POSITION,
                                          MAV_DATA_STREAM_EXTRA1, 
                                          MAV_DATA_STREAM_EXTRA2};
  const uint16_t MAVRates[maxStreams] = {0x01, 0x01, 0x01, 0x01};

    
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
    Serial.write(buf, len);
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
 
  /**
   * Echo for manual debugging
   * Serial.println("---Start---");
   */

  while (Serial.available() > 0) 
  {
    uint8_t c = Serial.read();

    /* Try to get a new message */
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    {
      if(telemetry_lost > 0)
      {
        telemetry_lost = 0;
        page = 0;
      }
      last_receive = millis();
      
      /* Handle message */
      switch(msg.msgid) 
      {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
        {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);

          switch (hb.system_status)
          {
            case MAV_STATE_UNINIT:
            case MAV_STATE_BOOT:
            case MAV_STATE_CALIBRATING:
            case MAV_STATE_STANDBY:
              start = 0;
            break;
            
            case MAV_STATE_ACTIVE: start = 2; break;

            case MAV_STATE_CRITICAL: error = 5; break;
            
            case MAV_STATE_EMERGENCY: error = 6; break;
          }

          switch (hb.custom_mode)
          {
            case 0: flight_mode = 1; break;   // 自稳模式
            case 2: flight_mode = 2; break;   // 定高模式
            case 3: heading_lock = 1; break;  // 锁头模式
            case 4: flight_mode = 4; break;   // RTL模式
            case 5: flight_mode = 3; break;   // 定点模式
          }
          
          /**
           * E.g. read GCS heartbeat and go into
           * comm lost mode if timer times out
           */
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("PX HB -- ");
            mySerial.print("custom_mode: ");
            mySerial.print(hb.custom_mode);
            mySerial.print(", system_status: ");
            mySerial.println(hb.system_status);
          #endif
        }
        break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
        {
          /* Message decoding: PRIMITIVE
           * mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
           */
          //mavlink_message_t* msg;
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
          battery_voltage = (float)sys_status.voltage_battery / 1000;
          if (battery_voltage < BATTERY_COMPARE_VOLTAGE) error = 1;
          
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("#");
            mySerial.print(MAVLINK_MSG_ID_PARAM_VALUE);
            mySerial.println(", battery_voltage: ");
            mySerial.println(battery_voltage);
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
            mySerial.print("#");
            mySerial.print(MAVLINK_MSG_ID_ATTITUDE);
            mySerial.print(", roll: ");
            mySerial.print(attitude.roll);
            mySerial.print(", pitch: ");
            mySerial.println(attitude.pitch);
          #endif
        }
        break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:  // #24
        {
          mavlink_gps_raw_int_t gps_raw;
          mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
          l_lat_gps = gps_raw.lat;
          l_lon_gps = gps_raw.lon;
          number_used_sats = gps_raw.satellites_visible;
          fix_type = gps_raw.fix_type;
        
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("#");
            mySerial.print(MAVLINK_MSG_ID_GPS_RAW_INT);
            mySerial.print(", lat: ");
            mySerial.print(l_lat_gps);
            mySerial.print(", lon: ");
            mySerial.print(l_lon_gps);
            mySerial.print(", alt: ");
            mySerial.print(gps_raw.alt);
            mySerial.print(", number_used_sats: ");
            mySerial.print(number_used_sats);
            mySerial.print(", fix_type: ");
            mySerial.println(fix_type);
          #endif
        }
        break;

        case MAVLINK_MSG_ID_VFR_HUD:  // #74
        {
          mavlink_vfr_hud_t vfr_hud;
          mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
          altitude_meters = vfr_hud.alt;
          actual_compass_heading = vfr_hud.heading;
          
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("#");
            mySerial.print(MAVLINK_MSG_ID_VFR_HUD);
            mySerial.print(", alt: ");
            mySerial.print(altitude_meters);
            mySerial.print(", airspeed: ");
            mySerial.print(vfr_hud.airspeed);
            mySerial.print(", groundspeed: ");
            mySerial.print(vfr_hud.groundspeed);
            mySerial.print(", throttle: ");
            mySerial.println(vfr_hud.throttle);
            
          #endif
        }
        break;

        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: // #62
        {
          mavlink_nav_controller_output_t nco;
          mavlink_msg_nav_controller_output_decode(&msg, &nco);
        
          #ifdef SOFT_SERIAL_DEBUGGING
            mySerial.print("#");
            mySerial.print(MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT);
            mySerial.print(", nav_roll: ");
            mySerial.print(nco.nav_roll);
            mySerial.print(", nav_pitch: ");
            mySerial.print(nco.nav_pitch);
            mySerial.println(", nav_bearing: ");
          #endif
        }
        break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // #33
        {
          mavlink_global_position_int_t global_position;
          mavlink_msg_global_position_int_decode(&msg, &global_position);
          
          #ifdef SOFT_SERIAL_DEBUGGING
//            mySerial.print("#");
//            mySerial.print(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
//            mySerial.print(", relative_alt: ");
//            mySerial.print(global_position.relative_alt);
//            mySerial.println("");
          #endif
        }
        break;

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: // #35
        {
          mavlink_rc_channels_override_t rc;
          mavlink_msg_rc_channels_override_decode(&msg, &rc);

          if (rc.chan7_raw > 1500) flight_mode = 4;  // RTL模式
          else 
          {
            if (rc.chan5_raw > 990 && rc.chan5_raw < 1230)
              flight_mode = 1;  // 自稳模式
            else if (rc.chan5_raw > 1491 && rc.chan5_raw < 1620)
              flight_mode = 2;  // 定高模式
            else if (rc.chan5_raw > 1750)
              flight_mode = 3;  // 定点模式
          }
          
          
          #ifdef SOFT_SERIAL_DEBUGGING
//            mySerial.print("#");
//            mySerial.print(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE);
//            mySerial.print(", chan5_raw: ");
//            mySerial.print(rc.chan5_raw);
//            mySerial.print(", chan7_raw: ");
//            mySerial.print(rc.chan7_raw);
//            mySerial.println("");
          #endif
        }
        break;

        
       default:
       {
        #ifdef SOFT_SERIAL_DEBUGGING
          mySerial.println("");
          mySerial.print("--- Otros: ");
          mySerial.print("ID: ");
          mySerial.print(msg.msgid);
          mySerial.print(", seq: ");
          mySerial.print(msg.seq);
          mySerial.println("");
        #endif
       }
       break;
      }
    }
  }
}
