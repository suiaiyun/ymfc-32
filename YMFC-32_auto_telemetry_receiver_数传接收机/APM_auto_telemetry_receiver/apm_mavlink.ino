#include "mavlink/mavlink.h"

unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000;
const int num_hbs = 60;
int num_hbs_pasados = num_hbs;

int leds_status = 0;
int leds_modo = 1;

void mavlink_setup()
{
  Serial.begin(57600);
}

void mavlink_loop()
{
  // MAVLink 配置
  int sysid = 1;
  int compid = 158;
  int type = MAV_TYPE_QUADROTOR;

  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;
  uint32_t custom_mode = 0;
  uint8_t system_state = MAV_STATE_STANDBY;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, 
    system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  unsigned long currentMillisMAVLink = millis();
  
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) 
  {
    previousMillisMAVLink = currentMillisMAVLink;

    Serial.write(buf, len);
    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs)
    {
      Mav_Request_Data();
      num_hbs_pasados = 0;
    }
  }

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

   const int maxStreams = 2;
   const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_POSITION};
   const uint16_t MAVRates[maxStreams] = {0x02, 0x05, 0x02};

   for (int i = 0; i < maxStreams; i++)
   {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, 
      MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
   }
}

void comm_receive()
{
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while (Serial.available() > 0)
  {
    uint8_t c = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:
        break;

        case MAVLINK_MSG_ID_SYS_STATUS:
          mavlink_sys_status_t sys_status;
          mavlink_msg_sys_status_decode(&msg, &sys_status);
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

          if (attitude.roll > 1) leds_modo = 0;
          else if (attitude.roll < -1) leds_modo = 2;
          else leds_modo = 1;
        break;

        case MAVLINK_MSG_ID_ALTITUDE:
          mavlink_altitude_t altitude;
          mavlink_msg_altitude_decode(&msg, &altitude);
        break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          mavlink_global_position_int_t global_position;
          mavlink_msg_global_position_int_decode(&msg, &global_position);
        break;

        case MAVLINK_MSG_ID_BATTERY_STATUS:
          mavlink_battery_status_t battery;
          mavlink_msg_battery_status_decode(&msg, &battery);
        break;

        default: 
        break;
      }
    }
  }
}
