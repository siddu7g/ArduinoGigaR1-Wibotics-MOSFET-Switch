/* 
  Integrated Charging Control for GIGA R1 WiFi + Pixhawk (TELEM1)
  - Monitors MAVLink HEARTBEAT on Serial1 (TELEM1)
  - Detects GUIDED mode (set GUIDED_CUSTOM_MODE)
  - Sends SET_POSITION_TARGET_GLOBAL_INT to target coords
  - Sends MAV_CMD_NAV_LAND to land at target
  - Detects landing via ARMED -> DISARMED
  - After WAIT_AFTER_LAND_MS: enable charger (MOSFET_CHARGER_PIN) and disable ESC (MOSFET_ESC_PIN)
  - If vehicle re-arms: disable charger and re-enable ESC
  - Requires MAVLink headers: #include <mavlink.h>
*/

#include <Arduino.h>
#include <mavlink.h>

// ---------------- USER CONFIG ----------------
const uint32_t TELEM_BAUD = 57600;   // TELEM1 baudrate (match Pixhawk)

const uint8_t MOSFET_CHARGER_PIN = 24; // D6 -> gate for charger MOSFET (3.3V gate drive)
const uint8_t MOSFET_ESC_PIN     = 51; // D7 -> gate for ESC power MOSFET (3.3V gate drive)
const bool MOSFET_ACTIVE_HIGH    = true; // true if HIGH = ON

const uint32_t WAIT_AFTER_LAND_MS = 10000; // 10 seconds (adjust 10000..15000)

// GUIDED detection (set to your autopilot's GUIDED custom_mode integer)
const uint32_t GUIDED_CUSTOM_MODE = 4;    // <<-- edit if your autopilot reports different value

// Hardcoded target (fill before test)
// Use lat/lon in 1e7 degrees (e.g., 37.7749 -> 377749000)
const int32_t TARGET_LAT_E7 = 0;   // e.g., 377749000
const int32_t TARGET_LON_E7 = 0;   // e.g., -1224194000 (note sign)
const int32_t TARGET_ALT_CM  = 100; // desired approach altitude in cm (e.g., 100 -> 1m)
// ---------------------------------------------

HardwareSerial &telem = Serial1;   // TELEM1
HardwareSerial &dbg = Serial;      // USB debug serial

// mavlink parsing
mavlink_message_t msg;
mavlink_status_t status;

// Simple state machine
enum State {
  IDLE,
  GUIDED_COMMANDED,
  FLYING_TO_TARGET,
  WAIT_LANDED,
  CHARGING
};
State state = IDLE;

bool prev_armed = false;
bool in_guided = false;
unsigned long guidedCommandSentAt = 0;
unsigned long landedDetectedAt = 0;
const unsigned long SETPOINT_SEND_INTERVAL_MS = 1000;

// Helper functions forward
void sendPositionSetpoint(int32_t lat_e7, int32_t lon_e7, int32_t alt_cm);
void sendLandCommand(int32_t lat_e7, int32_t lon_e7);
void enableCharger(bool on);
void setESCPower(bool on);

void setup() {
  dbg.begin(115200);
  telem.begin(TELEM_BAUD);
  pinMode(MOSFET_CHARGER_PIN, OUTPUT);
  pinMode(MOSFET_ESC_PIN, OUTPUT);

  // Start safe: charger OFF, ESC ON
  digitalWrite(MOSFET_CHARGER_PIN, MOSFET_ACTIVE_HIGH ? LOW : HIGH); // charger off
  digitalWrite(MOSFET_ESC_PIN,     MOSFET_ACTIVE_HIGH ? HIGH : LOW); // ESC on

  dbg.println(F("=== GIGA R1 Charging Control ==="));
  dbg.print(F("TELEM baud: ")); dbg.println(TELEM_BAUD);
  dbg.print(F("Charger pin D")); dbg.println(MOSFET_CHARGER_PIN);
  dbg.print(F("ESC pin D")); dbg.println(MOSFET_ESC_PIN);
  dbg.print(F("GUIDED_CUSTOM_MODE: ")); dbg.println(GUIDED_CUSTOM_MODE);
  dbg.print(F("Target lat_e7: ")); dbg.println(TARGET_LAT_E7);
  dbg.print(F("Target lon_e7: ")); dbg.println(TARGET_LON_E7);
  dbg.print(F("Target alt (cm): ")); dbg.println(TARGET_ALT_CM);
  dbg.print(F("WAIT_AFTER_LAND_MS: ")); dbg.println(WAIT_AFTER_LAND_MS);
  dbg.println(F("Waiting for MAVLink heartbeat on Serial1..."));
}

void loop() {
  // read MAVLink and handle messages
  while (telem.available()) {
    uint8_t c = telem.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // parsed full message
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);

          bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
          uint32_t custom_mode = hb.custom_mode;

          // debug print (lightweight)
          dbg.print(F("HB: type=")); dbg.print(hb.type);
          dbg.print(F(" base_mode=0x")); dbg.print(hb.base_mode, HEX);
          dbg.print(F(" custom_mode=")); dbg.println(custom_mode);

          // GUIDED detection (enter)
          bool now_guided = (custom_mode == GUIDED_CUSTOM_MODE);
          if (now_guided && !in_guided) {
            in_guided = true;
            dbg.println(F("Detected GUIDED mode -> sending target and LAND"));
            // Send position setpoint and then a land command at that location
            sendPositionSetpoint(TARGET_LAT_E7, TARGET_LON_E7, TARGET_ALT_CM);
            guidedCommandSentAt = millis();
            state = GUIDED_COMMANDED;
            // Also immediately request a LAND at the coords (many autopilot implementations respect MAV_CMD_NAV_LAND)
            sendLandCommand(TARGET_LAT_E7, TARGET_LON_E7);
          } else if (!now_guided && in_guided) {
            in_guided = false;
            dbg.println(F("Exited GUIDED mode"));
            if (state == GUIDED_COMMANDED || state == FLYING_TO_TARGET) {
              state = IDLE;
            }
          }

          // Detect ARMED->DISARMED for landing
          if (prev_armed && !armed) {
            dbg.println(F("Detected ARMED->DISARMED (likely landed)"));
            // Only act if we had commanded GUIDED/landing
            if (state == GUIDED_COMMANDED || state == FLYING_TO_TARGET) {
              landedDetectedAt = millis();
              state = WAIT_LANDED;
            }
          }

          // If we are guided and still armed => flying to target
          if ((state == GUIDED_COMMANDED || state == IDLE) && armed && in_guided) {
            state = FLYING_TO_TARGET;
          }

          // If re-armed after charging: revert outputs
          if (!prev_armed && armed) {
            dbg.println(F("Detected DISARMED->ARMED (re-armed) => disabling charger and enabling ESC"));
            enableCharger(false);
            setESCPower(true);
            // Reset state machine
            state = IDLE;
          }

          prev_armed = armed;
          break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          // Optional diagnostics
          mavlink_global_position_int_t gp;
          mavlink_msg_global_position_int_decode(&msg, &gp);
          dbg.print(F("GPS lat=")); dbg.print(gp.lat / 1e7, 7);
          dbg.print(F(" lon=")); dbg.print(gp.lon / 1e7, 7);
          dbg.print(F(" rel_alt(m)=")); dbg.println(gp.relative_alt / 1000.0);
          break;
        }

        default:
          // ignore other messages
          break;
      } // switch msgid
    } // if parsed
  } // while available

  // state behavior
  unsigned long now = millis();
  switch (state) {
    case GUIDED_COMMANDED:
      // keep sending setpoint periodically to ensure autopilot receives it
      if (now - guidedCommandSentAt >= SETPOINT_SEND_INTERVAL_MS) {
        sendPositionSetpoint(TARGET_LAT_E7, TARGET_LON_E7, TARGET_ALT_CM);
        guidedCommandSentAt = now;
        dbg.println(F("Resent position setpoint"));
      }
      break;

    case FLYING_TO_TARGET:
      if (now - guidedCommandSentAt >= SETPOINT_SEND_INTERVAL_MS) {
        sendPositionSetpoint(TARGET_LAT_E7, TARGET_LON_E7, TARGET_ALT_CM);
        guidedCommandSentAt = now;
      }
      break;

    case WAIT_LANDED:
      // wait specified time then enable charger + disable ESC
      if (now - landedDetectedAt >= WAIT_AFTER_LAND_MS) {
        dbg.println(F("Post-landing wait elapsed -> enabling charger, disabling ESC"));
        enableCharger(true);
        setESCPower(false);
        state = CHARGING;
      }
      break;

    case CHARGING:
      // nothing active here - charger enabled until re-arm or manual reset
      break;

    case IDLE:
    default:
      break;
  }
}

/* ------------------ MAVLink send utilities ------------------ */

void sendPositionSetpoint(int32_t lat_e7, int32_t lon_e7, int32_t alt_cm) {
  // alt_cm -> mm
  int32_t alt_mm = alt_cm * 10;

  mavlink_set_position_target_global_int_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.time_boot_ms = millis();
  sp.target_system = 1;
  sp.target_component = 1;
  sp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
  sp.lat_int = lat_e7;
  sp.lon_int = lon_e7;
  sp.alt = alt_mm;
  // mask: ignore velocities/accel/yaw (we only want pos)
  sp.type_mask = (MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_VX |
                  MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_VY |
                  MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_VZ |
                  MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_AFX |
                  MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_AFY |
                  MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_AFZ |
                  MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_YAW |
                  MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_FIELD_YAW_RATE);

  mavlink_message_t pmsg;
  mavlink_msg_set_position_target_global_int_encode(255, 190, &pmsg, &sp);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &pmsg);
  telem.write(buf, len);

  dbg.print(F("Sent SET_POSITION_TARGET_GLOBAL_INT to: "));
  dbg.print(lat_e7 / 1e7, 7);
  dbg.print(F(", "));
  dbg.print(lon_e7 / 1e7, 7);
  dbg.print(F(", alt_cm=")); dbg.println(alt_cm);
}

void sendLandCommand(int32_t lat_e7, int32_t lon_e7) {
  // MAV_CMD_NAV_LAND (21). param5=lat (deg), param6=lon (deg) for many implementations.
  // Convert e7 -> deg
  float lat_deg = lat_e7 / 1e7f;
  float lon_deg = lon_e7 / 1e7f;

  mavlink_command_long_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.command = MAV_CMD_NAV_LAND; // 21
  cmd.confirmation = 0;
  // Most autopilots accept param5/6 as lat/lon
  cmd.param5 = lat_deg;
  cmd.param6 = lon_deg;
  // others: param7 altitude (m) - leave 0 to land at ground
  cmd.param7 = 0;

  mavlink_message_t cmsg;
  mavlink_msg_command_long_encode(255, 190, &cmsg, &cmd);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &cmsg);
  telem.write(buf, len);

  dbg.print(F("Sent MAV_CMD_NAV_LAND to: "));
  dbg.print(lat_deg, 7);
  dbg.print(F(", "));
  dbg.println(lon_deg, 7);
}

/* ------------------ Power control helpers ------------------ */

void enableCharger(bool on) {
  if (on) digitalWrite(MOSFET_CHARGER_PIN, MOSFET_ACTIVE_HIGH ? HIGH : LOW);
  else    digitalWrite(MOSFET_CHARGER_PIN, MOSFET_ACTIVE_HIGH ? LOW : HIGH);

  dbg.print(F("enableCharger(")); dbg.print(on ? "ON" : "OFF");
  dbg.print(F(") pin D")); dbg.println(MOSFET_CHARGER_PIN);
}

void setESCPower(bool on) {
  if (on) digitalWrite(MOSFET_ESC_PIN, MOSFET_ACTIVE_HIGH ? HIGH : LOW);
  else    digitalWrite(MOSFET_ESC_PIN, MOSFET_ACTIVE_HIGH ? LOW : HIGH);

  dbg.print(F("setESCPower(")); dbg.print(on ? "ON" : "OFF");
  dbg.print(F(") pin D")); dbg.println(MOSFET_ESC_PIN);
}
