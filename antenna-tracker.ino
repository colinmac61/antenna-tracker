#define Serial Serial1
#define BT_RX 5  // Physical Pin D2 (HC-05 TXD)
#define BT_TX 6  // Physical Pin D3 (HC-05 RXD)

#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>
#include <MAVLink.h>
#include <ESP32Servo.h>

// ==================== CONFIGURATION ====================
// WiFi Configuration for ELRS Backpack
const char* ssid = "ExpressLRS TX Backpack ABFA86";        // Change to your ELRS backpack SSID
const char* password = "expresslrs";          // Change to your ELRS backpack password
const int udp_port = 14550;                      // MAVLink UDP port

// Define your Static IP configuration
//IPAddress local_IP(10, 0, 0, 3); 
//IPAddress gateway(10, 0, 0, 1);
//IPAddress subnet(255, 255, 255, 0);
//IPAddress primaryDNS(8, 8, 8, 8);   

// GPIO Pins for Servos
const int AZIMUTH_SERVO_PIN = 13;   // Servo for horizontal rotation (East/West)
const int ELEVATION_SERVO_PIN = 14; // Servo for vertical rotation (Up/Down)

const float gearRatio = 2.0; 

// GPIO Pins for GPS (Serial2)
const int GPS_RX_PIN = 11;
const int GPS_TX_PIN = 12;

// GPS lock configuration
const int GPS_SATELLITE_MIN = 8;       // Minimum satellites for fix
const float GPS_HDOP_MAX = 1.4;        // Good fix threshold

// Servo Calibration (adjust based on your servos)
const int AZIMUTH_MIN_US = 400;    // Microseconds for min position (270 degrees West)
const int AZIMUTH_MAX_US = 2400;    // Microseconds for max position (90 degrees East)
const int ELEVATION_MIN_US = 400;  // Microseconds for min position (0 degrees Down)
const int ELEVATION_MAX_US = 2400;  // Microseconds for max position (90 degrees Up)

// Telemetry timeout (milliseconds)
const unsigned long MAVLINK_TIMEOUT_MS = 5000;  // Stop tracking if no data for 5 seconds

// ==================== GLOBAL VARIABLES ====================
WiFiUDP udp;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

// Servo objects for hardware PWM control
Servo azimuthServo;
Servo elevationServo;

// GPS Lock State
struct {
  boolean locked;      // Once a valid fixed position is accepted, it stays locked forever
  boolean searching;   // True while waiting for a valid GPS lock
  boolean fix_achieved;
} gps_state;

// Local tracker position
struct {
  double latitude;
  double longitude;
  double altitude;
  float hdop;
  char direction;  // 'N', 'S', 'E', 'W'
  boolean fix;
  boolean fix_announced;  // Track if we've announced the fix once
} local_position;

// Remote UAV position from MAVLink
struct {
  double latitude;
  double longitude;
  double altitude;
  boolean position_valid;
  unsigned long last_update;
} uav_position;

// Tracking angles
struct {
  double azimuth;
  double elevation;
  boolean valid;
} tracking_angles;

// Tracking offsets for field tuning
double azimuth_offset = 0.0;
double elevation_offset = 0.0;

// Timing control
unsigned long lastServoUpdate = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 100;  // milliseconds (10 Hz)

// MAVLink statistics for debugging
struct {
  unsigned long total_packets_received;
  unsigned long successful_decodes;
  unsigned long failed_decodes;
  unsigned int last_message_id;
} mavlink_stats;

// ==================== SETUP ====================
void setup() {
  Serial.begin(57600, SERIAL_8N1, BT_RX, BT_TX);
  delay(1000);
  
  Serial.println("\n\n=== ANTENNA TRACKER STARTUP ===");
  Serial.println("Initializing systems...");
  
  // Initialize GPS Serial
  SerialGPS.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[GPS] Serial initialized at 115200 baud");
  
  // Initialize servo pins with hardware PWM
  azimuthServo.attach(AZIMUTH_SERVO_PIN, AZIMUTH_MIN_US, AZIMUTH_MAX_US);
  elevationServo.attach(ELEVATION_SERVO_PIN, ELEVATION_MIN_US, ELEVATION_MAX_US);
  Serial.println("[SERVO] Servo pins initialized with hardware PWM");
  
  // Initialize WiFi
  // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
  //  Serial.println("STA Failed to configure Static IP");
  // }
  WiFi.mode(WIFI_STA);
  Serial.print("[WiFi] Connecting to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] ✓ Connected successfully");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] ✗ Failed to connect");
  }
  
  // Start UDP
  if (udp.begin(udp_port)) {
    Serial.print("[UDP] ✓ Listening on port ");
    Serial.println(udp_port);
  } else {
    Serial.println("[UDP] ✗ Failed to initialize UDP");
  }
  
  // Initialize GPS lock state
  gps_state.locked = false;
  gps_state.searching = true;
  gps_state.fix_achieved = false;
  
  // Initialize position data
  local_position.latitude = 0;
  local_position.longitude = 0;
  local_position.altitude = 0;
  local_position.hdop = 999.0;
  local_position.direction = 'N';
  local_position.fix = false;
  local_position.fix_announced = false;
  
  uav_position.latitude = 0;
  uav_position.longitude = 0;
  uav_position.altitude = 0;
  uav_position.position_valid = false;
  uav_position.last_update = 0;
  
  tracking_angles.azimuth = 0;
  tracking_angles.elevation = 0;
  tracking_angles.valid = false;
  
  lastServoUpdate = 0;
  
  // Initialize MAVLink statistics
  mavlink_stats.total_packets_received = 0;
  mavlink_stats.successful_decodes = 0;
  mavlink_stats.failed_decodes = 0;
  mavlink_stats.last_message_id = 0;
  
  Serial.println("[SYSTEM] Initialization complete");
  Serial.println("[GPS] Searching for 3D fix with 8+ satellites and HDOP ≤ 1.4...\n");
  print_command_help();
}

// ==================== MAIN LOOP ====================
void loop() {
  // Read GPS data
  update_local_gps();
  
  // Check for serial commands
  if (Serial.available()) {
    handle_serial_command();
  }
  
  // Receive MAVLink telemetry
  receive_mavlink_data();
  
  // Update servo positions at regular intervals (non-blocking)
  if (millis() - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdate = millis();
    
    // Calculate tracking angles if we have valid data
    if (local_position.fix && is_mavlink_data_fresh()) {
      calculate_tracking_angles();
      update_servo_positions();
    } else {
      if (!local_position.fix) {
        Serial.println("[WARNING] Waiting for GPS fix...");
      }
      if (!is_mavlink_data_fresh()) {
        Serial.println("[WARNING] Waiting for fresh UAV telemetry...");
        // Park servos to safe position when no valid data
        park_servos();
      }
    }
  }
}

// ==================== GPS FUNCTIONS ====================
void update_local_gps() {
  // If GPS has already been accepted and fixed, freeze the value permanently.
  if (gps_state.locked) {
    return;
  }

  // Read available GPS data
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);
  }
  
  // Check if we have a valid 3D fix with enough satellites and a good HDOP
  if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isValid()) {
    int satellites = gps.satellites.value();
    float hdop = gps.hdop.hdop();
    local_position.hdop = hdop;

    if (satellites >= GPS_SATELLITE_MIN && hdop <= GPS_HDOP_MAX) {
      local_position.latitude = gps.location.lat();
      local_position.longitude = gps.location.lng();
      local_position.altitude = gps.altitude.meters();
      local_position.fix = true;
      gps_state.searching = false;
      gps_state.locked = true;
      gps_state.fix_achieved = true;

      if (!local_position.fix_announced) {
        local_position.fix_announced = true;
        Serial.println("\n[GPS] ✓✓✓ 3D FIX LOCKED ✓✓✓");
        Serial.print("  Satellites: ");
        Serial.println(satellites);
        Serial.print("  HDOP: ");
        Serial.println(hdop, 2);
        Serial.print("  Lat: ");
        Serial.println(local_position.latitude, 6);
        Serial.print("  Lon: ");
        Serial.println(local_position.longitude, 6);
        Serial.print("  Alt: ");
        Serial.print(local_position.altitude);
        Serial.println(" m");
        Serial.print("  Direction: ");
        Serial.println(local_position.direction);
        Serial.println("[GPS] Position is now permanently locked until reboot\n");
      }
    } else {
      Serial.print("[GPS] ✗ Fix rejected - Sats: ");
      Serial.print(satellites);
      Serial.print(", HDOP: ");
      Serial.println(hdop, 2);
    }
  }

  if (gps.satellites.isUpdated()) {
    Serial.print("[GPS] Satellites: ");
    Serial.println(gps.satellites.value());
  }
}

// ==================== MAVLINK FUNCTIONS ====================
void receive_mavlink_data() {
  int packet_len = udp.parsePacket();
  if (packet_len) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = udp.read(buf, MAVLINK_MAX_PACKET_LEN);
    
    mavlink_message_t msg;
    mavlink_status_t status;
    
    mavlink_stats.total_packets_received++;
    
    int decode_count = 0;
    for (int i = 0; i < len; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        decode_count++;
        mavlink_stats.successful_decodes++;
        mavlink_stats.last_message_id = msg.msgid;

        // Only display and process position messages.
        if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT ||
            msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
          Serial.print("[MAVLink] MSG_ID: ");
          Serial.println(msg.msgid);
          process_mavlink_message(&msg);
        }
      }
    }
    
    if (decode_count == 0) {
      mavlink_stats.failed_decodes++;
    }
  }
}

void process_mavlink_message(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      handle_global_position_int(msg);
      break;

    case MAVLINK_MSG_ID_GPS_RAW_INT:
      handle_gps_raw_int(msg);
      break;

    default:
      // Ignore all other MAVLink message types.
      break;
  }
}

void handle_global_position_int(mavlink_message_t* msg) {
  mavlink_global_position_int_t pos;
  mavlink_msg_global_position_int_decode(msg, &pos);
  
  uav_position.latitude = pos.lat / 1e7;  // Convert from int to degrees
  uav_position.longitude = pos.lon / 1e7;
  uav_position.altitude = pos.alt / 1000.0;  // Convert from mm to meters
  uav_position.position_valid = true;
  uav_position.last_update = millis();
}

void handle_gps_raw_int(mavlink_message_t* msg) {
  mavlink_gps_raw_int_t gps_raw;
  mavlink_msg_gps_raw_int_decode(msg, &gps_raw);
  
  // Extract GPS data (coordinates are in degrees * 1e7, altitude in mm)
  uav_position.latitude = gps_raw.lat / 1e7;
  uav_position.longitude = gps_raw.lon / 1e7;
  uav_position.altitude = gps_raw.alt / 1000.0;  // Convert mm to meters
  uav_position.position_valid = true;
  uav_position.last_update = millis();
}

// ==================== TELEMETRY VALIDATION ====================
boolean is_mavlink_data_fresh() {
  if (!uav_position.position_valid) {
    return false;
  }
  
  unsigned long age = millis() - uav_position.last_update;
  if (age > MAVLINK_TIMEOUT_MS) {
    Serial.print("[MAVLink] ✗ Telemetry timeout - data age: ");
    Serial.print(age / 1000);
    Serial.println(" seconds (exceeded ");
    Serial.print(MAVLINK_TIMEOUT_MS / 1000);
    Serial.println("s limit)");
    uav_position.position_valid = false;
    return false;
  }
  
  return true;
}

// ==================== ANGLE CALCULATION ====================
void calculate_tracking_angles() {
  Serial.println("\n[CALC] Calculating tracking angles...");
  
  // Calculate distance and bearing from tracker to UAV
  double delta_lat = radians(uav_position.latitude - local_position.latitude);
  double delta_lon = radians(uav_position.longitude - local_position.longitude);
  double delta_alt = uav_position.altitude - local_position.altitude;
  
  // Haversine formula for distance
  double a = sin(delta_lat / 2) * sin(delta_lat / 2) +
             cos(radians(local_position.latitude)) * cos(radians(uav_position.latitude)) *
             sin(delta_lon / 2) * sin(delta_lon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = 6371000 * c;  // Earth radius in meters
  
  // Calculate bearing (azimuth)
  double y = sin(delta_lon) * cos(radians(uav_position.latitude));
  double x = cos(radians(local_position.latitude)) * sin(radians(uav_position.latitude)) -
             sin(radians(local_position.latitude)) * cos(radians(uav_position.latitude)) * cos(delta_lon);
  double bearing = atan2(y, x);
  bearing = degrees(bearing);
  
  // Adjust bearing based on tracker direction
  bearing = adjust_bearing_for_direction(bearing);
  
  // Ensure bearing is 0-360
  if (bearing < 0) bearing += 360;
  if (bearing >= 360) bearing -= 360;
  
  // Calculate elevation angle
  double elevation = atan2(delta_alt, distance);
  elevation = degrees(elevation);
  
  // Ensure elevation is 0-90
  elevation = constrain(elevation, 0, 90);

  // Apply manual tracking trim offsets (fine tuning)
  tracking_angles.azimuth = bearing + azimuth_offset;
  tracking_angles.elevation = elevation + elevation_offset;

  if (tracking_angles.azimuth < 0) tracking_angles.azimuth += 360;
  if (tracking_angles.azimuth >= 360) tracking_angles.azimuth -= 360;
  tracking_angles.elevation = constrain(tracking_angles.elevation, 0, 90);
  tracking_angles.valid = true;
  
  Serial.print("[CALC] ✓ Distance: ");
  Serial.print(distance);
  Serial.println(" m");
  Serial.print("[CALC] ✓ Azimuth: ");
  Serial.print(tracking_angles.azimuth);
  Serial.println(" °");
  Serial.print("[CALC] ✓ Elevation: ");
  Serial.print(tracking_angles.elevation);
  Serial.println(" °");
  Serial.print("[CALC] ✓ Azimuth Offset: ");
  Serial.print(azimuth_offset);
  Serial.print(" °, Elevation Offset: ");
  Serial.print(elevation_offset);
  Serial.println(" °");
}

double adjust_bearing_for_direction(double bearing) {
  // Adjust bearing based on manual direction setting
  // N (0°) = 0°, E (90°) = 90°, S (180°) = 180°, W (270°) = 270°
  
  switch (local_position.direction) {
    case 'N':  // North
      return bearing;
    case 'E':  // East
      return bearing - 90;
    case 'S':  // South
      return bearing - 180;
    case 'W':  // West
      return bearing - 270;
    default:
      return bearing;
  }
}

// ==================== SERVO CONTROL ====================
void update_servo_positions() {
  if (!tracking_angles.valid) {
    Serial.println("[SERVO] Tracking angles not valid, skipping update");
    return;
  }
  
  // Convert the compass bearing to a servo-friendly angle, with North at center.
  // Bearing is 0..360° where 0° = North, 90° = East, 180° = South, 270° = West.
  // Servo is designed with:
  //   0° = West, 90° = North, 180° = East
  double relative_azimuth = tracking_angles.azimuth;
  if (relative_azimuth > 180.0) {
    relative_azimuth -= 360.0;  // Convert to -180..180 relative to North
  }

  // Map relative bearing to servo angle: West=-90° -> 0°, North=0° -> 90°, East=+90° -> 180°
  int calculated_angle = constrain((int)round(relative_azimuth + 90.0), 0, 180);
  int azimuth_angle = 180 - calculated_angle; // Corrects for the servo orientation on the physical mount

  // Elevation follows 0..90° directly, but is scaled by gear ratio if needed.
  int elevation_angle = constrain((int)round(tracking_angles.elevation * gearRatio), 0, 180);

  // Write angles to servos using hardware PWM
  azimuthServo.write(azimuth_angle);
  elevationServo.write(elevation_angle);
  
  Serial.print("[SERVO] Azimuth: ");
  Serial.print(azimuth_angle);
  Serial.print("° (raw: ");
  Serial.print(tracking_angles.azimuth);
  Serial.print("°), Elevation: ");
  Serial.print(elevation_angle);
  Serial.println("°");
}

void park_servos() {
  // Move servos to neutral/safe position when no valid tracking data
  azimuthServo.write(90);   // Center azimuth
  elevationServo.write(0);  // Lower elevation
  Serial.println("[SERVO] Servos parked to safe position");
}

// ==================== SERIAL COMMANDS ====================
void handle_serial_command() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  
  Serial.print("[CMD] Received: ");
  Serial.println(command);
  
  if (command == "N") {
    local_position.direction = 'N';
    Serial.println("[CMD] Direction set to NORTH");
  }
  else if (command == "S") {
    local_position.direction = 'S';
    Serial.println("[CMD] Direction set to SOUTH");
  }
  else if (command == "E") {
    local_position.direction = 'E';
    Serial.println("[CMD] Direction set to EAST");
  }
  else if (command == "W") {
    local_position.direction = 'W';
    Serial.println("[CMD] Direction set to WEST");
  }
  else if (command == "STATUS") {
    print_system_status();
  }
  else if (command == "HELP") {
    print_command_help();
  }
  else if (command == "CALIBRATE") {
    calibrate_servos();
  }
  else if (command == "MAVLINK_STATS") {
    print_mavlink_stats();
  }
  else if (command == "OFFSET_RESET") {
    azimuth_offset = 0.0;
    elevation_offset = 0.0;
    Serial.println("[CMD] Azimuth and elevation alignment offsets reset to 0°");
  }
  else if (command.startsWith("AZ_OFFSET")) {
    int index = command.indexOf(' ');
    if (index >= 0) {
      float value = command.substring(index + 1).toFloat();
      azimuth_offset = value;
      Serial.print("[CMD] Azimuth offset set to: ");
      Serial.print(azimuth_offset, 2);
      Serial.println("°");
    } else {
      Serial.println("[CMD] Usage: AZ_OFFSET <value>");
    }
  }
  else if (command.startsWith("EL_OFFSET")) {
    int index = command.indexOf(' ');
    if (index >= 0) {
      float value = command.substring(index + 1).toFloat();
      elevation_offset = value;
      Serial.print("[CMD] Elevation offset set to: ");
      Serial.print(elevation_offset, 2);
      Serial.println("°");
    } else {
      Serial.println("[CMD] Usage: EL_OFFSET <value>");
    }
  }
  else if (command.startsWith("SET LAT")) {
    if (command.length() > 8) {
      double lat = command.substring(8).toDouble();
      local_position.latitude = lat;
      Serial.print("[CMD] Latitude set to: ");
      Serial.println(lat, 6);
    } else {
      Serial.println("[CMD] Usage: SET LAT <value>");
    }
  }
  else if (command.startsWith("SET LON")) {
    if (command.length() > 8) {
      double lon = command.substring(8).toDouble();
      local_position.longitude = lon;
      Serial.print("[CMD] Longitude set to: ");
      Serial.println(lon, 6);
    } else {
      Serial.println("[CMD] Usage: SET LON <value>");
    }
  }
  else if (command.startsWith("SET ALT")) {
    if (command.length() > 8) {
      double alt = command.substring(8).toDouble();
      local_position.altitude = alt;
      Serial.print("[CMD] Altitude set to: ");
      Serial.println(alt, 2);
    } else {
      Serial.println("[CMD] Usage: SET ALT <value>");
    }
  }
  else {
    Serial.println("[CMD] Unknown command. Type 'HELP' for available commands.");
  }
}

void print_command_help() {
  Serial.println("\n========== ANTENNA TRACKER COMMANDS ==========");
  Serial.println("N                  - Set direction to NORTH");
  Serial.println("S                  - Set direction to SOUTH");
  Serial.println("E                  - Set direction to EAST");
  Serial.println("W                  - Set direction to WEST");
  Serial.println("STATUS             - Print current system status");
  Serial.println("HELP               - Print this help message");
  Serial.println("CALIBRATE          - Run servo calibration");
  Serial.println("MAVLINK_STATS      - Print MAVLink debug statistics");
  Serial.println("AZ_OFFSET <value>  - Set azimuth trim offset in degrees");
  Serial.println("EL_OFFSET <value>  - Set elevation trim offset in degrees");
  Serial.println("OFFSET_RESET       - Reset both trim offsets to zero");
  Serial.println("SET LAT <value>    - Manually set local latitude");
  Serial.println("SET LON <value>    - Manually set local longitude");
  Serial.println("SET ALT <value>    - Manually set local altitude (meters)");
  Serial.println("=============================================\n");
}

void print_system_status() {
  Serial.println("\n========== SYSTEM STATUS ==========");
  
  Serial.println("--- LOCAL POSITION (GPS) ---");
  Serial.print("Status:    ");
  if (gps_state.locked) {
    Serial.println("🔒 LOCKED");
  } else if (gps_state.searching) {
    Serial.println("🔍 SEARCHING");
  } else {
    Serial.println("❌ UNLOCKED");
  }
  Serial.print("Latitude:  ");
  Serial.println(local_position.latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(local_position.longitude, 6);
  Serial.print("Altitude:  ");
  Serial.print(local_position.altitude);
  Serial.println(" m");
  Serial.print("HDOP:      ");
  Serial.println(local_position.hdop, 2);
  Serial.print("Direction: ");
  Serial.println(local_position.direction);
  Serial.print("GPS Fix:   ");
  Serial.println(local_position.fix ? "✓ Yes" : "✗ No");

  Serial.println("\n--- TRACKING TRIM ---");
  Serial.print("Azimuth Offset:   ");
  Serial.print(azimuth_offset, 2);
  Serial.println(" °");
  Serial.print("Elevation Offset: ");
  Serial.print(elevation_offset, 2);
  Serial.println(" °");
  
  Serial.println("\n--- UAV POSITION ---");
  Serial.print("Latitude:  ");
  Serial.println(uav_position.latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(uav_position.longitude, 6);
  Serial.print("Altitude:  ");
  Serial.print(uav_position.altitude);
  Serial.println(" m");
  Serial.print("Valid:     ");
  Serial.println(is_mavlink_data_fresh() ? "✓ Yes (fresh)" : "✗ No (stale or missing)");
  if (uav_position.position_valid) {
    unsigned long age = millis() - uav_position.last_update;
    Serial.print("Data Age:  ");
    Serial.print(age / 1000);
    Serial.print(" seconds (timeout: ");
    Serial.print(MAVLINK_TIMEOUT_MS / 1000);
    Serial.println("s)");
  }
  
  Serial.println("\n--- TRACKING ANGLES ---");
  Serial.print("Azimuth:   ");
  Serial.print(tracking_angles.azimuth);
  Serial.println(" °");
  Serial.print("Elevation: ");
  Serial.print(tracking_angles.elevation);
  Serial.println(" °");
  Serial.print("Valid:     ");
  Serial.println(tracking_angles.valid ? "✓ Yes" : "✗ No");
  
  Serial.println("\n--- NETWORK ---");
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "✓ Connected" : "✗ Disconnected");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP Address:  ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal:      ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
  
  Serial.println("\n===================================\n");
}

void print_mavlink_stats() {
  Serial.println("\n========== MAVLINK STATISTICS ==========");
  Serial.print("Total UDP Packets:     ");
  Serial.println(mavlink_stats.total_packets_received);
  Serial.print("Successful Decodes:    ");
  Serial.println(mavlink_stats.successful_decodes);
  Serial.print("Failed Decodes:        ");
  Serial.println(mavlink_stats.failed_decodes);
  Serial.print("Last Message ID:       ");
  Serial.println(mavlink_stats.last_message_id);
  Serial.println("=========================================\n");
}

void calibrate_servos() {
  Serial.println("\n[CALIBRATE] Starting servo calibration...");
  Serial.println("[CALIBRATE] Setting azimuth to minimum (0°)");
  azimuthServo.write(0);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting azimuth to maximum (180°)");
  azimuthServo.write(180);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting azimuth to center (90°)");
  azimuthServo.write(90);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting elevation to maximum (90°)");
  elevationServo.write(gearRatio * 90);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting elevation to minimum (0°)");
  elevationServo.write(0);
  delay(2000);
  
  Serial.println("[CALIBRATE] ✓ Calibration complete\n");
}
