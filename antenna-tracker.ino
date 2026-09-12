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

// GPIO Pins for Servos
const int AZIMUTH_SERVO_PIN = 13;   // Servo for horizontal rotation (East/West)
const int ELEVATION_SERVO_PIN = 14; // Servo for vertical rotation (Up/Down)

// GPIO Pins for GPS (Serial2)
const int GPS_RX_PIN = 11;
const int GPS_TX_PIN = 12;

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

// Local tracker position
struct {
  double latitude;
  double longitude;
  double altitude;
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
  Serial.begin(115200);
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
  
  // Initialize position data
  local_position.latitude = 0;
  local_position.longitude = 0;
  local_position.altitude = 0;
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
  
  Serial.println("[SYSTEM] Initialization complete\n");
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
  // Read available GPS data
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);
  }
  
  // Check if we have a new valid position with 3D fix
  if (gps.location.isUpdated() && gps.location.isValid()) {
    local_position.latitude = gps.location.lat();
    local_position.longitude = gps.location.lng();
    local_position.altitude = gps.altitude.meters();
    local_position.fix = true;
    
    // Only print once when 3D fix is achieved
    if (!local_position.fix_announced) {
      local_position.fix_announced = true;
      Serial.println("\n[GPS] ✓ 3D Fix Achieved - Position Valid:");
      Serial.print("  Lat: ");
      Serial.println(local_position.latitude, 6);
      Serial.print("  Lon: ");
      Serial.println(local_position.longitude, 6);
      Serial.print("  Alt: ");
      Serial.print(local_position.altitude);
      Serial.println(" m");
      Serial.print("  Direction: ");
      Serial.println(local_position.direction);
    }
  } else if (!gps.location.isValid() && local_position.fix_announced) {
    // Lost GPS fix
    local_position.fix = false;
    local_position.fix_announced = false;
    Serial.println("[GPS] ✗ Lost GPS fix");
  }
  
  // Check GPS signal strength (print every update)
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
    
    Serial.print("[MAVLink] ✓ UDP packet received (");
    Serial.print(len);
    Serial.print(" bytes) - Decoding... ");
    
    int decode_count = 0;
    for (int i = 0; i < len; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        decode_count++;
        mavlink_stats.successful_decodes++;
        mavlink_stats.last_message_id = msg.msgid;
        Serial.print("[MSG_ID: ");
        Serial.print(msg.msgid);
        Serial.print("] ");
        process_mavlink_message(&msg);
      }
    }
    
    if (decode_count == 0) {
      mavlink_stats.failed_decodes++;
      Serial.println("[NO VALID MESSAGE DECODED]");
    } else {
      Serial.print("(");
      Serial.print(decode_count);
      Serial.println(" message(s) decoded)");
    }
  }
}

void process_mavlink_message(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      Serial.println("[MAVLink] Processing GLOBAL_POSITION_INT...");
      handle_global_position_int(msg);
      break;
      
    case MAVLINK_MSG_ID_ATTITUDE:
      Serial.println("[MAVLink] Processing ATTITUDE...");
      handle_attitude(msg);
      break;
      
    case MAVLINK_MSG_ID_HEARTBEAT:
      Serial.println("[MAVLink] ✓ Heartbeat received");
      break;
      
    default:
      Serial.print("[MAVLink] Received message ID: ");
      Serial.print(msg->msgid);
      Serial.println(" (no handler)");
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
  
  Serial.println("[MAVLink] ✓ UAV Position Updated:");
  Serial.print("  Lat: ");
  Serial.println(uav_position.latitude, 6);
  Serial.print("  Lon: ");
  Serial.println(uav_position.longitude, 6);
  Serial.print("  Alt: ");
  Serial.print(uav_position.altitude);
  Serial.println(" m");
}

void handle_attitude(mavlink_message_t* msg) {
  mavlink_attitude_t attitude;
  mavlink_msg_attitude_decode(msg, &attitude);
  
  Serial.println("[MAVLink] UAV Attitude:");
  Serial.print("  Roll: ");
  Serial.print(attitude.roll * 57.2958); // Convert rad to degrees
  Serial.println(" °");
  Serial.print("  Pitch: ");
  Serial.print(attitude.pitch * 57.2958);
  Serial.println(" °");
  Serial.print("  Yaw: ");
  Serial.print(attitude.yaw * 57.2958);
  Serial.println(" °");
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
  
  tracking_angles.azimuth = bearing;
  tracking_angles.elevation = elevation;
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
  
  // Convert azimuth (0-360) to servo angle (0-180)
  // Map: 0° = left, 90° = center, 180° = right
  int azimuth_angle = map(tracking_angles.azimuth, 0, 180, 0, 180);
  
  // Convert elevation (0-90) to servo angle (0-90)
  int elevation_angle = constrain(tracking_angles.elevation, 0, 90);
  
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
  Serial.println("SET LAT <value>    - Manually set local latitude");
  Serial.println("SET LON <value>    - Manually set local longitude");
  Serial.println("SET ALT <value>    - Manually set local altitude (meters)");
  Serial.println("=============================================\n");
}

void print_system_status() {
  Serial.println("\n========== SYSTEM STATUS ==========");
  
  Serial.println("--- LOCAL POSITION ---");
  Serial.print("Latitude:  ");
  Serial.println(local_position.latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(local_position.longitude, 6);
  Serial.print("Altitude:  ");
  Serial.print(local_position.altitude);
  Serial.println(" m");
  Serial.print("Direction: ");
  Serial.println(local_position.direction);
  Serial.print("GPS Fix:   ");
  Serial.println(local_position.fix ? "✓ Yes" : "✗ No");
  
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
  elevationServo.write(90);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting elevation to minimum (0°)");
  elevationServo.write(0);
  delay(2000);
  
  Serial.println("[CALIBRATE] ✓ Calibration complete\n");
}
