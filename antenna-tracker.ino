#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>
#include <MAVLink.h>

// ==================== CONFIGURATION ====================
// WiFi Configuration for ELRS Backpack
const char* ssid = "ExpressLRS TX Backpack ABFA86";        // Change to your ELRS backpack SSID
const char* password = "expresslrs";          // Change to your ELRS backpack password
const int udp_port = 14550;                      // MAVLink UDP port

// GPIO Pins for Servos
const int AZIMUTH_SERVO_PIN = 18;   // Servo for horizontal rotation (East/West)
const int ELEVATION_SERVO_PIN = 19; // Servo for vertical rotation (Up/Down)

// GPIO Pins for GPS (Serial2)
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 17;

// Servo Calibration (adjust based on your servos)
const int AZIMUTH_MIN_US = 1000;    // Microseconds for min position (270 degrees West)
const int AZIMUTH_MAX_US = 2000;    // Microseconds for max position (90 degrees East)
const int ELEVATION_MIN_US = 1000;  // Microseconds for min position (0 degrees Down)
const int ELEVATION_MAX_US = 2000;  // Microseconds for max position (90 degrees Up)

// ==================== GLOBAL VARIABLES ====================
WiFiUDP udp;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

// Local tracker position
struct {
  double latitude;
  double longitude;
  double altitude;
  char direction;  // 'N', 'S', 'E', 'W'
  boolean fix;
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

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== ANTENNA TRACKER STARTUP ===");
  Serial.println("Initializing systems...");
  
  // Initialize GPS Serial
  SerialGPS.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[GPS] Serial initialized at 115200 baud");
  
  // Initialize servo pins
  pinMode(AZIMUTH_SERVO_PIN, OUTPUT);
  pinMode(ELEVATION_SERVO_PIN, OUTPUT);
  Serial.println("[SERVO] Servo pins initialized");
  
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
  
  uav_position.latitude = 0;
  uav_position.longitude = 0;
  uav_position.altitude = 0;
  uav_position.position_valid = false;
  uav_position.last_update = 0;
  
  tracking_angles.azimuth = 0;
  tracking_angles.elevation = 0;
  tracking_angles.valid = false;
  
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
  
  // Calculate tracking angles if we have valid data
  if (local_position.fix && uav_position.position_valid) {
    calculate_tracking_angles();
    update_servo_positions();
  } else {
    if (!local_position.fix) {
      Serial.println("[WARNING] Waiting for GPS fix...");
    }
    if (!uav_position.position_valid) {
      Serial.println("[WARNING] Waiting for UAV telemetry...");
    }
    delay(1000);
  }
  
  delay(100);  // Main loop delay
}

// ==================== GPS FUNCTIONS ====================
void update_local_gps() {
  // Read available GPS data
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);
  }
  
  // Check if we have a new valid position
  if (gps.location.isUpdated()) {
    local_position.latitude = gps.location.lat();
    local_position.longitude = gps.location.lng();
    local_position.altitude = gps.altitude.meters();
    local_position.fix = gps.location.isValid();
    
    Serial.println("\n[GPS] ✓ Position Updated:");
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
  
  // Check GPS signal strength
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
    
    Serial.print("[MAVLink] Received packet, length: ");
    Serial.println(len);
    
    for (int i = 0; i < len; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        process_mavlink_message(&msg);
      }
    }
  }
}

void process_mavlink_message(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      handle_global_position_int(msg);
      break;
      
    case MAVLINK_MSG_ID_ATTITUDE:
      handle_attitude(msg);
      break;
      
    case MAVLINK_MSG_ID_HEARTBEAT:
      Serial.println("[MAVLink] ✓ Heartbeat received");
      break;
      
    default:
      Serial.print("[MAVLink] Message ID received: ");
      Serial.println(msg->msgid);
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
  
  // Convert azimuth (0-360) to servo microseconds
  // Map: 0° = left (AZIMUTH_MIN), 90° = center, 180° = right (AZIMUTH_MAX)
  int azimuth_us = map_angle_to_servo(tracking_angles.azimuth, 0, 180, AZIMUTH_MIN_US, AZIMUTH_MAX_US);
  
  // Convert elevation (0-90) to servo microseconds
  int elevation_us = map_angle_to_servo(tracking_angles.elevation, 0, 90, ELEVATION_MIN_US, ELEVATION_MAX_US);
  
  // Send PWM signals
  digitalWrite(AZIMUTH_SERVO_PIN, HIGH);
  delayMicroseconds(azimuth_us);
  digitalWrite(AZIMUTH_SERVO_PIN, LOW);
  
  digitalWrite(ELEVATION_SERVO_PIN, HIGH);
  delayMicroseconds(elevation_us);
  digitalWrite(ELEVATION_SERVO_PIN, LOW);
  
  Serial.print("[SERVO] Azimuth: ");
  Serial.print(azimuth_us);
  Serial.print(" µs, Elevation: ");
  Serial.print(elevation_us);
  Serial.println(" µs");
}

int map_angle_to_servo(double angle, double min_angle, double max_angle, int min_us, int max_us) {
  angle = constrain(angle, min_angle, max_angle);
  return map(angle * 100, min_angle * 100, max_angle * 100, min_us, max_us);
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
  else if (command.startsWith("SET LAT")) {
    double lat = command.substring(8).toDouble();
    local_position.latitude = lat;
    Serial.print("[CMD] Latitude set to: ");
    Serial.println(lat, 6);
  }
  else if (command.startsWith("SET LON")) {
    double lon = command.substring(8).toDouble();
    local_position.longitude = lon;
    Serial.print("[CMD] Longitude set to: ");
    Serial.println(lon, 6);
  }
  else if (command.startsWith("SET ALT")) {
    double alt = command.substring(8).toDouble();
    local_position.altitude = alt;
    Serial.print("[CMD] Altitude set to: ");
    Serial.println(alt, 2);
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
  Serial.println(uav_position.position_valid ? "✓ Yes" : "✗ No");
  unsigned long age = millis() - uav_position.last_update;
  Serial.print("Data Age:  ");
  Serial.print(age / 1000);
  Serial.println(" seconds");
  
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

void calibrate_servos() {
  Serial.println("\n[CALIBRATE] Starting servo calibration...");
  Serial.println("[CALIBRATE] Setting azimuth to minimum (West)");
  digitalWrite(AZIMUTH_SERVO_PIN, HIGH);
  delayMicroseconds(AZIMUTH_MIN_US);
  digitalWrite(AZIMUTH_SERVO_PIN, LOW);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting azimuth to maximum (East)");
  digitalWrite(AZIMUTH_SERVO_PIN, HIGH);
  delayMicroseconds(AZIMUTH_MAX_US);
  digitalWrite(AZIMUTH_SERVO_PIN, LOW);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting azimuth to center (North)");
  digitalWrite(AZIMUTH_SERVO_PIN, HIGH);
  delayMicroseconds((AZIMUTH_MIN_US + AZIMUTH_MAX_US) / 2);
  digitalWrite(AZIMUTH_SERVO_PIN, LOW);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting elevation to minimum (Down)");
  digitalWrite(ELEVATION_SERVO_PIN, HIGH);
  delayMicroseconds(ELEVATION_MIN_US);
  digitalWrite(ELEVATION_SERVO_PIN, LOW);
  delay(2000);
  
  Serial.println("[CALIBRATE] Setting elevation to maximum (Up)");
  digitalWrite(ELEVATION_SERVO_PIN, HIGH);
  delayMicroseconds(ELEVATION_MAX_US);
  digitalWrite(ELEVATION_SERVO_PIN, LOW);
  delay(2000);
  
  Serial.println("[CALIBRATE] ✓ Calibration complete\n");
}
