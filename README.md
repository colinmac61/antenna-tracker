# Antenna Tracker with MAVLink 2 Support

A GPS-guided antenna tracker for ESP32 that automatically points a directional antenna at a UAV using MAVLink 2 telemetry via ELRS Backpack WiFi.

## Features

- **Manual Direction Setup**: Point antenna North, South, East, or West
- **Local GPS Positioning**: Uses local GPS module for tracker location (Lat/Lon/Alt)
- **MAVLink 2 Telemetry**: Receives UAV position via ELRS Backpack WIFI
- **Dual-Axis Servo Control**: Independent azimuth (horizontal) and elevation (vertical) positioning
- **Real-time Angle Calculation**: Computes azimuth and elevation to target UAV
- **Serial Command Interface**: Control and monitor via Serial (115200 baud)
- **Comprehensive Debug Output**: Serial print statements for troubleshooting

## Hardware Requirements

### Microcontroller
- **ESP32** Development Board
  - WiFi connectivity for ELRS Backpack
  - GPIO 18, 19 for servo PWM
  - GPIO 16, 17 for GPS serial (UART2)
  
### GPS Module
- Any NMEA-compatible GPS (9600 baud)
- Examples: u-blox NEO-6M, SIM68M, or similar
- Provides: Latitude, Longitude, Altitude, Satellite count

### Servo Motors (x2)
- Standard 5V RC servos (SG90, MG90S, or higher torque)
- **Azimuth**: Horizontal rotation (East/West pointing)
- **Elevation**: Vertical rotation (Up/Down angle)

### Connectivity
- **ELRS Backpack** with WiFi enabled
- **UAV** with flight controller running MAVLink protocol

### Power Supply
- 5V supply capable of 2A+ (servos + ESP32)
- Consider separate servo power supply for stability

## Wiring Diagram

```
ESP32                 GPS Module (9600 baud)
├─ GPIO 16 (RX2) ────→ TX
├─ GPIO 17 (TX2) ────→ RX
├─ GND ──────────────→ GND
└─ 5V ──────────────→ 5V

ESP32                 Azimuth Servo
├─ GPIO 18 (PWM) ────→ Signal (Orange)
├─ GND ──────────────→ GND (Brown)
└─ 5V ──────────────→ 5V (Red)

ESP32                 Elevation Servo
├─ GPIO 19 (PWM) ────→ Signal (Orange)
├─ GND ──────────────→ GND (Brown)
└─ 5V ──────────────→ 5V (Red)

WiFi                  ELRS Backpack
└─ 2.4 GHz ──────────→ MAVLink Telemetry (UDP port 14550)
```

## Software Installation

### 1. Arduino IDE Setup
- Download: https://www.arduino.cc/en/software
- Install version 1.8.19 or later

### 2. Add ESP32 Board Support
1. **File** → **Preferences**
2. Paste in "Additional Boards Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools** → **Board Manager** → Search "esp32" → Install "ESP32 by Espressif Systems"

### 3. Install Libraries
**Sketch** → **Include Library** → **Manage Libraries**:

1. **TinyGPS++** (Mikal Hart)
   - Handles GPS NMEA parsing
   
2. **MAVLink** (ArduPilot)
   - Download headers from: https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink/
   - Place in `Arduino/libraries/` folder

### 4. Configure Settings
Edit the main sketch:

```cpp
// WiFi
const char* ssid = "YOUR_ELRS_SSID";
const char* password = "YOUR_PASSWORD";

// GPIO Pins
const int AZIMUTH_SERVO_PIN = 18;
const int ELEVATION_SERVO_PIN = 19;
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 17;

// Servo Calibration (adjust for your servos)
const int AZIMUTH_MIN_US = 1000;   // 270° West
const int AZIMUTH_MAX_US = 2000;   // 90° East
const int ELEVATION_MIN_US = 1000; // 0° Horizontal
const int ELEVATION_MAX_US = 2000; // 90° Vertical
```

### 5. Upload
1. Connect ESP32 via USB
2. **Tools** → **Board** → **ESP32 Dev Module**
3. **Tools** → **Port** → Select COM port
4. **Sketch** → **Upload**
5. **Tools** → **Serial Monitor** (115200 baud)

## Quick Start

### Step 1: Verify GPS
```
Look for: [GPS] ✓ Position Updated
         [GPS] Satellites: 8+
```

### Step 2: Connect to ELRS
```
Look for: [WiFi] ✓ Connected successfully
         [WiFi] IP Address: 192.168.x.x
         [UDP] ✓ Listening on port 14550
```

### Step 3: Set Antenna Direction
Send via Serial Monitor:
```
N    // Point North
E    // Point East
S    // Point South
W    // Point West
```

### Step 4: Calibrate Servos
```
CALIBRATE
```
Observes servo movements. Adjust pulse widths if needed:
- Min/max should match your servo's physical limits
- Typical range: 1000-2000 microseconds

### Step 5: Start UAV
- Power on flight controller
- Look for: `[MAVLink] ✓ UAV Position Updated`
- Servos should begin tracking

### Monitor Status
```
STATUS
```
Displays complete system status including positions and angles.

## Serial Commands

| Command | Purpose |
|---------|----------|
| `N` | Set antenna pointing North |
| `S` | Set antenna pointing South |
| `E` | Set antenna pointing East |
| `W` | Set antenna pointing West |
| `STATUS` | Print system status |
| `HELP` | List all commands |
| `CALIBRATE` | Run servo calibration |
| `SET LAT <value>` | Override local latitude |
| `SET LON <value>` | Override local longitude |
| `SET ALT <value>` | Override local altitude (meters) |

## Troubleshooting

### GPS Issues
- **No satellites**: Move antenna outdoors, clear sky view needed
- **No position**: Wait 30-60 seconds for cold start
- **Check pins**: GPIO 16 (RX), GPIO 17 (TX)
- **Verify baud**: GPS typically 9600 baud

### WiFi Issues  
- **Cannot connect**: Verify SSID/password
- **Wrong IP range**: Check ELRS backpack IP subnet
- **UDP no data**: Confirm ELRS is forwarding MAVLink

### Servo Issues
- **No movement**: Check GPIO 18/19 connections
- **Limited range**: Adjust MIN/MAX microsecond values
- **Jitter**: Increase main loop delay or check power supply
- **Test**: Send `CALIBRATE` command

### MAVLink Issues
- **No telemetry**: Verify UDP port 14550 correct
- **Data old**: Check flight controller MAVLink output enabled
- **Message errors**: Confirm MAVLink 2 protocol selected

## Advanced Configuration

### Angle Adjustment
If tracking angles seem off:
1. Verify GPS accuracy (STATUS → GPS fix quality)
2. Check antenna direction setting (N/S/E/W correct?)
3. Confirm servo calibration (CALIBRATE command)
4. Test with known UAV position

### Servo Speed
Adjust update interval in main code:
```cpp
delay(100);  // Faster updates = quicker tracking
delay(500);  // Slower updates = smoother motion
```

### Range Extension
- Higher altitude UAV = better tracking accuracy
- More satellites for GPS = better position
- RTK-GPS can improve accuracy to cm-level

## Technical Details

### Coordinate System
- **Azimuth**: 0°=North, 90°=East, 180°=South, 270°=West
- **Elevation**: 0°=Horizontal, 90°=Vertical (zenith)

### Calculations
- Uses Haversine formula for great-circle distance
- Bearing calculated from tracker → UAV position
- Elevation based on altitude difference and distance

### Update Rates
- GPS: ~1 Hz (typical)
- MAVLink: 10-50 Hz (flight controller dependent)
- Servo: 10 Hz (100ms update interval)

## Performance Tips

- Use RTK-GPS for cm-level accuracy on long-range tracks
- Mount GPS antenna on roof/pole for better signal
- Servo power supply should be stable and independent
- Increase servo update rate for responsive tracking (lower latency)
- Test with UAV at known positions first

## Future Enhancements

- [ ] CRSF telemetry protocol support
- [ ] SD card flight logging
- [ ] Web dashboard
- [ ] Position prediction
- [ ] Multiple target tracking
- [ ] Motor encoders for feedback
- [ ] Battery monitoring

## License

Educational/hobbyist use. See repository for details.

---

**Questions?** Check Serial Monitor debug output (prefixed with `[...]` tags).