# ESP32 Reflow Oven Controller

A PID-controlled reflow oven controller built with ESP32, supporting lead-free solder profiles with cycle burst SSR control.

## Features

- Zero-crossing SSR control using cycle burst method
- MAX6675 thermocouple interface for accurate temperature measurement
- 0.96" OLED display (128x64) for real-time status
- PID temperature control with profile stages
- Serial command interface
- Support for lead-free solder profiles (SAC305)
- Safety features including maximum temperature cutoff

## Hardware Requirements

- ESP32 development board
- SSR-25DA Solid State Relay
- MAX6675 Thermocouple Module
- K-type Thermocouple
- 0.96" OLED Display (I2C, SSD1306)
- 24V Power Supply (for the heating element)
- Toaster Oven (recommended >1200W)

## Pin Connections

### OLED Display (Onboard I2C)
- SDA → GPIO12 (D6)
- SCL → GPIO14 (D5)
- VCC → 3.3V
- GND → GND

### MAX6675 Thermocouple (SPI)
- SO (MISO) → GPIO19
- CS → GPIO5
- SCK → GPIO18
- VCC → 3.3V
- GND → GND

### SSR Control
- Control Pin → GPIO16 (D0)
- SSR Power → 230V AC
- Load → AC Mains to Heating Element

## Reflow Profile Parameters

The controller implements a standard lead-free (SAC305) reflow profile:

1. **Preheat Stage**
   - Ramp Rate: 2.0°C/s
   - Target: 150°C

2. **Soak Stage**
   - Temperature: 165°C
   - Duration: 60s

3. **Reflow Stage**
   - Ramp Rate: 2.0°C/s
   - Peak Temperature: 245°C
   - Time Above Liquidus (217°C): 60s

4. **Cooling Stage**
   - Cooling Rate: 3.0°C/s
   - Safe Handling Temperature: 100°C

## Building and Flashing

This project uses PlatformIO for development. To build and flash:

1. Install PlatformIO IDE
2. Clone this repository
3. Open the project in PlatformIO
4. Build and upload to your ESP32

```bash
First, clean the file system:
pio run -t erase
-Then upload the filesystem:
pio run -t uploadfs
pio run -t upload
```

## Serial Commands

The controller accepts the following commands via serial (115200 baud):

- `start` - Start the reflow process
- `stop` - Stop the reflow process
- `status` - Show current temperature and status
- `h` - Toggle heating on/off

## Safety Features

- Maximum temperature cutoff (300°C)
- Sensor disconnection detection
- Cycle burst control for SSR longevity
- Minimum ON/OFF times to prevent rapid switching

## Control Method

The system uses a PID controller with cycle burst output instead of traditional PWM to better work with zero-crossing SSRs:

- Cycle Period: 1000ms (1 second)
- Minimum ON/OFF Time: 100ms
- PID Output mapped to duty cycle within each period
- Zero-crossing compatible operation

## Contributing

Feel free to submit issues and pull requests.

## License

This project is open source and available under the MIT License.

## Acknowledgments

- Uses Adafruit libraries for display and sensor interfaces
- PID library by Brett Beauregard
- MAX6675 library by Adafruit 

## Feature Requests

1. **External Display Interface**: Implement an external display interface to replace the damaged onboard display.
2. **Dynamic Wi-Fi Connection**: Create a dynamic Wi-Fi connection system that allows users to connect to any Wi-Fi network instead of using a static SSID and password.
3. **On-Screen Keyboard**: Develop an on-screen keyboard for users to select Wi-Fi SSID and enter passwords, designed for a 128x64 pixel AMOLED display. 