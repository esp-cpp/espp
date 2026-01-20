# WiFi Provisioning Example

Web-based WiFi provisioning system with temporary access point and mobile-friendly interface.

## How to Use

### Hardware Setup

Any ESP32 module with WiFi capability. Optionally connect:
- Reset button (GPIO configurable in menuconfig)
- Status LED (GPIO configurable in menuconfig)

### Configure the Project

```bash
idf.py menuconfig
```

Navigate to `WiFi Provisioning Example Configuration`:
- Set AP SSID and password for provisioning
- Configure AP channel and max connections
- Set web server port (default: 80)
- Configure timeout (0 for no timeout)
- Enable auto-generate unique SSID (appends MAC address)
- Set reset button GPIO (or -1 to disable)
- Set status LED GPIO (or -1 to disable)
- Enable/disable credential validation
- Configure NVS namespace

### Build and Flash

```bash
idf.py build flash monitor
```

## Usage Flow

### First Boot (No Credentials Stored)

1. Device creates temporary WiFi access point
2. Connect to the AP with your phone/computer:
   - SSID: `ESP32-Setup` (or your configured name)
   - Password: `configure` (or your configured password)
3. Open browser to `http://192.168.4.1` (or configured IP:port)
4. Web interface shows:
   - List of available WiFi networks (scanned)
   - Manual SSID entry option
   - Password field
5. Select network and enter password
6. Click "Connect"
7. Device validates credentials (if enabled)
8. Credentials saved to NVS
9. Device restarts and connects to WiFi

### Subsequent Boots

Device automatically connects using stored credentials. If connection fails, provisioning mode restarts.

### Reset Credentials

- Press reset button (if configured) to clear stored credentials
- Or manually erase NVS partition
- Device will enter provisioning mode on next boot

## Example Output

```
I (380) Provisioning Example: Starting WiFi Provisioning Example
I (385) Provisioning Example: Starting WiFi provisioning...
I (561) Provisioning Example: Provisioning started!
I (562) Provisioning Example: Connect to WiFi: ESP32-Setup-AB12CD
I (563) Provisioning Example: Password: configure
I (564) Provisioning Example: Open browser to: http://192.168.4.1:80
```

After provisioning:
```
I (380) Provisioning Example: Found stored credentials for SSID: MyHomeWiFi
I (385) Provisioning Example: Attempting to connect to stored WiFi network...
I (2450) Provisioning Example: Successfully connected to WiFi!
I (2451) Provisioning Example: IP Address: 192.168.1.105
```

## Features

- **Mobile-friendly UI**: Responsive design works on phones and tablets
- **Network scanning**: Shows available WiFi networks with signal strength
- **Credential validation**: Tests connection before saving (configurable)
- **Unique SSID generation**: Appends MAC address to prevent conflicts
- **Status LED**: Visual feedback during provisioning (optional)
- **Reset button**: Clear credentials without reflashing (optional)
- **Timeout protection**: Automatically exits provisioning after timeout
- **NVS storage**: Persistent credential storage
- **Secure**: Credentials never displayed in logs

## Customization

All settings are configurable via menuconfig without modifying code. The provisioning component can be easily integrated into your own application.

## Troubleshooting

- **Can't see AP**: Check channel (some devices don't support channels 12-13)
- **Can't connect to AP**: Verify password is at least 8 characters (or leave empty for open network)
- **Stored credentials don't work**: Enable validation to test before saving
- **Need to reset**: Use reset button or manually erase NVS
