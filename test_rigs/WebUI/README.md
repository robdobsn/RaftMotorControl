# TwoStepperMagRotTestRig WebUI

Lightweight React-based web interface for controlling and monitoring the two-stepper motor test rig with magnetic rotary encoders (MT6701/AS5600).

## Features

- **WebSocket Connection**: Real-time communication with ESP32 firmware
- **Motor Control**: Independent control of two stepper motors (speed, direction, position)
- **Encoder Display**: Real-time angle monitoring from MT6701 and AS5600 magnetic encoders
- **System Status**: Display firmware version, MAC address, and system information
- **Lightweight**: Optimized for ESP32 flash storage (~200KB gzipped)

## Development

### Prerequisites

- Node.js 18+ and npm

### Install Dependencies

```bash
npm install
```

### Run Development Server

```bash
npm start
```

This will start the Parcel dev server at `http://localhost:1234` with hot reload.

### Build for Production

```bash
npm run build
```

The optimized output will be in the `dist/` folder. The Raft build system will automatically:
1. Run `npm install` and `npm run build`
2. Gzip compress all HTML, CSS, and JS files
3. Copy files to the ESP32 filesystem image

To manually rebuild firmware with WebUI:

```bash
# In the test rig directory
raft b -i
```

## Usage

1. **Connect**: Enter the ESP32 IP address (e.g., `192.168.1.100`) and click Connect
2. **Monitor Encoders**: View real-time angle measurements from magnetic encoders
3. **Control Motors**:
   - Adjust speed with sliders
   - Use CW/CCW buttons for continuous rotation
   - Enter position and click "Go" for precise positioning
   - Use "Home" to return to home position
4. **View Status**: Check system information and connection status

## API Commands

The UI sends REST commands to the firmware via WebSocket:

- `motor/1/move?steps=200&speed=50` - Move motor 1
- `motor/1/goto?pos=1000&speed=75` - Go to position
- `motor/1/stop` - Emergency stop
- `motor/1/home` - Home motor

Adjust these commands in `MotorControl.tsx` to match your firmware API.

## Build Optimization

- Parcel bundler with tree-shaking
- Terser JS minification
- CSS minification (cssnano)
- Gzip compression (handled by Raft build system)
- Bootstrap via CDN (no bundle bloat)
- Single bundle (no code splitting)

## Architecture

- **ConnManager**: Singleton managing WebSocket connection via raftjs
- **SystemTypeTwoStepper**: Custom system type for device subscription
- **Components**: Modular React components for UI sections
- **TypeScript**: Type-safe development

## License

MIT
