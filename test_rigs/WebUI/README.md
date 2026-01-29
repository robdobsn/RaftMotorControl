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

The UI sends REST commands to the firmware via WebSocket. The commands are routed through MainSysMod to the MotorControl device.

### Command Format

Commands use URL query parameter format that gets converted to JSON:

```
motors?cmd=<command>&param1=value1&param2=value2...
```

### Supported Commands

#### Motion Command
```
motors?cmd=motion&mode=<mode>&speed=<speed>&pos=[<pos0>,<pos1>]&nosplit=1
```
- **mode**: `abs` (absolute), `rel` (relative), `pos-abs-steps` (absolute steps), `pos-rel-steps` (relative steps)
- **speed**: Speed in units per minute (e.g., `3000upm`) or percent of max (e.g., `80`)
- **pos**: Position array for axes (use `null` to skip an axis), e.g. `[100,200]` or `[100,null]`
- **nosplit**: `1` to disable path splitting, `0` to allow splitting
- **imm**: `1` to stop and clear queue first, `0` for normal queuing

Examples:
```
motors?cmd=motion&mode=abs&speed=3000upm&pos=[100,200]&nosplit=1
motors?cmd=motion&mode=pos-rel-steps&speed=1200upm&pos=[200,-200]&nosplit=1
```

#### Stop Command
```
motors?cmd=stop&disableMotors=<true|false>
```
- **disableMotors**: `true` to disable motors after stopping, `false` to keep enabled

Example:
```
motors?cmd=stop&disableMotors=false
```

#### Set Origin Command
```
motors?cmd=setOrigin
```
Sets current position as origin (0, 0).

#### Homing Pattern Command
```
motors?cmd=startPattern&pattern=homing&forMs=30000
```
- **pattern**: Pattern name (e.g., `homing`)
- **forMs**: Maximum runtime in milliseconds

#### Stop Pattern Command
```
motors?cmd=stopPattern
```
Stops currently running motion pattern.

#### Max Current Command
```
motors?cmd=maxCurrent&axisIdx=0&maxCurrentA=1.5
```
- **axisIdx**: Axis index (0 or 1)
- **maxCurrentA**: Maximum current in amps

#### Motor Off Timer Command
```
motors?cmd=offAfter&offAfterS=10
```
- **offAfterS**: Time in seconds before motors turn off after movement

### Command Processing Flow

1. WebUI sends REST command via WebSocket (e.g., `motors?cmd=motion&mode=abs&speed=3000upm&pos=[100,200]`)
2. WebServer receives and forwards to ProtocolExchange
3. ProtocolExchange decodes and routes to RestAPIEndpointManager
4. RestAPIEndpointManager matches "motors" endpoint to MainSysMod
5. MainSysMod converts REST parameters to JSON using `getJSONFromRESTRequest()`
6. MainSysMod sends JSON to MotorControl device via DeviceManager
7. MotorControl executes the command

See [WebSocket_to_MotorControl_Communication.md](../../docs/WebSocket_to_MotorControl_Communication.md) for detailed architecture.

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
