import React, { useState, useEffect } from 'react';
import ConnectionPanel from './components/ConnectionPanel';
import MotorControl from './components/MotorControl';
import PathExecutor from './components/PathExecutor';
import EncoderDisplay from './components/EncoderDisplay';
import AngleChart from './components/AngleChart';
import RobotVisualization from './components/RobotVisualization';
import ConnManager from './ConnManager';
import { RaftConnEvent, RaftUpdateEvent, RaftPublishEvent, RaftLog } from '@robdobsn/raftjs';
import { RaftLogLevel } from '@robdobsn/raftjs/dist/web/RaftLog';
import { getRobotGeometry } from './utils/RobotGeometry';

const connManager = ConnManager.getInstance();
const robotGeometry = getRobotGeometry();

export interface AxisConfig {
  name: string;
  unitsPerRot: number;
  stepsPerRot: number;
  maxSpeedUps: number;
  maxAccUps2: number;
  gearFactor: number;
}

export interface RobotConfig {
  geometry: string;
  arm1LengthMM: number;
  arm2LengthMM: number;
  maxRadiusMM: number;
  originTheta2OffsetDegrees: number;
  axes?: AxisConfig[];
}

// Expected path point for visualization
export interface ExpectedPathPoint {
  x: number;
  y: number;
}

export default function App() {
  const [sensorConnectionStatus, setSensorConnectionStatus] = useState<RaftConnEvent>(
    RaftConnEvent.CONN_DISCONNECTED
  );
  const [motorConnectionReady, setMotorConnectionReady] = useState<boolean>(false);
  const [lastUpdate, setLastUpdate] = useState<number>(0);
  const [robotConfig, setRobotConfig] = useState<RobotConfig | null>(null);
  const [expectedPath, setExpectedPath] = useState<ExpectedPathPoint[]>([]);

  useEffect(() => {
    // Set log level once at startup, before any connections
    RaftLog.setLogLevel(RaftLogLevel.VERBOSE);

    const listener = (
      eventType: string,
      eventEnum: RaftConnEvent | RaftUpdateEvent | RaftPublishEvent,
      eventName: string,
      data?: object | string | null
    ) => {
      if (eventType === 'conn') {
        if (
          eventEnum === RaftConnEvent.CONN_CONNECTED ||
          eventEnum === RaftConnEvent.CONN_DISCONNECTED
        ) {
          setSensorConnectionStatus(eventEnum);
          
          // Set up device update listener when connected
          if (eventEnum === RaftConnEvent.CONN_CONNECTED) {
            const deviceManager = connManager.getSensorConnector().getSystemType()?.deviceMgrIF;
            if (deviceManager) {
              deviceManager.addNewAttributeCallback(() => {
                setLastUpdate(Date.now());
              });
              deviceManager.addAttributeDataCallback(() => {
                setLastUpdate(Date.now());
              });
            }
          }
        }
      }
    };

    connManager.setSensorConnectionEventListener(listener);

    // Cleanup handler for page unload/refresh
    const handleBeforeUnload = async () => {
      await connManager.disconnectSensor();
    };

    window.addEventListener('beforeunload', handleBeforeUnload);

    // Auto-connect if served from device (not localhost)
    const hostname = window.location.hostname;
    const isDevelopment = hostname === 'localhost' || hostname === '127.0.0.1' || hostname === '';
    if (!isDevelopment) {
      connManager.connectSensor(hostname);
    }

    return () => {
      window.removeEventListener('beforeunload', handleBeforeUnload);
      connManager.setSensorConnectionEventListener(() => {});
      connManager.disconnectSensor();
    };
  }, []);

  const sensorConnected = sensorConnectionStatus === RaftConnEvent.CONN_CONNECTED;

  return (
    <div className="app-container">
      <header className="app-header">
        <h1>Two Stepper Motor Test Rig</h1>
      </header>

      <ConnectionPanel 
        connectionStatus={sensorConnectionStatus}
        onMotorConnectionChange={setMotorConnectionReady}
        onRobotConfigReceived={setRobotConfig}
      />

      {sensorConnected && (
        <>
          <EncoderDisplay lastUpdate={lastUpdate} />
          <div className="chart-section">
            <AngleChart lastUpdate={lastUpdate} />
          </div>
          <div className="visualization-section">
            <RobotVisualization lastUpdate={lastUpdate} robotConfig={robotConfig} expectedPath={expectedPath} />
          </div>
          <PathExecutor motorConnectionReady={motorConnectionReady} robotConfig={robotConfig} onPathChange={setExpectedPath} />
          <MotorControl motorConnectionReady={motorConnectionReady} />
        </>
      )}
    </div>
  );
}
