import React, { useState, useEffect } from 'react';
import ConnectionPanel from './components/ConnectionPanel';
import MotorControllerConnection, { RobotConfig } from './components/MotorControllerConnection';
import MotorControl from './components/MotorControl';
import EncoderDisplay from './components/EncoderDisplay';
import StatusPanel from './components/StatusPanel';
import AngleChart from './components/AngleChart';
import RobotVisualization from './components/RobotVisualization';
import ConnManager from './ConnManager';
import { RaftConnEvent } from '@robdobsn/raftjs';

const connManager = ConnManager.getInstance();

export default function App() {
  const [sensorConnectionStatus, setSensorConnectionStatus] = useState<RaftConnEvent>(
    RaftConnEvent.CONN_DISCONNECTED
  );
  const [motorConnectionReady, setMotorConnectionReady] = useState<boolean>(false);
  const [lastUpdate, setLastUpdate] = useState<number>(0);
  const [robotConfig, setRobotConfig] = useState<RobotConfig | null>(null);

  useEffect(() => {
    const listener = (
      eventType: string,
      eventEnum: RaftConnEvent,
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

    // Auto-connect if served from device (not localhost)
    const hostname = window.location.hostname;
    const isDevelopment = hostname === 'localhost' || hostname === '127.0.0.1' || hostname === '';
    if (!isDevelopment) {
      connManager.connectSensor(hostname);
    }

    return () => {
      connManager.setSensorConnectionEventListener(() => {});
    };
  }, []);

  const isDevelopment = window.location.hostname === 'localhost' || 
                        window.location.hostname === '127.0.0.1' ||
                        window.location.hostname === '';

  const sensorConnected = sensorConnectionStatus === RaftConnEvent.CONN_CONNECTED;

  return (
    <div className="app-container">
      <header className="app-header">
        <h1>Two Stepper Motor Test Rig</h1>
      </header>

      {isDevelopment && (
        <ConnectionPanel 
          connectionStatus={sensorConnectionStatus}
        />
      )}

      {sensorConnected && (
        <>
          <MotorControllerConnection 
            onMotorConnectionChange={setMotorConnectionReady}
            onRobotConfigReceived={setRobotConfig}
          />
          
          <div className="main-grid">
            <EncoderDisplay lastUpdate={lastUpdate} />
            <StatusPanel lastUpdate={lastUpdate} />
          </div>
          <div className="chart-section">
            <AngleChart lastUpdate={lastUpdate} />
          </div>
          <div className="visualization-section">
            <RobotVisualization lastUpdate={lastUpdate} robotConfig={robotConfig} />
          </div>
          <MotorControl motorConnectionReady={motorConnectionReady} />
        </>
      )}
    </div>
  );
}
