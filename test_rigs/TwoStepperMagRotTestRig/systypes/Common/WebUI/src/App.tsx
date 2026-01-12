import React, { useState, useEffect } from 'react';
import ConnectionPanel from './components/ConnectionPanel';
import MotorControl from './components/MotorControl';
import EncoderDisplay from './components/EncoderDisplay';
import StatusPanel from './components/StatusPanel';
import ConnManager from './ConnManager';
import { RaftConnEvent } from '@robdobsn/raftjs';

const connManager = ConnManager.getInstance();

export default function App() {
  const [connectionStatus, setConnectionStatus] = useState<RaftConnEvent>(
    RaftConnEvent.CONN_DISCONNECTED
  );
  const [lastUpdate, setLastUpdate] = useState<number>(0);

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
          setConnectionStatus(eventEnum);
          
          // Set up device update listener when connected
          if (eventEnum === RaftConnEvent.CONN_CONNECTED) {
            const deviceManager = connManager.getConnector().getSystemType()?.deviceMgrIF;
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

    connManager.setConnectionEventListener(listener);

    // Auto-connect if served from device (not localhost)
    const hostname = window.location.hostname;
    const isDevelopment = hostname === 'localhost' || hostname === '127.0.0.1' || hostname === '';
    if (!isDevelopment) {
      connManager.connect(hostname);
    }

    return () => {
      connManager.setConnectionEventListener(() => {});
    };
  }, []);

  const isDevelopment = window.location.hostname === 'localhost' || 
                        window.location.hostname === '127.0.0.1' ||
                        window.location.hostname === '';

  return (
    <div className="app-container">
      <header className="app-header">
        <h1>Two Stepper Motor Test Rig</h1>
      </header>

      {isDevelopment && (
        <ConnectionPanel 
          connectionStatus={connectionStatus}
        />
      )}

      {connectionStatus === RaftConnEvent.CONN_CONNECTED && (
        <>
          <div className="main-grid">
            <EncoderDisplay lastUpdate={lastUpdate} />
            <StatusPanel lastUpdate={lastUpdate} />
          </div>
          <MotorControl />
        </>
      )}
    </div>
  );
}
