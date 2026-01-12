import React, { useState } from 'react';
import ConnManager from '../ConnManager';
import { RaftConnEvent } from '@robdobsn/raftjs';

const connManager = ConnManager.getInstance();

interface ConnectionPanelProps {
  connectionStatus: RaftConnEvent;
}

export default function ConnectionPanel({ connectionStatus }: ConnectionPanelProps) {
  const [ipAddress, setIpAddress] = useState<string>(
    localStorage.getItem('lastIpAddress') || ''
  );

  const handleConnect = async () => {
    if (ipAddress.trim() === '') {
      alert('Please enter an IP address');
      return;
    }
    localStorage.setItem('lastIpAddress', ipAddress);
    await connManager.connect(ipAddress);
  };

  const handleDisconnect = () => {
    connManager.disconnect();
  };

  const getStatusClass = () => {
    switch (connectionStatus) {
      case RaftConnEvent.CONN_CONNECTED:
        return 'status-connected';
      case RaftConnEvent.CONN_CONNECTING:
        return 'status-connecting';
      default:
        return 'status-disconnected';
    }
  };

  const getStatusText = () => {
    switch (connectionStatus) {
      case RaftConnEvent.CONN_CONNECTED:
        return 'Connected';
      case RaftConnEvent.CONN_CONNECTING:
        return 'Connecting...';
      default:
        return 'Disconnected';
    }
  };

  const isConnected = connectionStatus === RaftConnEvent.CONN_CONNECTED;

  return (
    <div className="panel">
      <h2>
        <span className={`status-indicator ${getStatusClass()}`}></span>
        Connection: {getStatusText()}
      </h2>
      
      {!isConnected ? (
        <div className="row g-3 align-items-end">
          <div className="col-md-8">
            <label htmlFor="ipAddress" className="form-label">ESP32 IP Address</label>
            <input
              type="text"
              className="form-control"
              id="ipAddress"
              placeholder="192.168.1.100"
              value={ipAddress}
              onChange={(e) => setIpAddress(e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && handleConnect()}
            />
          </div>
          <div className="col-md-4">
            <button 
              className="btn btn-primary w-100"
              onClick={handleConnect}
              disabled={connectionStatus === RaftConnEvent.CONN_CONNECTING}
            >
              Connect
            </button>
          </div>
        </div>
      ) : (
        <div className="d-flex justify-content-between align-items-center">
          <div>
            <strong>Connected to:</strong> {ipAddress}
          </div>
          <button 
            className="btn btn-danger"
            onClick={handleDisconnect}
          >
            Disconnect
          </button>
        </div>
      )}
    </div>
  );
}
