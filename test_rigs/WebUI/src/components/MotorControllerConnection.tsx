import React, { useState, useEffect } from 'react';
import ConnManager from '../ConnManager';
import { RaftConnEvent } from '@robdobsn/raftjs';

const connManager = ConnManager.getInstance();

interface MotorControllerConnectionProps {
  onMotorConnectionChange: (connected: boolean) => void;
}

export default function MotorControllerConnection({ onMotorConnectionChange }: MotorControllerConnectionProps) {
  const [useSeparateController, setUseSeparateController] = useState<boolean>(
    connManager.getUseSeparateMotorController()
  );
  const [motorIpAddress, setMotorIpAddress] = useState<string>(
    connManager.getSensorIpAddress()
  );
  const [motorConnectionStatus, setMotorConnectionStatus] = useState<RaftConnEvent>(
    RaftConnEvent.CONN_DISCONNECTED
  );
  const [isConnecting, setIsConnecting] = useState<boolean>(false);

  useEffect(() => {
    // Initialize motor IP to sensor IP if available
    const sensorIp = connManager.getSensorIpAddress();
    if (sensorIp && !motorIpAddress) {
      setMotorIpAddress(sensorIp);
    }
  }, []);

  useEffect(() => {
    if (!useSeparateController) {
      // When using same device, motor is "connected" if sensor is connected
      const connected = connManager.isSensorConnected();
      setMotorConnectionStatus(
        connected ? RaftConnEvent.CONN_CONNECTED : RaftConnEvent.CONN_DISCONNECTED
      );
      onMotorConnectionChange(connected);
    } else {
      // Set up motor controller connection event listener
      const listener = (
        eventType: string,
        eventEnum: RaftConnEvent,
        eventName: string,
        data?: object | string | null
      ) => {
        if (eventType === 'conn') {
          setMotorConnectionStatus(eventEnum);
          onMotorConnectionChange(eventEnum === RaftConnEvent.CONN_CONNECTED);
          if (eventEnum === RaftConnEvent.CONN_CONNECTED || 
              eventEnum === RaftConnEvent.CONN_DISCONNECTED) {
            setIsConnecting(false);
          }
        }
      };

      connManager.setMotorConnectionEventListener(listener);

      // Check current status
      const connected = connManager.isMotorConnected();
      setMotorConnectionStatus(
        connected ? RaftConnEvent.CONN_CONNECTED : RaftConnEvent.CONN_DISCONNECTED
      );
      onMotorConnectionChange(connected);

      return () => {
        connManager.setMotorConnectionEventListener(() => {});
      };
    }
  }, [useSeparateController, onMotorConnectionChange]);

  const handleUseSeparateChange = (checked: boolean) => {
    setUseSeparateController(checked);
    connManager.setUseSeparateMotorController(checked);
    
    if (!checked) {
      // Disconnect motor controller if it was separate
      if (motorConnectionStatus === RaftConnEvent.CONN_CONNECTED) {
        connManager.disconnectMotor();
      }
      // Motor control now uses sensor connection
      onMotorConnectionChange(connManager.isSensorConnected());
    } else {
      // Need to connect to separate motor controller
      onMotorConnectionChange(false);
    }
  };

  const handleConnect = async () => {
    if (!motorIpAddress.trim()) {
      return;
    }

    setIsConnecting(true);
    setMotorConnectionStatus(RaftConnEvent.CONN_CONNECTING);
    
    try {
      await connManager.connectMotor(motorIpAddress.trim());
    } catch (error) {
      console.error('Failed to connect to motor controller:', error);
      setIsConnecting(false);
      setMotorConnectionStatus(RaftConnEvent.CONN_DISCONNECTED);
    }
  };

  const handleDisconnect = () => {
    connManager.disconnectMotor();
  };

  const getStatusClass = () => {
    switch (motorConnectionStatus) {
      case RaftConnEvent.CONN_CONNECTED:
        return 'status-connected';
      case RaftConnEvent.CONN_CONNECTING:
        return 'status-connecting';
      default:
        return 'status-disconnected';
    }
  };

  const getStatusText = () => {
    if (!useSeparateController) {
      return connManager.isSensorConnected() 
        ? 'Using sensor device for motor control'
        : 'Waiting for sensor device connection';
    }

    switch (motorConnectionStatus) {
      case RaftConnEvent.CONN_CONNECTED:
        return 'Motor controller connected';
      case RaftConnEvent.CONN_CONNECTING:
        return 'Connecting to motor controller...';
      default:
        return 'Motor controller disconnected';
    }
  };

  return (
    <div className="panel motor-controller-panel">
      <h2>Motor Controller Configuration</h2>
      
      <div className="control-group">
        <div className="form-check">
          <input
            className="form-check-input"
            type="checkbox"
            id="useSeparateController"
            checked={useSeparateController}
            onChange={(e) => handleUseSeparateChange(e.target.checked)}
          />
          <label className="form-check-label" htmlFor="useSeparateController">
            Use separate motor controller device
          </label>
        </div>
      </div>

      {useSeparateController && (
        <div className="control-group">
          <label htmlFor="motor-ip-address">Motor Controller IP Address</label>
          <div className="input-group">
            <input
              type="text"
              id="motor-ip-address"
              className="form-control"
              placeholder="Enter IP address (e.g., 192.168.1.100)"
              value={motorIpAddress}
              onChange={(e) => setMotorIpAddress(e.target.value)}
              disabled={motorConnectionStatus === RaftConnEvent.CONN_CONNECTED || isConnecting}
            />
            {motorConnectionStatus !== RaftConnEvent.CONN_CONNECTED ? (
              <button
                className="btn btn-primary"
                onClick={handleConnect}
                disabled={!motorIpAddress.trim() || isConnecting}
              >
                {isConnecting ? 'Connecting...' : 'Connect'}
              </button>
            ) : (
              <button
                className="btn btn-danger"
                onClick={handleDisconnect}
              >
                Disconnect
              </button>
            )}
          </div>
        </div>
      )}

      <div className="connection-status">
        <span className={`status-indicator ${getStatusClass()}`}></span>
        <span>{getStatusText()}</span>
      </div>
    </div>
  );
}
