import React, { useState, useEffect } from 'react';
import ConnManager from '../ConnManager';
import { RaftConnEvent } from '@robdobsn/raftjs';

const connManager = ConnManager.getInstance();

export interface AxisConfig {
  name: string;
  unitsPerRot: number;
  stepsPerRot: number;
  maxSpeedUps: number;
  maxAccUps2: number;
}

export interface RobotConfig {
  geometry: string;
  arm1LengthMM: number;
  arm2LengthMM: number;
  maxRadiusMM: number;
  originTheta2OffsetDegrees: number;
  axes?: AxisConfig[];
}

interface MotorControllerConnectionProps {
  onMotorConnectionChange: (connected: boolean) => void;
  onRobotConfigReceived?: (config: RobotConfig | null) => void;
}

export default function MotorControllerConnection({ onMotorConnectionChange, onRobotConfigReceived }: MotorControllerConnectionProps) {
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

  // Fetch robot configuration settings from motor controller
  const fetchRobotSettings = async () => {
    try {
      console.log('Fetching robot settings from motor controller...');
      const systemUtils = connManager.getMotorConnector().getRaftSystemUtils();
      const msgHandler = systemUtils.getMsgHandler();
      const response = await msgHandler.sendRICRESTURL<any>('getsettings');
      console.log('Robot settings response:', response);
      if (response && typeof response === 'object') {
        console.log('Robot settings (formatted):', JSON.stringify(response, null, 2));
        
        // Parse motion configuration
        const motorControl = response?.base?.DevMan?.Devices?.find(
          (dev: any) => dev.class === 'MotorControl'
        );
        
        if (motorControl?.motion) {
          const motion = motorControl.motion;
          
          // Extract axes configuration if available
          const axes: AxisConfig[] = motorControl.axes?.map((axis: any) => ({
            name: axis.name || '',
            unitsPerRot: axis.params?.unitsPerRot || 360,
            stepsPerRot: axis.params?.stepsPerRot || 3200,
            maxSpeedUps: axis.params?.maxSpeedUps || 50,
            maxAccUps2: axis.params?.maxAccUps2 || 10
          })) || [];
          
          const robotConfig: RobotConfig = {
            geometry: motion.geom || 'SingleArmSCARA',
            arm1LengthMM: motion.arm1LenMM || 150,
            arm2LengthMM: motion.arm2LenMM || 150,
            maxRadiusMM: motion.maxRadiusMM || 290,
            originTheta2OffsetDegrees: motion.originTheta2OffsetDegrees || 180,
            axes: axes
          };
          console.log('Parsed robot config with axes:', robotConfig);
          if (onRobotConfigReceived) {
            onRobotConfigReceived(robotConfig);
          }
        }
      }
    } catch (error) {
      console.error('Failed to fetch robot settings:', error);
      if (onRobotConfigReceived) {
        onRobotConfigReceived(null);
      }
    }
  };

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
      
      // Fetch settings when connected
      if (connected) {
        fetchRobotSettings();
      }
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
          
          // Fetch settings when motor controller connects
          if (eventEnum === RaftConnEvent.CONN_CONNECTED) {
            fetchRobotSettings();
          }
          
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
