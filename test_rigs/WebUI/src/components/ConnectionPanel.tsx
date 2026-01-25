import React, { useState, useEffect } from 'react';
import ConnManager from '../ConnManager';
import { RaftConnEvent, RaftSystemInfo } from '@robdobsn/raftjs';

const connManager = ConnManager.getInstance();

interface ConnectionPanelProps {
  connectionStatus: RaftConnEvent;
  onMotorConnectionChange: (connected: boolean) => void;
  onRobotConfigReceived?: (config: any) => void;
}

export default function ConnectionPanel({ connectionStatus, onMotorConnectionChange, onRobotConfigReceived }: ConnectionPanelProps) {
  const [ipAddress, setIpAddress] = useState<string>(
    localStorage.getItem('lastIpAddress') || ''
  );
  const [useSeparateController, setUseSeparateController] = useState<boolean>(connManager.getUseSeparateMotorController());
  const [motorIpAddress, setMotorIpAddress] = useState<string>('');
  const [motorConnectionStatus, setMotorConnectionStatus] = useState<RaftConnEvent>(RaftConnEvent.CONN_DISCONNECTED);
  const [isConnectingMotor, setIsConnectingMotor] = useState<boolean>(false);
  const [systemInfo, setSystemInfo] = useState<RaftSystemInfo | null>(null);

  const handleConnect = async () => {
    if (ipAddress.trim() === '') {
      alert('Please enter an IP address');
      return;
    }
    localStorage.setItem('lastIpAddress', ipAddress);
    await connManager.connect(ipAddress);
  };

  const handleDisconnect = async () => {
    await connManager.disconnect();
  };

  const fetchRobotSettings = async () => {
    try {
      const motorConnector = connManager.getMotorConnector();
      if (!motorConnector || !motorConnector.isConnected()) {
        console.log('Motor connector not ready yet');
        return;
      }
      const systemUtils = motorConnector.getRaftSystemUtils();
      if (!systemUtils) {
        console.log('System utils not available yet');
        return;
      }
      const msgHandler = systemUtils.getMsgHandler();
      if (!msgHandler) {
        console.log('Message handler not available yet');
        return;
      }
      const response = await msgHandler.sendRICRESTURL<any>('getsettings');
      if (response && typeof response === 'object') {
        const motorControl = response?.base?.DevMan?.Devices?.find(
          (dev: any) => dev.class === 'MotorControl'
        );
        
        if (motorControl?.motion && onRobotConfigReceived) {
          const motion = motorControl.motion;
          
          // Extract axes configuration if available
          const axes = motorControl.axes?.map((axis: any) => ({
            name: axis.name || '',
            unitsPerRot: axis.params?.unitsPerRot || 360,
            stepsPerRot: axis.params?.stepsPerRot || 3200,
            maxSpeedUps: axis.params?.maxSpeedUps || 50,
            maxAccUps2: axis.params?.maxAccUps2 || 10
          })) || [];
          
          const robotConfig = {
            geometry: motion.geom || 'SingleArmSCARA',
            arm1LengthMM: motion.arm1LenMM || 150,
            arm2LengthMM: motion.arm2LenMM || 150,
            maxRadiusMM: motion.maxRadiusMM || 290,
            originTheta2OffsetDegrees: motion.originTheta2OffsetDegrees || 180,
            axes: axes
          };
          console.log('Parsed robot config with axes:', robotConfig);
          onRobotConfigReceived(robotConfig);
        }
      }
    } catch (error) {
      console.error('Failed to fetch robot settings:', error);
    }
  };

  const fetchSystemInfo = async () => {
    try {
      const connector = connManager.getConnector();
      if (!connector || !connector.isConnected()) {
        return;
      }
      const systemUtils = connector.getRaftSystemUtils();
      if (systemUtils) {
        const info = await systemUtils.getSystemInfo();
        setSystemInfo(info);
      }
    } catch (error) {
      console.warn('Failed to get system info:', error);
    }
  };

  useEffect(() => {
    if (connectionStatus === RaftConnEvent.CONN_CONNECTED) {
      // Delay fetching to allow connector to fully initialize
      const timer = setTimeout(() => {
        fetchSystemInfo();
        if (!useSeparateController) {
          onMotorConnectionChange(true);
          fetchRobotSettings();
        }
      }, 100);
      return () => clearTimeout(timer);
    } else {
      setSystemInfo(null);
    }
  }, [connectionStatus, useSeparateController]);

  useEffect(() => {
    if (!useSeparateController) {
      const connected = connManager.isSensorConnected();
      setMotorConnectionStatus(connected ? RaftConnEvent.CONN_CONNECTED : RaftConnEvent.CONN_DISCONNECTED);
      onMotorConnectionChange(connected);
      if (connected) fetchRobotSettings();
    } else {
      const listener = (eventType: string, eventEnum: RaftConnEvent) => {
        if (eventType === 'conn') {
          setMotorConnectionStatus(eventEnum);
          onMotorConnectionChange(eventEnum === RaftConnEvent.CONN_CONNECTED);
          if (eventEnum === RaftConnEvent.CONN_CONNECTED) fetchRobotSettings();
          if (eventEnum === RaftConnEvent.CONN_CONNECTED || eventEnum === RaftConnEvent.CONN_DISCONNECTED) {
            setIsConnectingMotor(false);
          }
        }
      };
      connManager.setMotorConnectionEventListener(listener);
      return () => connManager.setMotorConnectionEventListener(() => {});
    }
  }, [useSeparateController]);

  const handleMotorConnect = async () => {
    if (!motorIpAddress.trim()) return;
    setIsConnectingMotor(true);
    setMotorConnectionStatus(RaftConnEvent.CONN_CONNECTING);
    
    try {
      await connManager.connectMotor(motorIpAddress.trim());
    } catch (error) {
      console.error('Failed to connect to motor controller:', error);
      setIsConnectingMotor(false);
      setMotorConnectionStatus(RaftConnEvent.CONN_DISCONNECTED);
    }
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

  const getMotorStatusClass = () => {
    if (!useSeparateController) return getStatusClass();
    switch (motorConnectionStatus) {
      case RaftConnEvent.CONN_CONNECTED:
        return 'status-connected';
      case RaftConnEvent.CONN_CONNECTING:
        return 'status-connecting';
      default:
        return 'status-disconnected';
    }
  };

  const isConnected = connectionStatus === RaftConnEvent.CONN_CONNECTED;

  return (
    <div className="panel panel-compact">
      <h2>Device Connection</h2>
      
      {/* Sensor Connection */}
      <div className="connection-section">
        <div className="connection-row">
          <div className="connection-status-inline">
            <span className={`status-indicator ${getStatusClass()}`}></span>
            <span className="status-label">Sensor:</span>
          </div>
          {!isConnected ? (
            <div className="connection-input-group">
              <input
                type="text"
                className="form-control form-control-sm"
                placeholder="192.168.1.100"
                value={ipAddress}
                onChange={(e) => setIpAddress(e.target.value)}
                onKeyDown={(e) => e.key === 'Enter' && handleConnect()}
              />
              <button 
                className="btn btn-primary btn-sm"
                onClick={handleConnect}
                disabled={connectionStatus === RaftConnEvent.CONN_CONNECTING}
              >
                Connect
              </button>
            </div>
          ) : (
            <div className="connection-info">
              <span className="connected-ip">{ipAddress}</span>
              <button className="btn btn-danger btn-sm" onClick={handleDisconnect}>Disconnect</button>
            </div>
          )}
        </div>

        {/* System Status - shown when connected */}
        {isConnected && systemInfo && (
          <div className="system-status-inline">
            <span className="status-text">{systemInfo.SystemName || 'Unknown'} v{systemInfo.SystemVersion || 'N/A'}</span>
            {systemInfo.MAC && <span className="status-detail">MAC: {systemInfo.MAC}</span>}
          </div>
        )}
      </div>

      {/* Motor Controller Configuration */}
      {isConnected && (
        <div className="connection-section">
          <div className="form-check form-check-sm">
            <input
              className="form-check-input"
              type="checkbox"
              id="useSeparateController"
              checked={useSeparateController}
              onChange={(e) => {
                setUseSeparateController(e.target.checked);
                connManager.setUseSeparateMotorController(e.target.checked);
              }}
            />
            <label className="form-check-label" htmlFor="useSeparateController">
              Use separate motor controller
            </label>
          </div>

          {useSeparateController && (
            <div className="connection-row">
              <div className="connection-status-inline">
                <span className={`status-indicator ${getMotorStatusClass()}`}></span>
                <span className="status-label">Motor:</span>
              </div>
              {motorConnectionStatus !== RaftConnEvent.CONN_CONNECTED ? (
                <div className="connection-input-group">
                  <input
                    type="text"
                    className="form-control form-control-sm"
                    placeholder="192.168.1.101"
                    value={motorIpAddress}
                    onChange={(e) => setMotorIpAddress(e.target.value)}
                    disabled={isConnectingMotor}
                  />
                  <button
                    className="btn btn-primary btn-sm"
                    onClick={handleMotorConnect}
                    disabled={!motorIpAddress.trim() || isConnectingMotor}
                  >
                    {isConnectingMotor ? 'Connecting...' : 'Connect'}
                  </button>
                </div>
              ) : (
                <div className="connection-info">
                  <span className="connected-ip">{motorIpAddress}</span>
                  <button className="btn btn-danger btn-sm" onClick={() => connManager.disconnectMotor()}>Disconnect</button>
                </div>
              )}
            </div>
          )}

          {!useSeparateController && (
            <div className="connection-status-inline">
              <span className={`status-indicator ${getMotorStatusClass()}`}></span>
              <span className="status-text">Motor controller: using sensor device</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
