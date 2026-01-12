import React, { useState, useEffect } from 'react';
import ConnManager from '../ConnManager';
import { RaftSystemInfo } from '@robdobsn/raftjs';

const connManager = ConnManager.getInstance();

interface StatusPanelProps {
  lastUpdate: number;
}

export default function StatusPanel({ lastUpdate }: StatusPanelProps) {
  const [systemInfo, setSystemInfo] = useState<RaftSystemInfo | null>(null);

  useEffect(() => {
    const fetchSystemInfo = async () => {
      try {
        const systemUtils = connManager.getConnector().getSystemType()?.setup && 
                           connManager.getConnector().getRaftSystemUtils();
        if (systemUtils) {
          const info = await systemUtils.getSystemInfo();
          setSystemInfo(info);
        }
      } catch (error) {
        console.warn('Failed to get system info:', error);
      }
    };

    if (connManager.isConnected()) {
      fetchSystemInfo();
    }
  }, [lastUpdate]);

  return (
    <div className="panel">
      <h2>System Status</h2>
      {systemInfo ? (
        <div className="info-grid">
          <span className="info-label">System Name:</span>
          <span className="info-value">{systemInfo.SystemName || 'N/A'}</span>
          
          <span className="info-label">Version:</span>
          <span className="info-value">{systemInfo.SystemVersion || 'N/A'}</span>
          
          <span className="info-label">MAC Address:</span>
          <span className="info-value">{systemInfo.MAC || 'N/A'}</span>
          
          <span className="info-label">Hardware Rev:</span>
          <span className="info-value">{systemInfo.HwRev || 'N/A'}</span>
          
          <span className="info-label">Friendly Name:</span>
          <span className="info-value">{systemInfo.Friendly || 'N/A'}</span>
        </div>
      ) : (
        <div className="text-center text-muted">
          <p>Loading system information...</p>
        </div>
      )}
    </div>
  );
}
