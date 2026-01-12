import React, { useState, useEffect } from 'react';
import ConnManager from '../ConnManager';

const connManager = ConnManager.getInstance();

interface EncoderDisplayProps {
  lastUpdate: number;
}

export default function EncoderDisplay({ lastUpdate }: EncoderDisplayProps) {
  const [mt6701Angle, setMt6701Angle] = useState<number | null>(null);
  const [as5600Angle, setAs5600Angle] = useState<number | null>(null);

  useEffect(() => {
    const deviceManager = connManager.getConnector().getSystemType()?.deviceMgrIF;
    if (!deviceManager) return;

    // Look for MT6701 encoder (I2C address 6, device key format: busName_devAddr_devType)
    const mt6701State = deviceManager.getDeviceState('I2CA_6_MT6701');
    if (mt6701State?.deviceAttributes?.angle) {
      const values = mt6701State.deviceAttributes.angle.values;
      if (values.length > 0) {
        setMt6701Angle(values[values.length - 1]);
      }
    }

    // Look for AS5600 encoder (I2C address 36, device key format: busName_devAddr_devType)
    const as5600State = deviceManager.getDeviceState('I2CA_36_AS5600');
    if (as5600State?.deviceAttributes?.angle) {
      const values = as5600State.deviceAttributes.angle.values;
      if (values.length > 0) {
        setAs5600Angle(values[values.length - 1]);
      }
    }
  }, [lastUpdate]);

  return (
    <div className="panel">
      <h2>Magnetic Encoders</h2>
      <div className="encoder-display">
        <div className="encoder-value">
          <div className="label">MT6701</div>
          <div className="value">
            {mt6701Angle !== null ? mt6701Angle.toFixed(1) : '--.-'}
            <span className="unit">°</span>
          </div>
        </div>
        <div className="encoder-value">
          <div className="label">AS5600</div>
          <div className="value">
            {as5600Angle !== null ? as5600Angle.toFixed(1) : '--.-'}
            <span className="unit">°</span>
          </div>
        </div>
      </div>
      <div className="text-center mt-3">
        <small className="text-muted">
          {mt6701Angle === null && as5600Angle === null && 'No encoder data received'}
          {(mt6701Angle !== null || as5600Angle !== null) && 'Real-time angle measurement'}
        </small>
      </div>
    </div>
  );
}
