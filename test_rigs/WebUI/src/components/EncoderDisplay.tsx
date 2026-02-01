import React, { useState, useEffect } from 'react';
import ConnManager from '../ConnManager';
import { getRobotGeometry } from '../utils/RobotGeometry';

const connManager = ConnManager.getInstance();
const robotGeometry = getRobotGeometry();

interface EncoderDisplayProps {
  lastUpdate: number;
}

export default function EncoderDisplay({ lastUpdate }: EncoderDisplayProps) {
  const [mt6701Angle, setMt6701Angle] = useState<number | null>(null);
  const [as5600Angle, setAs5600Angle] = useState<number | null>(null);
  const [publishedPos, setPublishedPos] = useState<{ x: number | null; y: number | null; z: number | null }>({ x: null, y: null, z: null });
  const [motorBusy, setMotorBusy] = useState<boolean | null>(null);
  const [motorPaused, setMotorPaused] = useState<boolean | null>(null);
  const [invertJoint1, setInvertJoint1] = useState<boolean>(robotGeometry.shouldInvertJoint1Sensor());
  const [invertJoint2, setInvertJoint2] = useState<boolean>(robotGeometry.shouldInvertJoint2Sensor());

  const handleInvertJoint1Change = (e: React.ChangeEvent<HTMLInputElement>) => {
    const checked = e.target.checked;
    setInvertJoint1(checked);
    robotGeometry.setInvertJoint1Sensor(checked);
  };

  const handleInvertJoint2Change = (e: React.ChangeEvent<HTMLInputElement>) => {
    const checked = e.target.checked;
    setInvertJoint2(checked);
    robotGeometry.setInvertJoint2Sensor(checked);
  };

  useEffect(() => {
    const deviceManager = connManager.getConnector().getSystemType()?.deviceMgrIF;
    if (!deviceManager) return;

    // Look for MT6701 encoder (I2C address 6, device key format: busName_devAddr_devType)
    const mt6701State = deviceManager.getDeviceState('1_6');
    if (mt6701State?.deviceAttributes?.angle) {
      const values = mt6701State.deviceAttributes.angle.values;
      if (values.length > 0) {
        setMt6701Angle(values[values.length - 1]);
      }
    }

    // Look for AS5600 encoder (I2C address 36, device key format: busName_devAddr_devType)
    const as5600State = deviceManager.getDeviceState('1_36');
    if (as5600State?.deviceAttributes?.angle) {
      const values = as5600State.deviceAttributes.angle.values;
      if (values.length > 0) {
        setAs5600Angle(values[values.length - 1]);
      }
    }

    // Look for MotorControl device (direct connection, device key: 0_0)
    const motorControlState = deviceManager.getDeviceState('0_0');
    if (motorControlState?.deviceAttributes) {
      const pos0 = motorControlState.deviceAttributes.pos0?.values;
      const pos1 = motorControlState.deviceAttributes.pos1?.values;
      const pos2 = motorControlState.deviceAttributes.pos2?.values;
      const busy = motorControlState.deviceAttributes.busy?.values;
      const paused = motorControlState.deviceAttributes.paused?.values;
      
      setPublishedPos({
        x: pos0?.length > 0 ? pos0[pos0.length - 1] : null,
        y: pos1?.length > 0 ? pos1[pos1.length - 1] : null,
        z: pos2?.length > 0 ? pos2[pos2.length - 1] : null,
      });
      
      if (busy?.length > 0) {
        setMotorBusy(busy[busy.length - 1] !== 0);
      }
      if (paused?.length > 0) {
        setMotorPaused(paused[paused.length - 1] !== 0);
      }
    }
  }, [lastUpdate]);

  return (
    <div className="panel panel-compact">
      <h2 style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', flexWrap: 'wrap', gap: '8px' }}>
        <span>Current Position</span>
        <span style={{ display: 'flex', gap: '12px', fontSize: '0.75rem', fontWeight: 'normal' }}>
          <label style={{ display: 'flex', alignItems: 'center', gap: '4px', cursor: 'pointer' }}>
            <input
              type="checkbox"
              checked={invertJoint1}
              onChange={handleInvertJoint1Change}
              style={{ cursor: 'pointer' }}
            />
            Inv J1
          </label>
          <label style={{ display: 'flex', alignItems: 'center', gap: '4px', cursor: 'pointer' }}>
            <input
              type="checkbox"
              checked={invertJoint2}
              onChange={handleInvertJoint2Change}
              style={{ cursor: 'pointer' }}
            />
            Inv J2
          </label>
        </span>
      </h2>
      <div className="encoder-display-compact">
        <div className="encoder-value-compact">
          <span className="label">MT6701:</span>
          <span className="value">
            {mt6701Angle !== null ? mt6701Angle.toFixed(1) : '--.-'}°
          </span>
        </div>
        <div className="encoder-value-compact">
          <span className="label">AS5600:</span>
          <span className="value">
            {as5600Angle !== null ? as5600Angle.toFixed(1) : '--.-'}°
          </span>
        </div>
      </div>
      <div className="encoder-display-compact" style={{ marginTop: '8px', borderTop: '1px solid #333', paddingTop: '8px' }}>
        <div className="encoder-value-compact">
          <span className="label">Pos X:</span>
          <span className="value" style={{ color: '#ff5252' }}>
            {publishedPos.x !== null ? publishedPos.x.toFixed(2) : '--.-'}mm
          </span>
        </div>
        <div className="encoder-value-compact">
          <span className="label">Pos Y:</span>
          <span className="value" style={{ color: '#ff5252' }}>
            {publishedPos.y !== null ? publishedPos.y.toFixed(2) : '--.-'}mm
          </span>
        </div>
        <div className="encoder-value-compact">
          <span className="label">Pos Z:</span>
          <span className="value" style={{ color: '#ff5252' }}>
            {publishedPos.z !== null ? publishedPos.z.toFixed(2) : '--.-'}mm
          </span>
        </div>
        <div className="encoder-value-compact">
          <span className="label">Status:</span>
          <span className="value">
            {motorBusy !== null ? (motorBusy ? '🔄 Busy' : '✓ Idle') : '--'}
            {motorPaused ? ' ⏸' : ''}
          </span>
        </div>
      </div>
    </div>
  );
}
