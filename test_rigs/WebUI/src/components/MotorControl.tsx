import React, { useState } from 'react';
import ConnManager from '../ConnManager';

const connManager = ConnManager.getInstance();

interface MotorControlProps {
  motorConnectionReady: boolean;
}

export default function MotorControl({ motorConnectionReady }: MotorControlProps) {
  const [motor1Speed, setMotor1Speed] = useState(50);
  const [motor2Speed, setMotor2Speed] = useState(50);
  const [motor1Position, setMotor1Position] = useState(0);
  const [motor2Position, setMotor2Position] = useState(0);

  const sendCommand = async (command: string) => {
    try {
      await connManager.getMotorConnector().sendRICRESTMsg(command, {});
      console.log('Command sent:', command);
    } catch (error) {
      console.error('Failed to send command:', error);
    }
  };

  const moveMotor = (motorNum: number, direction: 'cw' | 'ccw') => {
    const speed = motorNum === 1 ? motor1Speed : motor2Speed;
    const steps = 200; // One full rotation at 200 steps/rev
    const dirValue = direction === 'cw' ? 1 : -1;
    sendCommand(`motor/${motorNum}/move?steps=${steps * dirValue}&speed=${speed}`);
  };

  const stopMotor = (motorNum: number) => {
    sendCommand(`motor/${motorNum}/stop`);
  };

  const goToPosition = (motorNum: number) => {
    const position = motorNum === 1 ? motor1Position : motor2Position;
    const speed = motorNum === 1 ? motor1Speed : motor2Speed;
    sendCommand(`motor/${motorNum}/goto?pos=${position}&speed=${speed}`);
  };

  const homeMotor = (motorNum: number) => {
    sendCommand(`motor/${motorNum}/home`);
  };

  return (
    <div className="panel mt-4">
      <h2>Motor Control</h2>
      
      {!motorConnectionReady && (
        <div className="alert alert-warning" role="alert">
          <strong>Motor controller not connected.</strong> Configure the motor controller connection above to enable motor controls.
        </div>
      )}
      
      <div className={`motor-controls ${!motorConnectionReady ? 'disabled' : ''}`}>
        {/* Motor 1 */}
        <div className="motor-section">
          <h3>Motor 1</h3>
          
          <div className="control-group">
            <label htmlFor="motor1Speed">Speed: {motor1Speed}%</label>
            <input
              type="range"
              className="form-range"
              id="motor1Speed"
              min="1"
              disabled={!motorConnectionReady}
              max="100"
              value={motor1Speed}
              onChange={(e) => setMotor1Speed(parseInt(e.target.value))}
            />
          </div>

          <div className="btn-group-custom">
            <button 
              className="btn btn-primary btn-sm"
              onClick={() => moveMotor(1, 'ccw')}
              disabled={!motorConnectionReady}
            >
              ◄ CCW
            </button>
            <button 
              className="btn btn-danger btn-sm"
              onClick={() => stopMotor(1)}
              disabled={!motorConnectionReady}
            >
              ■ Stop
            </button>
            <button 
              className="btn btn-primary btn-sm"
              onClick={() => moveMotor(1, 'cw')}
              disabled={!motorConnectionReady}
            >
              CW ►
            </button>
          </div>

          <div className="control-group mt-3">
            <label htmlFor="motor1Pos">Go to Position</label>
            <div className="input-group input-group-sm">
              <input
                type="number"
                className="form-control"
                id="motor1Pos"
                value={motor1Position}
                onChange={(e) => setMotor1Position(parseInt(e.target.value) || 0)}
                disabled={!motorConnectionReady}
              />
              <button 
                className="btn btn-outline-primary"
                onClick={() => goToPosition(1)}
                disabled={!motorConnectionReady}
              >
                Go
              </button>
            </div>
          </div>

          <button 
            className="btn btn-warning btn-sm w-100 mt-2"
            onClick={() => homeMotor(1)}
            disabled={!motorConnectionReady}
          >
            🏠 Home
          </button>
        </div>

        {/* Motor 2 */}
        <div className="motor-section">
          <h3>Motor 2</h3>
          
          <div className="control-group">
            <label htmlFor="motor2Speed">Speed: {motor2Speed}%</label>
            <input
              type="range"
              className="form-range"
              id="motor2Speed"
              min="1"
              max="100"
              value={motor2Speed}
              onChange={(e) => setMotor2Speed(parseInt(e.target.value))}
              disabled={!motorConnectionReady}
            />
          </div>

          <div className="btn-group-custom">
            <button 
              className="btn btn-primary btn-sm"
              onClick={() => moveMotor(2, 'ccw')}
              disabled={!motorConnectionReady}
            >
              ◄ CCW
            </button>
            <button 
              className="btn btn-danger btn-sm"
              onClick={() => stopMotor(2)}
              disabled={!motorConnectionReady}
            >
              ■ Stop
            </button>
            <button 
              className="btn btn-primary btn-sm"
              onClick={() => moveMotor(2, 'cw')}
              disabled={!motorConnectionReady}
            >
              CW ►
            </button>
          </div>

          <div className="control-group mt-3">
            <label htmlFor="motor2Pos">Go to Position</label>
            <div className="input-group input-group-sm">
              <input
                type="number"
                className="form-control"
                id="motor2Pos"
                value={motor2Position}
                onChange={(e) => setMotor2Position(parseInt(e.target.value) || 0)}
                disabled={!motorConnectionReady}
              />
              <button 
                className="btn btn-outline-primary"
                onClick={() => goToPosition(2)}
                disabled={!motorConnectionReady}
              >
                Go
              </button>
            </div>
          </div>

          <button 
            className="btn btn-warning btn-sm w-100 mt-2"
            onClick={() => homeMotor(2)}
            disabled={!motorConnectionReady}
          >
            🏠 Home
          </button>
        </div>
      </div>
    </div>
  );
}
