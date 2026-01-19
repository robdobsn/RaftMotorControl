import React, { useState, useRef } from 'react';
import ConnManager from '../ConnManager';
import { RobotConfig } from '../App';

const connManager = ConnManager.getInstance();

interface PathExecutorProps {
  motorConnectionReady: boolean;
  robotConfig: RobotConfig | null;
}

interface PathPoint {
  a0: number;
  a1: number;
}

type PatternType = 'circle' | 'rectangle' | 'lissajous1' | 'lissajous2' | 'lissajous3' | 'figure8' | 'star';

interface PatternDefinition {
  name: string;
  type: PatternType;
  description: string;
}

const PATTERNS: PatternDefinition[] = [
  { name: 'Circle', type: 'circle', description: 'Circular path' },
  { name: 'Rectangle', type: 'rectangle', description: 'Rectangular path' },
  { name: 'Lissajous (1:1)', type: 'lissajous1', description: 'Lissajous curve a=1, b=1, δ=π/2' },
  { name: 'Lissajous (3:2)', type: 'lissajous2', description: 'Lissajous curve a=3, b=2, δ=0' },
  { name: 'Lissajous (5:4)', type: 'lissajous3', description: 'Lissajous curve a=5, b=4, δ=π/2' },
  { name: 'Figure-8', type: 'figure8', description: 'Figure-eight pattern' },
  { name: 'Star', type: 'star', description: '5-pointed star' },
];

export default function PathExecutor({ motorConnectionReady, robotConfig }: PathExecutorProps) {
  const [selectedPattern, setSelectedPattern] = useState<PatternType>('circle');
  const [speed, setSpeed] = useState(50);
  const [repetitions, setRepetitions] = useState(1);
  const [delayMs, setDelayMs] = useState(50);
  const [pointsPerPath, setPointsPerPath] = useState(50);
  const [isExecuting, setIsExecuting] = useState(false);
  const [progress, setProgress] = useState({ current: 0, total: 0 });
  const [connectorReady, setConnectorReady] = useState(false);
  
  const executionRef = useRef<{ shouldStop: boolean }>({ shouldStop: false });

  // Check connector readiness
  React.useEffect(() => {
    const checkConnector = () => {
      const motorConnector = connManager.getMotorConnector();
      const ready = motorConnectionReady && motorConnector?.isConnected() === true;
      setConnectorReady(ready);
    };
    
    // Check immediately and set up interval
    checkConnector();
    const interval = setInterval(checkConnector, 500);
    return () => clearInterval(interval);
  }, [motorConnectionReady]);

  // Generate normalized path points (0 to 1 range)
  const generateNormalizedPath = (patternType: PatternType, numPoints: number): PathPoint[] => {
    const points: PathPoint[] = [];
    
    switch (patternType) {
      case 'circle':
        for (let i = 0; i < numPoints; i++) {
          const angle = (i / numPoints) * 2 * Math.PI;
          points.push({
            a0: 0.5 + 0.5 * Math.cos(angle),
            a1: 0.5 + 0.5 * Math.sin(angle),
          });
        }
        break;

      case 'rectangle':
        const pointsPerSide = Math.floor(numPoints / 4);
        // Top edge
        for (let i = 0; i < pointsPerSide; i++) {
          points.push({ a0: i / pointsPerSide, a1: 0 });
        }
        // Right edge
        for (let i = 0; i < pointsPerSide; i++) {
          points.push({ a0: 1, a1: i / pointsPerSide });
        }
        // Bottom edge
        for (let i = 0; i < pointsPerSide; i++) {
          points.push({ a0: 1 - i / pointsPerSide, a1: 1 });
        }
        // Left edge
        for (let i = 0; i < pointsPerSide; i++) {
          points.push({ a0: 0, a1: 1 - i / pointsPerSide });
        }
        break;

      case 'lissajous1':
        // a=1, b=1, δ=π/2 (circle rotated 45°)
        for (let i = 0; i < numPoints; i++) {
          const t = (i / numPoints) * 2 * Math.PI;
          points.push({
            a0: 0.5 + 0.5 * Math.sin(t),
            a1: 0.5 + 0.5 * Math.sin(t + Math.PI / 2),
          });
        }
        break;

      case 'lissajous2':
        // a=3, b=2, δ=0
        for (let i = 0; i < numPoints; i++) {
          const t = (i / numPoints) * 2 * Math.PI;
          points.push({
            a0: 0.5 + 0.5 * Math.sin(3 * t),
            a1: 0.5 + 0.5 * Math.sin(2 * t),
          });
        }
        break;

      case 'lissajous3':
        // a=5, b=4, δ=π/2
        for (let i = 0; i < numPoints; i++) {
          const t = (i / numPoints) * 2 * Math.PI;
          points.push({
            a0: 0.5 + 0.5 * Math.sin(5 * t),
            a1: 0.5 + 0.5 * Math.sin(4 * t + Math.PI / 2),
          });
        }
        break;

      case 'figure8':
        for (let i = 0; i < numPoints; i++) {
          const t = (i / numPoints) * 2 * Math.PI;
          points.push({
            a0: 0.5 + 0.5 * Math.sin(t),
            a1: 0.5 + 0.25 * Math.sin(2 * t),
          });
        }
        break;

      case 'star':
        const starPoints = 5;
        const outerRadius = 0.5;
        const innerRadius = 0.2;
        for (let i = 0; i < starPoints * 2; i++) {
          const angle = (i / (starPoints * 2)) * 2 * Math.PI - Math.PI / 2;
          const radius = i % 2 === 0 ? outerRadius : innerRadius;
          points.push({
            a0: 0.5 + radius * Math.cos(angle),
            a1: 0.5 + radius * Math.sin(angle),
          });
        }
        // Close the star
        const firstAngle = -Math.PI / 2;
        points.push({
          a0: 0.5 + outerRadius * Math.cos(firstAngle),
          a1: 0.5 + outerRadius * Math.sin(firstAngle),
        });
        break;
    }

    return points;
  };

  // Scale normalized points to fit within robot workspace
  const scalePathToWorkspace = (normalizedPoints: PathPoint[]): PathPoint[] => {
    if (!robotConfig) {
      // Default to a small workspace if no config
      const defaultRadius = 100;
      return normalizedPoints.map(p => ({
        a0: (p.a0 - 0.5) * defaultRadius * 2,
        a1: (p.a1 - 0.5) * defaultRadius * 2,
      }));
    }

    // For SCARA, use the smaller of maxRadiusMM or sum of arm lengths
    const arm1Len = robotConfig.arm1LengthMM || 150;
    const arm2Len = robotConfig.arm2LengthMM || 150;
    const maxRadius = robotConfig.maxRadiusMM || Math.min(290, arm1Len + arm2Len);
    
    // Scale to 90% of workspace to leave some margin
    const workspaceSize = maxRadius * 2 * 0.9;

    return normalizedPoints.map(p => ({
      a0: (p.a0 - 0.5) * workspaceSize,
      a1: (p.a1 - 0.5) * workspaceSize,
    }));
  };

  // Execute a single movement command
  const sendMoveCommand = async (point: PathPoint, speedValue: number): Promise<void> => {
    const command = `motors/setPos?a0=${point.a0.toFixed(2)}&a1=${point.a1.toFixed(2)}&speedUps=${speedValue}&stopAndClear=false&noSplit=true`;
    try {
      await connManager.getMotorConnector().sendRICRESTMsg(command, {});
      console.log('Move command sent:', command);
    } catch (error) {
      console.error('Failed to send move command:', error);
      throw error;
    }
  };

  // Execute the path with sequential commands
  const executePath = async () => {
    if (!connectorReady) {
      alert('Motor connector not ready. Please wait for connection to fully initialize.');
      return;
    }

    setIsExecuting(true);
    executionRef.current.shouldStop = false;

    try {
      // Generate the path
      const normalizedPath = generateNormalizedPath(selectedPattern, pointsPerPath);
      const scaledPath = scalePathToWorkspace(normalizedPath);

      const totalPoints = scaledPath.length * repetitions;
      setProgress({ current: 0, total: totalPoints });

      // Execute repetitions
      for (let rep = 0; rep < repetitions && !executionRef.current.shouldStop; rep++) {
        for (let i = 0; i < scaledPath.length && !executionRef.current.shouldStop; i++) {
          await sendMoveCommand(scaledPath[i], speed);
          setProgress({ 
            current: rep * scaledPath.length + i + 1, 
            total: totalPoints 
          });

          // Wait before sending next command
          if (i < scaledPath.length - 1 || rep < repetitions - 1) {
            await new Promise(resolve => setTimeout(resolve, delayMs));
          }
        }
      }

      if (!executionRef.current.shouldStop) {
        console.log('Path execution completed');
      } else {
        console.log('Path execution stopped by user');
      }
    } catch (error) {
      console.error('Error during path execution:', error);
      alert('Error during path execution. Check console for details.');
    } finally {
      setIsExecuting(false);
      setProgress({ current: 0, total: 0 });
    }
  };

  const stopExecution = async () => {
    executionRef.current.shouldStop = true;
    
    // Send stop command to motors
    try {
      await connManager.getMotorConnector().sendRICRESTMsg('motors/stop', {});
      console.log('Stop command sent');
    } catch (error) {
      console.error('Failed to send stop command:', error);
    }
  };

  const selectedPatternDef = PATTERNS.find(p => p.type === selectedPattern);

  return (
    <div className="panel panel-compact">
      <h2>Path Executor</h2>

      {!connectorReady && (
        <div className="alert alert-warning alert-compact" role="alert">
          <strong>{motorConnectionReady ? 'Initializing connection...' : 'Motor controller not connected.'}</strong> {motorConnectionReady ? 'Please wait.' : 'Configure connection to enable path execution.'}
        </div>
      )}

      <div className={`path-executor-controls ${!connectorReady ? 'disabled' : ''}`}>
        {/* Pattern Selection and Parameters in Grid */}
        <div className="path-params-grid">
          <div className="control-group-compact">
            <label htmlFor="patternSelect">Pattern</label>
            <select
              id="patternSelect"
              className="form-select form-select-sm"
              value={selectedPattern}
              onChange={(e) => setSelectedPattern(e.target.value as PatternType)}
              disabled={!connectorReady || isExecuting}
            >
              {PATTERNS.map(pattern => (
                <option key={pattern.type} value={pattern.type}>
                  {pattern.name}
                </option>
              ))}
            </select>
          </div>

          <div className="control-group-compact">
            <label htmlFor="speedSlider">Speed: {speed}</label>
            <input
              type="range"
              className="form-range form-range-sm"
              id="speedSlider"
              min="10"
              max="200"
              value={speed}
              onChange={(e) => setSpeed(parseInt(e.target.value))}
              disabled={!connectorReady || isExecuting}
            />
          </div>

          <div className="control-group-compact">
            <label htmlFor="pointsSlider">Points: {pointsPerPath}</label>
            <input
              type="range"
              className="form-range form-range-sm"
              id="pointsSlider"
              min="20"
              max="200"
              step="10"
              value={pointsPerPath}
              onChange={(e) => setPointsPerPath(parseInt(e.target.value))}
              disabled={!connectorReady || isExecuting}
            />
          </div>

          <div className="control-group-compact">
            <label htmlFor="delaySlider">Delay: {delayMs}ms</label>
            <input
              type="range"
              className="form-range form-range-sm"
              id="delaySlider"
              min="10"
              max="500"
              step="10"
              value={delayMs}
              onChange={(e) => setDelayMs(parseInt(e.target.value))}
              disabled={!connectorReady || isExecuting}
            />
          </div>

          <div className="control-group-compact">
            <label htmlFor="repetitionsInput">Reps</label>
            <input
              type="number"
              className="form-control form-control-sm"
              id="repetitionsInput"
              min="1"
              max="100"
              value={repetitions}
              onChange={(e) => setRepetitions(Math.max(1, parseInt(e.target.value) || 1))}
              disabled={!connectorReady || isExecuting}
            />
          </div>
        </div>

        {/* Control Buttons */}
        <div className="btn-group-custom">
          {!isExecuting ? (
            <button
              className="btn btn-success btn-sm"
              onClick={executePath}
              disabled={!connectorReady}
            >
              Start Path
            </button>
          ) : (
            <>
              <button
                className="btn btn-danger btn-sm"
                onClick={stopExecution}
              >
                Stop
              </button>
              <div className="execution-progress">
                <small>
                  {progress.current}/{progress.total} ({Math.round((progress.current / progress.total) * 100)}%)
                </small>
              </div>
            </>
          )}
        </div>
      </div>
    </div>
  );
}
