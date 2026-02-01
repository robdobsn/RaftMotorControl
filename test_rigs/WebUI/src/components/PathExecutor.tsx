import React, { useState, useRef, useEffect } from 'react';
import ConnManager from '../ConnManager';
import { RobotConfig, ExpectedPathPoint } from '../App';
import { getRobotGeometry } from '../utils/RobotGeometry';

const connManager = ConnManager.getInstance();
const robotGeometry = getRobotGeometry();

interface PathExecutorProps {
  motorConnectionReady: boolean;
  robotConfig: RobotConfig | null;
  onPathChange?: (path: ExpectedPathPoint[]) => void;
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

export default function PathExecutor({ motorConnectionReady, robotConfig, onPathChange }: PathExecutorProps) {
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
        // Close the circle by adding the first point at the end
        if (points.length > 0) {
          points.push({ ...points[0] });
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
        // Close the rectangle by adding the first point at the end
        if (points.length > 0) {
          points.push({ ...points[0] });
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
        // Close the lissajous curve
        if (points.length > 0) {
          points.push({ ...points[0] });
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
        // Close the lissajous curve
        if (points.length > 0) {
          points.push({ ...points[0] });
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
        // Close the lissajous curve
        if (points.length > 0) {
          points.push({ ...points[0] });
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
        // Close the figure-8
        if (points.length > 0) {
          points.push({ ...points[0] });
        }
        break;

      case 'star':
        // 5-pointed star: distribute points evenly across all 10 edges
        const starVertices = 5;
        const outerRadius = 0.5;
        const innerRadius = 0.2;
        
        // Generate the 10 vertices of the star (alternating outer/inner)
        const vertices: PathPoint[] = [];
        for (let i = 0; i < starVertices * 2; i++) {
          const angle = (i / (starVertices * 2)) * 2 * Math.PI - Math.PI / 2;
          const radius = i % 2 === 0 ? outerRadius : innerRadius;
          vertices.push({
            a0: 0.5 + radius * Math.cos(angle),
            a1: 0.5 + radius * Math.sin(angle),
          });
        }
        
        // Interpolate points along each edge
        const numEdges = vertices.length;
        const pointsPerEdge = Math.max(1, Math.floor(numPoints / numEdges));
        
        for (let edge = 0; edge < numEdges; edge++) {
          const start = vertices[edge];
          const end = vertices[(edge + 1) % numEdges];
          
          // Don't include the last point of each edge (it's the start of the next)
          for (let p = 0; p < pointsPerEdge; p++) {
            const t = p / pointsPerEdge;
            points.push({
              a0: start.a0 + t * (end.a0 - start.a0),
              a1: start.a1 + t * (end.a1 - start.a1),
            });
          }
        }
        
        // Close the star by adding the first point at the end
        if (points.length > 0) {
          points.push({ ...points[0] });
        }
        break;
    }

    return points;
  };

  // Get path-specific information
  const getPathInfo = (patternType: PatternType): string => {
    const { width, height } = robotGeometry.getPatternWorkspaceDimensions();
    
    switch (patternType) {
      case 'circle':
        const radius = Math.min(width, height) / 2;
        return `Circle: Radius ${radius.toFixed(1)}mm, Center at origin (0, 0)`;
      case 'rectangle':
        return `Rectangle: ${width.toFixed(1)}mm x ${height.toFixed(1)}mm, Center at origin (0, 0)`;
      case 'figure8':
        return `Figure-8: Width ${width.toFixed(1)}mm, Height ${height.toFixed(1)}mm, Center at origin (0, 0)`;
      case 'star':
        const starRadius = Math.min(width, height) / 2;
        return `Star: Radius ${starRadius.toFixed(1)}mm (5-pointed), Center at origin (0, 0)`;
      case 'lissajous1':
        return `Lissajous (1:1): ${width.toFixed(1)}mm x ${height.toFixed(1)}mm, δ=π/2`;
      case 'lissajous2':
        return `Lissajous (3:2): ${width.toFixed(1)}mm x ${height.toFixed(1)}mm, δ=0`;
      case 'lissajous3':
        return `Lissajous (5:4): ${width.toFixed(1)}mm x ${height.toFixed(1)}mm, δ=π/2`;
      default:
        return `Size: ${width.toFixed(1)}mm x ${height.toFixed(1)}mm`;
    }
  };

  // Scale normalized points to fit within robot workspace
  const scalePathToWorkspace = (normalizedPoints: PathPoint[]): PathPoint[] => {
    // Use pattern workspace dimensions which account for circular SCARA workspace
    const { width, height } = robotGeometry.getPatternWorkspaceDimensions();

    return normalizedPoints.map(p => ({
      a0: (p.a0 - 0.5) * width,
      a1: (p.a1 - 0.5) * height,
    }));
  };

  // Update expected path visualization when pattern or points change
  useEffect(() => {
    const normalizedPath = generateNormalizedPath(selectedPattern, pointsPerPath);
    const scaledPath = scalePathToWorkspace(normalizedPath);
    const expectedPath: ExpectedPathPoint[] = scaledPath.map(p => ({ x: p.a0, y: p.a1 }));
    onPathChange?.(expectedPath);
  }, [selectedPattern, pointsPerPath, onPathChange]);

  // Execute a single movement command
  const sendMoveCommand = async (point: PathPoint, speedValue: number): Promise<void> => {
    // Build command using modern pos=[x,y] array format (backend auto-detects arrays/numbers)
    // Speed is sent as percentage (100 = 100% of max speed)
    const speedPercent = Math.round(speedValue * 100);
    const command = `motors?cmd=motion&mode=abs&speed=${speedPercent}&pos=[${point.a0.toFixed(2)},${point.a1.toFixed(2)}]&imm=0&nosplit=1`;
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

      // Return to origin after completing all repetitions
      if (!executionRef.current.shouldStop) {
        await new Promise(resolve => setTimeout(resolve, delayMs));
        await sendMoveCommand({ a0: 0, a1: 0 }, speed);
        console.log('Returned to origin (0, 0)');
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
      await connManager.getMotorConnector().sendRICRESTMsg('motors?cmd=stop&disableMotors=true', {});
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
              max="1000"
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

        {/* Path Information Display */}
        {connectorReady && selectedPattern && (
          <div className="path-info-display">
            <div className="info-section">
              <strong>Workspace:</strong> {robotGeometry.getWorkspaceDimensions().description}
            </div>
            <div className="info-section">
              <strong>Path Details:</strong> {getPathInfo(selectedPattern)}
            </div>
            {selectedPatternDef && (
              <div className="info-section">
                <strong>Description:</strong> {selectedPatternDef.description}
              </div>
            )}
          </div>
        )}

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
