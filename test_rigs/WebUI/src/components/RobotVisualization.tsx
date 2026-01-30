import React, { useState, useEffect, useMemo } from 'react';
import ConnManager from '../ConnManager';
import { RobotConfig } from '../App';
import { getRobotGeometry, JointAngles, CartesianPosition } from '../utils/RobotGeometry';
import { getSensorAngleTracker } from '../utils/SensorAngleTracker';
import SCARAVisualization from './SCARAVisualization';
import CartesianVisualization from './CartesianVisualization';

const connManager = ConnManager.getInstance();
const robotGeometry = getRobotGeometry();
const angleTracker = getSensorAngleTracker();

// Debug flag - set to true to log sensor readings and computed angles
const DEBUG_ANGLE_LOGGING = true;

interface RobotVisualizationProps {
  lastUpdate: number;
  robotConfig: RobotConfig | null;
}

// Placeholder geometry types - expand based on your robot configurations
const ROBOT_GEOMETRIES = [
  { id: 'xy_cartesian', name: 'XY Cartesian' },
  { id: 'singlearmscara', name: 'SingleArmSCARA' },
  { id: 'polar', name: 'Polar' },
  { id: 'delta', name: 'Delta' },
  { id: 'custom', name: 'Custom' },
];

interface PositionHistoryPoint {
  x: number;
  y: number;
  timestamp: number;
}

const DEFAULT_TRAIL_DURATION_MS = 5000; // 5 seconds default fade
const TRAIL_MAX_POINTS = 2000; // Limit number of points (increased for longer trails)

export default function RobotVisualization({ lastUpdate, robotConfig }: RobotVisualizationProps) {
  const [selectedGeometry, setSelectedGeometry] = useState<string>('singlearmscara');
  const [jointAngles, setJointAngles] = useState<JointAngles>({ joint1: 0, joint2: 0 });
  const [endEffectorPos, setEndEffectorPos] = useState<CartesianPosition>({ x: 0, y: 0 });
  const [positionHistory, setPositionHistory] = useState<PositionHistoryPoint[]>([]);
  const [trailFadeDuration, setTrailFadeDuration] = useState<number>(DEFAULT_TRAIL_DURATION_MS);
  
  // Refs for angle unwrapping to handle sensor wraparound at 0°/360° boundary
  const prevTheta2MotorRef = React.useRef<number | null>(null);
  const theta2AccumulatorRef = React.useRef<number>(0);

  // Update selected geometry when robot config is first received
  useEffect(() => {
    if (robotConfig && robotConfig.geometry) {
      const geomType = robotGeometry.getGeometryType();
      if (geomType === 'singlearmscara') {
        setSelectedGeometry('singlearmscara');
      } else if (geomType === 'xy_cartesian') {
        setSelectedGeometry('xy_cartesian');
      }
    }
  }, [robotConfig]);

  // Configure angle tracker with gear factors when robot config changes
  useEffect(() => {
    if (robotConfig && robotConfig.axes) {
      // Configure trackers for each axis with their gear factors
      const gearFactor1 = robotConfig.axes[0]?.gearFactor || 1.0;
      const gearFactor2 = robotConfig.axes[1]?.gearFactor || 1.0;
      
      // Device IDs for MT6701 (joint 1) and AS5600 (joint 2)
      angleTracker.configure('1_6', gearFactor1);
      angleTracker.configure('1_36', gearFactor2);
    }
  }, [robotConfig]);

  useEffect(() => {
    const deviceManager = connManager.getConnector().getSystemType()?.deviceMgrIF;
    if (!deviceManager) return;

    let mt6701Angle: number | null = null;
    let as5600Angle: number | null = null;
    let timestamp: number = Date.now();

    // Get MT6701 encoder data (joint 1)
    const mt6701State = deviceManager.getDeviceState('1_6');
    if (mt6701State?.deviceAttributes?.angle) {
      const values = mt6701State.deviceAttributes.angle.values;
      if (values.length > 0) {
        // Get raw angle and apply angle direction convention
        let rawAngle = values[values.length - 1];
        if (robotGeometry.shouldInvertAngleDirection()) {
          // For anti-clockwise positive: invert the angle
          rawAngle = 360 - rawAngle;
          if (rawAngle >= 360) rawAngle -= 360;
        }
        
        // Use SensorAngleTracker to handle multi-turn accumulation and gear factor
        mt6701Angle = angleTracker.update('1_6', rawAngle);
        
        // Use the timestamp from the device timeline if available
        const timestamps = mt6701State.deviceTimeline?.timestampsUs;
        if (timestamps && timestamps.length > 0) {
          // Convert microseconds to milliseconds
          timestamp = timestamps[timestamps.length - 1] / 1000;
        }
      }
    }

    // Get AS5600 encoder data (joint 2)
    const as5600State = deviceManager.getDeviceState('1_36');
    if (as5600State?.deviceAttributes?.angle) {
      const values = as5600State.deviceAttributes.angle.values;
      if (values.length > 0) {
        // Get raw angle and apply angle direction convention
        let rawAngle = values[values.length - 1];
        if (robotGeometry.shouldInvertAngleDirection()) {
          // For anti-clockwise positive: invert the angle
          rawAngle = 360 - rawAngle;
          if (rawAngle >= 360) rawAngle -= 360;
        }
        
        // Use SensorAngleTracker to handle multi-turn accumulation and gear factor
        as5600Angle = angleTracker.update('1_36', rawAngle);
        
        // If we didn't get timestamp from MT6701, use AS5600's timestamp
        if (timestamp === Date.now()) {
          const timestamps = as5600State.deviceTimeline?.timestampsUs;
          if (timestamps && timestamps.length > 0) {
            // Convert microseconds to milliseconds
            timestamp = timestamps[timestamps.length - 1] / 1000;
          }
        }
      }
    }

    // Update joint angles (already gear-corrected by SensorAngleTracker)
    const angles: JointAngles = {
      joint1: mt6701Angle ?? 0,
      joint2: as5600Angle ?? 0,
    };
    setJointAngles(angles);

    // Calculate forward kinematics based on selected geometry
    // Joint angles are already gear-corrected by SensorAngleTracker
    const sensorAngle1 = angles.joint1;
    const sensorAngle2 = angles.joint2;
    
    // Kinematic angles: Apply same transformation as firmware
    // Firmware adds originTheta2OffsetDegrees (180°) to joint2
    // At origin (0,0): θ1 ≈ 0°, θ2 ≈ 180° (arms doubled back)
    const theta1 = robotGeometry.normalizeAngle(sensorAngle1);
    const theta2 = robotGeometry.normalizeAngle(sensorAngle2 + 180);
    
    const kinematicAngles: JointAngles = {
      joint1: theta1,
      joint2: theta2
    };
    
    const position = robotGeometry.forwardKinematics(kinematicAngles);
    
    // Debug logging
    if (DEBUG_ANGLE_LOGGING) {
      // Show motor shaft angles and joint angles (after gear correction)
      const mt6701Info = angleTracker.getDebugInfo('1_6');
      const as5600Info = angleTracker.getDebugInfo('1_36');
      console.log(`[ANGLE_DEBUG] MT6701: raw=${mt6701Info?.prevRaw?.toFixed(1)}° shaft=${mt6701Info?.accumulated?.toFixed(1)}° joint=${angles.joint1.toFixed(2)}° (gear=${mt6701Info?.gearFactor}) | AS5600: raw=${as5600Info?.prevRaw?.toFixed(1)}° shaft=${as5600Info?.accumulated?.toFixed(1)}° joint=${angles.joint2.toFixed(2)}° (gear=${as5600Info?.gearFactor}) | θ1=${theta1.toFixed(2)}° θ2=${theta2.toFixed(2)}° | Pos: (${position.x.toFixed(2)}, ${position.y.toFixed(2)})mm`);
    }
    setEndEffectorPos(position);
    
    // Add to position history
    setPositionHistory((prev) => {
      const newPoint: PositionHistoryPoint = {
        x: position.x,
        y: position.y,
        timestamp: timestamp,
      };
      
      // Filter out old points and add new one
      // Only filter by time if fade is enabled (trailFadeDuration > 0)
      const filtered = trailFadeDuration > 0 
        ? prev.filter((p) => timestamp - p.timestamp < trailFadeDuration)
        : prev;
      
      // Only add if position has changed significantly (more than 0.5mm)
      const lastPoint = filtered[filtered.length - 1];
      if (!lastPoint || 
          Math.abs(lastPoint.x - position.x) > 0.5 || 
          Math.abs(lastPoint.y - position.y) > 0.5) {
        const updated = [...filtered, newPoint];
        
        // Limit total points
        if (updated.length > TRAIL_MAX_POINTS) {
          return updated.slice(updated.length - TRAIL_MAX_POINTS);
        }
        return updated;
      }
      
      return filtered;
    });
  }, [lastUpdate, selectedGeometry]);

  // Calculate workspace radius for viewBox scaling
  const workspaceRadius = useMemo(() => {
    if (selectedGeometry === 'scara' || selectedGeometry === 'sandbot7') {
      const scaraConfig = robotGeometry.getSCARAConfig();
      const maxReach = robotConfig?.maxRadiusMM || (scaraConfig.link1Length + scaraConfig.link2Length);
      return maxReach * robotGeometry.getScale();
    }
    // For other geometries, use a default
    return 200;
  }, [selectedGeometry, robotConfig]);

  // Calculate viewBox with 10% margin
  const viewBoxSize = useMemo(() => {
    const size = workspaceRadius * 2 * 1.1; // 10% margin
    return size;
  }, [workspaceRadius]);

  // Convert world coordinates to SVG coordinates
  const worldToSVG = (x: number, y: number) => {
    const center = viewBoxSize / 2;
    return {
      x: center + x,  // Center at viewBox center
      y: center - y,  // Flip Y axis (SVG Y increases downward)
    };
  };

  // Render position trail
  const renderTrail = () => {
    if (positionHistory.length < 2) return null;
    
    // Use the latest point's timestamp as "now" instead of Date.now()
    const latestTimestamp = positionHistory.length > 0 ? positionHistory[positionHistory.length - 1].timestamp : Date.now();
    
    // Create path segments with varying opacity based on age
    const lines = positionHistory.map((point, index) => {
      if (index === 0) return null; // Skip first point (no line to draw)
      
      const prevPoint = positionHistory[index - 1];
      const age = latestTimestamp - point.timestamp;
      // If fade is disabled (duration = 0), use full opacity
      const opacity = trailFadeDuration > 0 
        ? Math.max(0, 1 - (age / trailFadeDuration))
        : 1;
      
      // Skip if opacity is too low (trail has expired)
      if (opacity < 0.01) return null;
      
      // Skip drawing if there's a large time gap (>1 second) between consecutive points
      // This prevents drawing spurious lines when trail restarts after a gap
      const timeDiff = point.timestamp - prevPoint.timestamp;
      if (timeDiff > 1000) return null;
      
      // Skip if there's a large spatial gap between consecutive points (>50mm)
      // This prevents drawing spurious lines from invalid initial positions
      const distSq = Math.pow(point.x - prevPoint.x, 2) + Math.pow(point.y - prevPoint.y, 2);
      if (distSq > 2500) return null; // 50mm * 50mm = 2500
      
      // Trail positions are already in world coordinates (unscaled mm)
      // Apply the same scaling used by the robot rendering
      // Use robotGeometry.scale for all geometries to ensure consistency
      const scale = robotGeometry.getScale();
      const svgPoint = worldToSVG(point.x * scale, point.y * scale);
      const svgPrevPoint = worldToSVG(prevPoint.x * scale, prevPoint.y * scale);
      
      return (
        <line
          key={`trail-${point.timestamp}-${index}`}
          x1={svgPrevPoint.x}
          y1={svgPrevPoint.y}
          x2={svgPoint.x}
          y2={svgPoint.y}
          stroke="#ffeb3b"
          strokeWidth="2"
          strokeLinecap="round"
          opacity={opacity * 0.8}
        />
      );
    }).filter(line => line !== null);
    
    return lines.length > 0 ? <g>{lines}</g> : null;
  };

  // Render robot based on selected geometry
  const renderRobot = () => {
    if (selectedGeometry === 'xy_cartesian') {
      return (
        <CartesianVisualization
          jointAngles={jointAngles}
          viewBoxSize={viewBoxSize}
          worldToSVG={worldToSVG}
        />
      );
    }
    
    if (selectedGeometry === 'singlearmscara') {
      return (
        <SCARAVisualization
          jointAngles={jointAngles}
          robotConfig={robotConfig}
          viewBoxSize={viewBoxSize}
          worldToSVG={worldToSVG}
        />
      );
    }
    
    // Placeholder for other geometries
    return (
      <text 
        x="200" 
        y="200" 
        textAnchor="middle" 
        fill="#666" 
        fontSize="14"
      >
        {ROBOT_GEOMETRIES.find(g => g.id === selectedGeometry)?.name} - Not yet implemented
      </text>
    );
  };

  return (
    <div className="panel visualization-panel">
      <h2>Robot Visualization</h2>
      
      <div className="control-group">
        <select 
          id="geometry-select"
          className="form-select"
          value={selectedGeometry}
          onChange={(e) => setSelectedGeometry(e.target.value)}
        >
          {ROBOT_GEOMETRIES.map((geom) => (
            <option key={geom.id} value={geom.id}>
              {geom.name}
            </option>
          ))}
        </select>
      </div>

      <div className="visualization-container">
        <svg 
          viewBox={`0 0 ${viewBoxSize} ${viewBoxSize}`}
          className="robot-canvas"
          style={{ 
            border: '1px solid #444',
            borderRadius: '4px',
            background: '#1a1a1a' 
          }}
        >
          {/* Grid lines */}
          <g opacity="0.2">
            {[-0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75].map((offset) => {
              const center = viewBoxSize / 2;
              const gridOffset = offset * workspaceRadius;
              return (
                <React.Fragment key={`grid-${offset}`}>
                  <line
                    x1={center + gridOffset}
                    y1={viewBoxSize * 0.1}
                    x2={center + gridOffset}
                    y2={viewBoxSize * 0.9}
                    stroke="#555"
                    strokeWidth="0.5"
                  />
                  <line
                    x1={viewBoxSize * 0.1}
                    y1={center - gridOffset}
                    x2={viewBoxSize * 0.9}
                    y2={center - gridOffset}
                    stroke="#555"
                    strokeWidth="0.5"
                  />
                </React.Fragment>
              );
            })}
          </g>

          {/* Coordinate system */}
          <g>
            {/* X axis */}
            <line 
              x1={viewBoxSize * 0.1} 
              y1={viewBoxSize / 2} 
              x2={viewBoxSize * 0.9} 
              y2={viewBoxSize / 2} 
              stroke="#666" 
              strokeWidth="2" 
            />
            <polygon 
              points={`${viewBoxSize * 0.9},${viewBoxSize / 2} ${viewBoxSize * 0.9 - 5},${viewBoxSize / 2 - 3} ${viewBoxSize * 0.9 - 5},${viewBoxSize / 2 + 3}`} 
              fill="#666" 
            />
            <text 
              x={viewBoxSize * 0.92} 
              y={viewBoxSize / 2 + 5} 
              fill="#888" 
              fontSize="14" 
              fontWeight="bold"
            >
              X
            </text>
            
            {/* Y axis */}
            <line 
              x1={viewBoxSize / 2} 
              y1={viewBoxSize * 0.9} 
              x2={viewBoxSize / 2} 
              y2={viewBoxSize * 0.1} 
              stroke="#666" 
              strokeWidth="2" 
            />
            <polygon 
              points={`${viewBoxSize / 2},${viewBoxSize * 0.1} ${viewBoxSize / 2 - 3},${viewBoxSize * 0.1 + 5} ${viewBoxSize / 2 + 3},${viewBoxSize * 0.1 + 5}`} 
              fill="#666" 
            />
            <text 
              x={viewBoxSize / 2 + 5} 
              y={viewBoxSize * 0.08} 
              fill="#888" 
              fontSize="14" 
              fontWeight="bold"
            >
              Y
            </text>
            
            {/* Origin */}
            <circle cx={viewBoxSize / 2} cy={viewBoxSize / 2} r="3" fill="#888" />
            <text 
              x={viewBoxSize / 2 + 10} 
              y={viewBoxSize / 2 + 15} 
              fill="#888" 
              fontSize="12"
            >
              (0,0)
            </text>
          </g>

          {/* Robot */}
          {renderTrail()}
          {renderRobot()}
        </svg>
      </div>

      <div className="visualization-info">
        <div className="info-grid">
          {selectedGeometry === 'singlearmscara' ? (
            <>
              <span className="info-label">Geometry:</span>
              <span className="info-value">
                {ROBOT_GEOMETRIES.find(g => g.id === selectedGeometry)?.name}
                {(() => {
                  const scaraConfig = robotGeometry.getSCARAConfig();
                  return robotConfig ? ` (${scaraConfig.link1Length.toFixed(0)}mm, ${scaraConfig.link2Length.toFixed(0)}mm, max ${robotConfig.maxRadiusMM.toFixed(0)}mm, θ2 offset ${robotConfig.originTheta2OffsetDegrees.toFixed(0)}°)` : '';
                })()}
              </span>
              
              <span className="info-label">Joint 1:</span>
              <span className="info-value">
                {jointAngles.joint1.toFixed(1)}° 
                {(() => {
                  const info = angleTracker.getDebugInfo('1_6');
                  return info ? ` (${info.rotations} rot, gear:${info.gearFactor})` : '';
                })()}
              </span>
              
              <span className="info-label">Joint 2:</span>
              <span className="info-value">
                {jointAngles.joint2.toFixed(1)}°
                {(() => {
                  const info = angleTracker.getDebugInfo('1_36');
                  return info ? ` (${info.rotations} rot, gear:${info.gearFactor})` : '';
                })()}
              </span>
            </>
          ) : selectedGeometry === 'xy_cartesian' ? (
            <>
              <span className="info-label">Geometry:</span>
              <span className="info-value">
                {ROBOT_GEOMETRIES.find(g => g.id === selectedGeometry)?.name}
                {(() => {
                  const cartesianConfig = robotGeometry.getCartesianConfig();
                  return robotConfig && robotConfig.axes && robotConfig.axes.length >= 2 
                    ? ` (X: ${cartesianConfig.xRange.toFixed(0)}mm/rot, Y: ${cartesianConfig.yRange.toFixed(0)}mm/rot)` 
                    : '';
                })()}
              </span>
              
              <span className="info-label">Axes Angles:</span>
              <span className="info-value">
                A0: {jointAngles.joint1.toFixed(1)}° (MT6701), A1: {jointAngles.joint2.toFixed(1)}° (AS5600)
              </span>
              
              <span className="info-label">Position:</span>
              <span className="info-value">
                X: {endEffectorPos.x.toFixed(1)}mm, Y: {endEffectorPos.y.toFixed(1)}mm
              </span>
            </>
          ) : (
            <>
              <span className="info-label">Geometry:</span>
              <span className="info-value">
                {ROBOT_GEOMETRIES.find(g => g.id === selectedGeometry)?.name}
              </span>
              
              <span className="info-label">Joint 1 (MT6701):</span>
              <span className="info-value">{jointAngles.joint1.toFixed(1)}°</span>
              
              <span className="info-label">Joint 2 (AS5600):</span>
              <span className="info-value">{jointAngles.joint2.toFixed(1)}°</span>
            </>
          )}
          
          <span className="info-label">Angle Tracking:</span>
          <span className="info-value">
            <button
              onClick={() => {
                angleTracker.reset();
                setPositionHistory([]);
              }}
              style={{
                padding: '4px 12px',
                fontSize: '0.85em',
                border: '1px solid #ff6b6b',
                borderRadius: '3px',
                background: '#fff5f5',
                color: '#c92a2a',
                cursor: 'pointer',
                fontWeight: '500'
              }}
              title="Reset multi-turn angle tracking to zero"
            >
              Reset Angles
            </button>
          </span>
          
          <span className="info-label">Trail Fade:</span>
          <span className="info-value" style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
            <input
              type="range"
              min="0"
              max="100000"
              step="500"
              value={trailFadeDuration}
              onChange={(e) => setTrailFadeDuration(parseInt(e.target.value))}
              style={{ flex: 1, minWidth: '80px' }}
              title={trailFadeDuration === 0 ? 'No fade' : `${(trailFadeDuration / 1000).toFixed(1)}s`}
            />
            <span style={{ minWidth: '35px', fontSize: '0.85em' }}>
              {trailFadeDuration === 0 ? 'Off' : `${(trailFadeDuration / 1000).toFixed(1)}s`}
            </span>
            <button
              onClick={() => setPositionHistory([])}
              style={{
                padding: '2px 8px',
                fontSize: '0.8em',
                border: '1px solid #ccc',
                borderRadius: '3px',
                background: '#f8f9fa',
                cursor: 'pointer',
                whiteSpace: 'nowrap'
              }}
              title="Clear trail"
            >
              Clear
            </button>
          </span>
        </div>
      </div>
    </div>
  );
}
