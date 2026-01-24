import React, { useState, useEffect, useMemo } from 'react';
import ConnManager from '../ConnManager';
import { RobotConfig } from '../App';

const connManager = ConnManager.getInstance();

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

// Robot configurations
const SINGLE_ARM_SCARA_CONFIG = {
  link1Length: 150, // Length of first link in mm
  link2Length: 150, // Length of second link in mm
  scale: 0.8,       // Scale factor for display
  invertJoint1: true,  // Joint 1 (shoulder) rotation direction - INVERTED
  invertJoint2: false, // Joint 2 (elbow) rotation direction - NOT inverted
  invertAngleDirection: true, // Anti-clockwise positive (0° at +X axis, positive CCW)
};

const XY_CARTESIAN_CONFIG = {
  // Map 360 degrees to a linear range (e.g., 200mm travel)
  xRange: 200,      // mm of X travel
  yRange: 200,      // mm of Y travel
  scale: 0.8,       // Scale factor for display
};

interface JointAngles {
  joint1: number; // degrees
  joint2: number; // degrees
}

interface CartesianPosition {
  x: number; // mm
  y: number; // mm
}

interface PositionHistoryPoint {
  x: number;
  y: number;
  timestamp: number;
}

const TRAIL_DURATION_MS = 5000; // 5 seconds
const TRAIL_MAX_POINTS = 100; // Limit number of points

// XY Cartesian: Direct angle to position mapping
function calculateXYCartesian(angles: JointAngles): CartesianPosition {
  const { xRange, yRange } = XY_CARTESIAN_CONFIG;
  
  // Map angle (0-360°) to linear position
  // Center at 0, so range goes from -xRange/2 to +xRange/2
  const x = ((angles.joint1 / 360) * xRange) - (xRange / 2);
  const y = ((angles.joint2 / 360) * yRange) - (yRange / 2);
  
  return { x, y };
}

// SCARA: Forward kinematics for 2-joint planar arm
function calculateSCARAKinematics(angles: JointAngles, config = SINGLE_ARM_SCARA_CONFIG): CartesianPosition {
  const { link1Length, link2Length, invertJoint1, invertJoint2 } = config;
  
  // Convert to radians and apply inversion based on config
  const theta1 = ((invertJoint1 ? -angles.joint1 : angles.joint1) * Math.PI) / 180;
  const theta2 = ((invertJoint2 ? -angles.joint2 : angles.joint2) * Math.PI) / 180;
  
  // Calculate end effector position
  // Both theta1 and theta2 are measured from the positive X axis
  const x = link1Length * Math.cos(theta1) + link2Length * Math.cos(theta2);
  const y = link1Length * Math.sin(theta1) + link2Length * Math.sin(theta2);
  
  return { x, y };
}

// Calculate SCARA joint positions for rendering
function calculateSCARAJointPositions(angles: JointAngles, config = SINGLE_ARM_SCARA_CONFIG) {
  const { link1Length, link2Length, scale, invertJoint1, invertJoint2 } = config;
  
  // Convert to radians and apply inversion based on config
  const theta1 = ((invertJoint1 ? -angles.joint1 : angles.joint1) * Math.PI) / 180;
  const theta2 = ((invertJoint2 ? -angles.joint2 : angles.joint2) * Math.PI) / 180;
  
  // Base at origin
  const base = { x: 0, y: 0 };
  
  // Joint 1 (elbow) position
  const joint1 = {
    x: link1Length * Math.cos(theta1) * scale,
    y: link1Length * Math.sin(theta1) * scale,
  };
  
  // End effector position
  // Both theta1 and theta2 are measured from the positive X axis
  const endEffector = {
    x: (link1Length * Math.cos(theta1) + link2Length * Math.cos(theta2)) * scale,
    y: (link1Length * Math.sin(theta1) + link2Length * Math.sin(theta2)) * scale,
  };
  
  return { base, joint1, endEffector };
}

export default function RobotVisualization({ lastUpdate, robotConfig }: RobotVisualizationProps) {
  const [selectedGeometry, setSelectedGeometry] = useState<string>('singlearmscara');
  const [jointAngles, setJointAngles] = useState<JointAngles>({ joint1: 0, joint2: 0 });
  const [endEffectorPos, setEndEffectorPos] = useState<CartesianPosition>({ x: 0, y: 0 });
  const [positionHistory, setPositionHistory] = useState<PositionHistoryPoint[]>([]);
  
  // Track rotations for multi-turn encoders (needed for geared systems)
  const [joint1Rotations, setJoint1Rotations] = useState<number>(0);
  const [joint2Rotations, setJoint2Rotations] = useState<number>(0);
  const [prevJoint1Angle, setPrevJoint1Angle] = useState<number | null>(null);
  const [prevJoint2Angle, setPrevJoint2Angle] = useState<number | null>(null);

  // Update selected geometry when robot config is first received
  useEffect(() => {
    if (robotConfig && robotConfig.geometry) {
      const geomLower = robotConfig.geometry.toLowerCase();
      if (geomLower.includes('singlearmscara') || geomLower.includes('scara')) {
        setSelectedGeometry('singlearmscara');
      } else if (geomLower.includes('cartesian') || geomLower.includes('xyz')) {
        setSelectedGeometry('xy_cartesian');
      }
    }
  }, [robotConfig]);

  // Determine which config to use based on selected geometry and available robot config
  // Memoized to prevent infinite re-renders
  const activeConfig = useMemo(() => {
    if (selectedGeometry === 'singlearmscara') {
      // Use robot config for SingleArmSCARA if available
      if (robotConfig && robotConfig.geometry.toLowerCase().includes('scara')) {
        const config = {
          link1Length: robotConfig.arm1LengthMM,
          link2Length: robotConfig.arm2LengthMM,
          scale: 0.8,
          invertJoint1: true,  // SingleArmSCARA inverts joint 1
          invertJoint2: false, // SingleArmSCARA does not invert joint 2
          invertAngleDirection: true, // SingleArmSCARA uses anti-clockwise positive
        };
        console.log('Using robot config for SingleArmSCARA:', config);
        return config;
      }
      console.log('Using default SingleArmSCARA config:', SINGLE_ARM_SCARA_CONFIG);
      return SINGLE_ARM_SCARA_CONFIG;
    }
    // For other geometries, use defaults
    return SINGLE_ARM_SCARA_CONFIG;
  }, [selectedGeometry, robotConfig]);

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
        if (activeConfig.invertAngleDirection) {
          // For anti-clockwise positive: invert the angle
          rawAngle = 360 - rawAngle;
          if (rawAngle >= 360) rawAngle -= 360;
        }
        
        // Track rotations: detect 360° wrapping
        if (prevJoint1Angle !== null) {
          // Transition from >270° to <90° means we crossed 0° going forward
          if (prevJoint1Angle > 270 && rawAngle < 90) {
            setJoint1Rotations(prev => prev + 1);
          }
          // Transition from <90° to >270° means we crossed 0° going backward
          else if (prevJoint1Angle < 90 && rawAngle > 270) {
            setJoint1Rotations(prev => prev - 1);
          }
        }
        setPrevJoint1Angle(rawAngle);
        
        // Calculate absolute angle including rotations
        mt6701Angle = joint1Rotations * 360 + rawAngle;
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
        if (activeConfig.invertAngleDirection) {
          // For anti-clockwise positive: invert the angle
          rawAngle = 360 - rawAngle;
          if (rawAngle >= 360) rawAngle -= 360;
        }
        
        // Track rotations: detect 360° wrapping
        if (prevJoint2Angle !== null) {
          // Transition from >270° to <90° means we crossed 0° going forward
          if (prevJoint2Angle > 270 && rawAngle < 90) {
            setJoint2Rotations(prev => prev + 1);
          }
          // Transition from <90° to >270° means we crossed 0° going backward
          else if (prevJoint2Angle < 90 && rawAngle > 270) {
            setJoint2Rotations(prev => prev - 1);
          }
        }
        setPrevJoint2Angle(rawAngle);
        
        // Calculate absolute angle including rotations
        as5600Angle = joint2Rotations * 360 + rawAngle;
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

    // Update joint angles
    const angles: JointAngles = {
      joint1: mt6701Angle ?? 0,
      joint2: as5600Angle ?? 0,
    };
    setJointAngles(angles);

    // Calculate forward kinematics based on selected geometry
    let position: CartesianPosition;
    if (selectedGeometry === 'xy_cartesian') {
      position = calculateXYCartesian(angles);
    } else if (selectedGeometry === 'singlearmscara') {
      // SingleArmSCARA has 1:3 gearing - divide measured angles by 3
      const gearRatio = 3;
      const gearAdjustedAngles = {
        joint1: angles.joint1 / gearRatio,
        joint2: angles.joint2 / gearRatio
      };
      position = calculateSCARAKinematics(gearAdjustedAngles, activeConfig);
    } else {
      position = { x: 0, y: 0 };
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
      const filtered = prev.filter((p) => timestamp - p.timestamp < TRAIL_DURATION_MS);
      
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
  }, [lastUpdate, selectedGeometry, activeConfig, joint1Rotations, joint2Rotations, prevJoint1Angle, prevJoint2Angle]);

  // Calculate workspace radius for viewBox scaling
  const workspaceRadius = useMemo(() => {
    if (selectedGeometry === 'scara' || selectedGeometry === 'sandbot7') {
      const maxReach = robotConfig?.maxRadiusMM || (activeConfig.link1Length + activeConfig.link2Length);
      return maxReach * activeConfig.scale;
    }
    // For other geometries, use a default
    return 200;
  }, [selectedGeometry, robotConfig, activeConfig]);

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
      const opacity = Math.max(0, 1 - (age / TRAIL_DURATION_MS));
      
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
      const scale = selectedGeometry === 'xy_cartesian' ? XY_CARTESIAN_CONFIG.scale : activeConfig.scale;
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
      const { xRange, yRange, scale } = XY_CARTESIAN_CONFIG;
      const position = calculateXYCartesian(jointAngles);
      
      // Scale position for display
      const displayPos = worldToSVG(position.x * scale, position.y * scale);
      
      // Draw X slider
      const xSliderY = 280;
      const xSliderStart = worldToSVG(-xRange/2 * scale, 0);
      const xSliderEnd = worldToSVG(xRange/2 * scale, 0);
      const xPos = worldToSVG(position.x * scale, 0);
      
      // Draw Y slider
      const ySliderX = 280;
      const ySliderStart = worldToSVG(0, -yRange/2 * scale);
      const ySliderEnd = worldToSVG(0, yRange/2 * scale);
      const yPos = worldToSVG(0, position.y * scale);
      
      return (
        <g>
          {/* X axis slider track */}
          <line
            x1={xSliderStart.x}
            y1={xSliderY}
            x2={xSliderEnd.x}
            y2={xSliderY}
            stroke="#4a9eff"
            strokeWidth="3"
            opacity="0.3"
          />
          
          {/* Y axis slider track */}
          <line
            x1={ySliderX}
            y1={ySliderStart.y}
            x2={ySliderX}
            y2={ySliderEnd.y}
            stroke="#ff6384"
            strokeWidth="3"
            opacity="0.3"
          />
          
          {/* X axis position marker */}
          <g>
            <line
              x1={xPos.x}
              y1={xSliderY - 10}
              x2={xPos.x}
              y2={xSliderY + 10}
              stroke="#4a9eff"
              strokeWidth="4"
              strokeLinecap="round"
            />
            <circle cx={xPos.x} cy={xSliderY} r="6" fill="#4a9eff" />
          </g>
          
          {/* Y axis position marker */}
          <g>
            <line
              x1={ySliderX - 10}
              y1={yPos.y}
              x2={ySliderX + 10}
              y2={yPos.y}
              stroke="#ff6384"
              strokeWidth="4"
              strokeLinecap="round"
            />
            <circle cx={ySliderX} cy={yPos.y} r="6" fill="#ff6384" />
          </g>
          
          {/* Position lines from sliders to end effector */}
          <line
            x1={xPos.x}
            y1={xSliderY}
            x2={displayPos.x}
            y2={displayPos.y}
            stroke="#4a9eff"
            strokeWidth="1"
            strokeDasharray="3,3"
            opacity="0.5"
          />
          <line
            x1={ySliderX}
            y1={yPos.y}
            x2={displayPos.x}
            y2={displayPos.y}
            stroke="#ff6384"
            strokeWidth="1"
            strokeDasharray="3,3"
            opacity="0.5"
          />
          
          {/* End effector */}
          <circle cx={displayPos.x} cy={displayPos.y} r="8" fill="#4caf50" stroke="#fff" strokeWidth="2" />
          <circle cx={displayPos.x} cy={displayPos.y} r="3" fill="#fff" />
        </g>
      );
    }
    
    if (selectedGeometry === 'singlearmscara') {
      // For SingleArmSCARA, apply 1:3 gearing to joint angles for display
      const displayAngles = { joint1: jointAngles.joint1 / 3, joint2: jointAngles.joint2 / 3 };
      const positions = calculateSCARAJointPositions(displayAngles, activeConfig);
      
      const base = worldToSVG(positions.base.x, positions.base.y);
      const joint1 = worldToSVG(positions.joint1.x, positions.joint1.y);
      const endEffector = worldToSVG(positions.endEffector.x, positions.endEffector.y);
      
      return (
        <g>
          {/* Link 1 */}
          <line
            x1={base.x}
            y1={base.y}
            x2={joint1.x}
            y2={joint1.y}
            stroke="#4a9eff"
            strokeWidth="4"
            strokeLinecap="round"
          />
          
          {/* Link 2 */}
          <line
            x1={joint1.x}
            y1={joint1.y}
            x2={endEffector.x}
            y2={endEffector.y}
            stroke="#ff6384"
            strokeWidth="4"
            strokeLinecap="round"
          />
          
          {/* Base */}
          <circle cx={base.x} cy={base.y} r="6" fill="#333" stroke="#4a9eff" strokeWidth="2" />
          
          {/* Joint 1 */}
          <circle cx={joint1.x} cy={joint1.y} r="5" fill="#333" stroke="#4a9eff" strokeWidth="2" />
          
          {/* End Effector */}
          <circle cx={endEffector.x} cy={endEffector.y} r="6" fill="#ff6384" stroke="#fff" strokeWidth="2" />
          
          {/* Workspace circle (max reach) */}
          <circle
            cx={base.x}
            cy={base.y}
            r={(robotConfig?.maxRadiusMM || (activeConfig.link1Length + activeConfig.link2Length)) * activeConfig.scale}
            fill="none"
            stroke="#333"
            strokeWidth="1"
            strokeDasharray="5,5"
          />
        </g>
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
                {robotConfig && ` (${activeConfig.link1Length.toFixed(0)}mm, ${activeConfig.link2Length.toFixed(0)}mm, max ${robotConfig.maxRadiusMM.toFixed(0)}mm, θ2 offset ${robotConfig.originTheta2OffsetDegrees.toFixed(0)}°)`}
              </span>
              
              <span className="info-label">Angle 1:</span>
              <span className="info-value">
                MT6701 {jointAngles.joint1.toFixed(1)}°, Joint {(jointAngles.joint1 / 3).toFixed(1)}°
              </span>
              
              <span className="info-label">Angle 2:</span>
              <span className="info-value">
                AS5600 {jointAngles.joint2.toFixed(1)}°, Joint {(jointAngles.joint2 / 3).toFixed(1)}°
              </span>
            </>
          ) : (
            <>
              <span className="info-label">Geometry:</span>
              <span className="info-value">
                {ROBOT_GEOMETRIES.find(g => g.id === selectedGeometry)?.name}
              </span>
              
              <span className="info-label">Arm 1 Length:</span>
              <span className="info-value">{activeConfig.link1Length.toFixed(1)}mm</span>
              
              <span className="info-label">Arm 2 Length:</span>
              <span className="info-value">{activeConfig.link2Length.toFixed(1)}mm</span>
              
              {robotConfig && (
                <>
                  <span className="info-label">Max Radius:</span>
                  <span className="info-value">{robotConfig.maxRadiusMM.toFixed(1)}mm</span>
                  
                  <span className="info-label">Theta2 Offset:</span>
                  <span className="info-value">{robotConfig.originTheta2OffsetDegrees.toFixed(1)}°</span>
                </>
              )}
              
              <span className="info-label">Joint 1 (MT6701):</span>
              <span className="info-value">{jointAngles.joint1.toFixed(1)}°</span>
              
              <span className="info-label">Joint 2 (AS5600):</span>
              <span className="info-value">{jointAngles.joint2.toFixed(1)}°</span>
            </>
          )}
          
          <span className="info-label">End Effector:</span>
          <span className="info-value">
            X: {endEffectorPos.x.toFixed(1)}mm, Y: {endEffectorPos.y.toFixed(1)}mm
          </span>
        </div>
      </div>
    </div>
  );
}
