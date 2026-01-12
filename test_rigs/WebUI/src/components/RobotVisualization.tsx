import React, { useState, useEffect } from 'react';
import ConnManager from '../ConnManager';

const connManager = ConnManager.getInstance();

interface RobotVisualizationProps {
  lastUpdate: number;
}

// Placeholder geometry types - expand based on your robot configurations
const ROBOT_GEOMETRIES = [
  { id: 'xy_cartesian', name: 'XY Cartesian' },
  { id: 'scara', name: 'SCARA' },
  { id: 'polar', name: 'Polar' },
  { id: 'delta', name: 'Delta' },
  { id: 'custom', name: 'Custom' },
];

// Robot configurations
const SCARA_CONFIG = {
  link1Length: 100, // Length of first link in mm
  link2Length: 80,  // Length of second link in mm
  scale: 0.8,       // Scale factor for display
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
function calculateSCARAKinematics(angles: JointAngles): CartesianPosition {
  const { link1Length, link2Length } = SCARA_CONFIG;
  
  // Convert to radians
  const theta1 = (angles.joint1 * Math.PI) / 180;
  const theta2 = (angles.joint2 * Math.PI) / 180;
  
  // Calculate end effector position
  const x = link1Length * Math.cos(theta1) + link2Length * Math.cos(theta1 + theta2);
  const y = link1Length * Math.sin(theta1) + link2Length * Math.sin(theta1 + theta2);
  
  return { x, y };
}

// Calculate SCARA joint positions for rendering
function calculateSCARAJointPositions(angles: JointAngles) {
  const { link1Length, link2Length, scale } = SCARA_CONFIG;
  
  // Convert to radians
  const theta1 = (angles.joint1 * Math.PI) / 180;
  const theta2 = (angles.joint2 * Math.PI) / 180;
  
  // Base at origin
  const base = { x: 0, y: 0 };
  
  // Joint 1 (elbow) position
  const joint1 = {
    x: link1Length * Math.cos(theta1) * scale,
    y: link1Length * Math.sin(theta1) * scale,
  };
  
  // End effector position
  const endEffector = {
    x: (link1Length * Math.cos(theta1) + link2Length * Math.cos(theta1 + theta2)) * scale,
    y: (link1Length * Math.sin(theta1) + link2Length * Math.sin(theta1 + theta2)) * scale,
  };
  
  return { base, joint1, endEffector };
}

export default function RobotVisualization({ lastUpdate }: RobotVisualizationProps) {
  const [selectedGeometry, setSelectedGeometry] = useState<string>(ROBOT_GEOMETRIES[0].id);
  const [jointAngles, setJointAngles] = useState<JointAngles>({ joint1: 0, joint2: 0 });
  const [endEffectorPos, setEndEffectorPos] = useState<CartesianPosition>({ x: 0, y: 0 });
  const [positionHistory, setPositionHistory] = useState<PositionHistoryPoint[]>([]);

  useEffect(() => {
    const deviceManager = connManager.getConnector().getSystemType()?.deviceMgrIF;
    if (!deviceManager) return;

    let mt6701Angle: number | null = null;
    let as5600Angle: number | null = null;

    // Get MT6701 encoder data (joint 1)
    const mt6701State = deviceManager.getDeviceState('I2CA_6_MT6701');
    if (mt6701State?.deviceAttributes?.angle) {
      const values = mt6701State.deviceAttributes.angle.values;
      if (values.length > 0) {
        mt6701Angle = values[values.length - 1];
      }
    }

    // Get AS5600 encoder data (joint 2)
    const as5600State = deviceManager.getDeviceState('I2CA_36_AS5600');
    if (as5600State?.deviceAttributes?.angle) {
      const values = as5600State.deviceAttributes.angle.values;
      if (values.length > 0) {
        as5600Angle = values[values.length - 1];
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
    } else if (selectedGeometry === 'scara') {
      position = calculateSCARAKinematics(angles);
    } else {
      position = { x: 0, y: 0 };
    }
    setEndEffectorPos(position);
    
    // Add to position history
    setPositionHistory((prev) => {
      const now = Date.now();
      const newPoint: PositionHistoryPoint = {
        x: position.x,
        y: position.y,
        timestamp: now,
      };
      
      // Filter out old points and add new one
      const filtered = prev.filter((p) => now - p.timestamp < TRAIL_DURATION_MS);
      
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

  // Convert world coordinates to SVG coordinates
  const worldToSVG = (x: number, y: number) => {
    return {
      x: 200 + x,  // Center at 200,200
      y: 200 - y,  // Flip Y axis (SVG Y increases downward)
    };
  };

  // Render position trail
  const renderTrail = () => {
    if (positionHistory.length < 2) return null;
    
    const now = Date.now();
    const scale = selectedGeometry === 'xy_cartesian' ? XY_CARTESIAN_CONFIG.scale : SCARA_CONFIG.scale;
    
    // Create path segments with varying opacity based on age
    return (
      <g>
        {positionHistory.map((point, index) => {
          if (index === 0) return null; // Skip first point (no line to draw)
          
          const prevPoint = positionHistory[index - 1];
          const age = now - point.timestamp;
          const opacity = Math.max(0, 1 - (age / TRAIL_DURATION_MS));
          
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
        })}
      </g>
    );
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
    
    if (selectedGeometry === 'scara') {
      const positions = calculateSCARAJointPositions(jointAngles);
      
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
          
          {/* Workspace circle (approximate reach) */}
          <circle
            cx={base.x}
            cy={base.y}
            r={(SCARA_CONFIG.link1Length + SCARA_CONFIG.link2Length) * SCARA_CONFIG.scale}
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
        <label htmlFor="geometry-select">Robot Geometry</label>
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
          viewBox="0 0 400 400" 
          className="robot-canvas"
          style={{ 
            border: '1px solid #444',
            borderRadius: '4px',
            background: '#1a1a1a' 
          }}
        >
          {/* Grid lines */}
          <g opacity="0.2">
            {[-150, -100, -50, 0, 50, 100, 150].map((offset) => (
              <React.Fragment key={`grid-${offset}`}>
                <line
                  x1={200 + offset}
                  y1="50"
                  x2={200 + offset}
                  y2="350"
                  stroke="#555"
                  strokeWidth="0.5"
                />
                <line
                  x1="50"
                  y1={200 - offset}
                  x2="350"
                  y2={200 - offset}
                  stroke="#555"
                  strokeWidth="0.5"
                />
              </React.Fragment>
            ))}
          </g>

          {/* Coordinate system */}
          <g>
            {/* X axis */}
            <line x1="50" y1="200" x2="350" y2="200" stroke="#666" strokeWidth="2" />
            <polygon points="350,200 345,197 345,203" fill="#666" />
            <text x="360" y="205" fill="#888" fontSize="14" fontWeight="bold">X</text>
            
            {/* Y axis */}
            <line x1="200" y1="350" x2="200" y2="50" stroke="#666" strokeWidth="2" />
            <polygon points="200,50 197,55 203,55" fill="#666" />
            <text x="205" y="40" fill="#888" fontSize="14" fontWeight="bold">Y</text>
            
            {/* Origin */}
            <circle cx="200" cy="200" r="3" fill="#888" />
            <text x="210" y="215" fill="#888" fontSize="12">(0,0)</text>
          </g>

          {/* Robot */}
          {renderTrail()}
          {renderRobot()}
        </svg>
      </div>

      <div className="visualization-info">
        <div className="info-grid">
          <span className="info-label">Geometry:</span>
          <span className="info-value">
            {ROBOT_GEOMETRIES.find(g => g.id === selectedGeometry)?.name}
          </span>
          
          <span className="info-label">Joint 1 (MT6701):</span>
          <span className="info-value">{jointAngles.joint1.toFixed(1)}°</span>
          
          <span className="info-label">Joint 2 (AS5600):</span>
          <span className="info-value">{jointAngles.joint2.toFixed(1)}°</span>
          
          <span className="info-label">End Effector:</span>
          <span className="info-value">
            X: {endEffectorPos.x.toFixed(1)}mm, Y: {endEffectorPos.y.toFixed(1)}mm
          </span>
        </div>
      </div>
    </div>
  );
}
