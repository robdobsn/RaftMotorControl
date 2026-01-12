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

// Robot configuration for XY Cartesian (2-joint arm)
const ROBOT_CONFIG = {
  link1Length: 100, // Length of first link in mm
  link2Length: 80,  // Length of second link in mm
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

// Forward kinematics for 2-joint planar arm
function calculateForwardKinematics(angles: JointAngles): CartesianPosition {
  const { link1Length, link2Length } = ROBOT_CONFIG;
  
  // Convert to radians
  const theta1 = (angles.joint1 * Math.PI) / 180;
  const theta2 = (angles.joint2 * Math.PI) / 180;
  
  // Calculate end effector position
  const x = link1Length * Math.cos(theta1) + link2Length * Math.cos(theta1 + theta2);
  const y = link1Length * Math.sin(theta1) + link2Length * Math.sin(theta1 + theta2);
  
  return { x, y };
}

// Calculate joint positions for rendering
function calculateJointPositions(angles: JointAngles) {
  const { link1Length, link2Length, scale } = ROBOT_CONFIG;
  
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

    // Calculate forward kinematics
    const position = calculateForwardKinematics(angles);
    setEndEffectorPos(position);
  }, [lastUpdate]);

  // Convert world coordinates to SVG coordinates
  const worldToSVG = (x: number, y: number) => {
    return {
      x: 200 + x,  // Center at 200,200
      y: 200 - y,  // Flip Y axis (SVG Y increases downward)
    };
  };

  // Render robot based on selected geometry
  const renderRobot = () => {
    if (selectedGeometry === 'xy_cartesian') {
      const positions = calculateJointPositions(jointAngles);
      
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
            r={(ROBOT_CONFIG.link1Length + ROBOT_CONFIG.link2Length) * ROBOT_CONFIG.scale}
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
