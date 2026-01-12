import React, { useState } from 'react';

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

export default function RobotVisualization({ lastUpdate }: RobotVisualizationProps) {
  const [selectedGeometry, setSelectedGeometry] = useState<string>(ROBOT_GEOMETRIES[0].id);

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
          {/* Coordinate system */}
          <g>
            {/* X axis */}
            <line x1="200" y1="200" x2="350" y2="200" stroke="#444" strokeWidth="1" />
            <text x="360" y="205" fill="#888" fontSize="12">X</text>
            
            {/* Y axis */}
            <line x1="200" y1="200" x2="200" y2="50" stroke="#444" strokeWidth="1" />
            <text x="205" y="40" fill="#888" fontSize="12">Y</text>
            
            {/* Origin */}
            <circle cx="200" cy="200" r="3" fill="#4a9eff" />
          </g>

          {/* Placeholder content - geometry will be rendered here */}
          <text 
            x="200" 
            y="350" 
            textAnchor="middle" 
            fill="#666" 
            fontSize="14"
          >
            2D End-Effector Visualization
          </text>
          <text 
            x="200" 
            y="370" 
            textAnchor="middle" 
            fill="#666" 
            fontSize="12"
          >
            (To be implemented)
          </text>
        </svg>
      </div>

      <div className="visualization-info">
        <div className="info-grid">
          <span className="info-label">Geometry:</span>
          <span className="info-value">
            {ROBOT_GEOMETRIES.find(g => g.id === selectedGeometry)?.name}
          </span>
          <span className="info-label">End Effector:</span>
          <span className="info-value">--</span>
          <span className="info-label">Position:</span>
          <span className="info-value">(X: --, Y: --)</span>
        </div>
      </div>
    </div>
  );
}
