import React from 'react';
import { getRobotGeometry, JointAngles } from '../utils/RobotGeometry';

const robotGeometry = getRobotGeometry();

interface CartesianVisualizationProps {
  jointAngles: JointAngles;
  viewBoxSize: number;
  worldToSVG: (x: number, y: number) => { x: number; y: number };
}

/**
 * Cartesian (XY) Robot Visualization Component
 * Renders a 2-axis cartesian robot with linear X and Y motion
 */
export default function CartesianVisualization({ 
  jointAngles, 
  viewBoxSize,
  worldToSVG 
}: CartesianVisualizationProps) {
  // Get cartesian configuration
  const cartesianConfig = robotGeometry.getCartesianConfig();
  const xRange = cartesianConfig.xRange;
  const yRange = cartesianConfig.yRange;
  const scale = cartesianConfig.scale;
  
  // Calculate XY position from joint angles
  const position = {
    x: (jointAngles.joint1 / 360) * xRange,
    y: (jointAngles.joint2 / 360) * yRange
  };
  
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
