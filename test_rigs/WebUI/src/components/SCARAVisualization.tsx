import React from 'react';
import { getRobotGeometry, JointAngles } from '../utils/RobotGeometry';
import { RobotConfig } from '../App';

const robotGeometry = getRobotGeometry();

interface SCARAVisualizationProps {
  jointAngles: JointAngles;
  robotConfig: RobotConfig | null;
  viewBoxSize: number;
  worldToSVG: (x: number, y: number) => { x: number; y: number };
}

/**
 * SCARA Robot Visualization Component
 * Renders a single-arm SCARA robot with two revolute joints
 */
export default function SCARAVisualization({ 
  jointAngles, 
  robotConfig, 
  viewBoxSize,
  worldToSVG 
}: SCARAVisualizationProps) {
  // Joint angles are already gear-corrected by SensorAngleTracker
  const theta1 = robotGeometry.normalizeAngle(jointAngles.joint1);
  const theta2 = robotGeometry.normalizeAngle(jointAngles.joint2 + 180);
  
  const displayAngles: JointAngles = { joint1: theta1, joint2: theta2 };
  const positions = robotGeometry.calculateSCARAJointPositions(displayAngles);
  
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
        r={(robotConfig?.maxRadiusMM || 300) * robotGeometry.getScale()}
        fill="none"
        stroke="#333"
        strokeWidth="1"
        strokeDasharray="5,5"
      />
    </g>
  );
}
