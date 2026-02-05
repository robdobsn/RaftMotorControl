/**
 * RobotGeometry - Centralized geometry and kinematics calculations for robot control
 * 
 * This class handles:
 * - Forward kinematics for different robot geometries (SCARA, Cartesian, etc.)
 * - Workspace calculations
 * - Coordinate transformations
 * - Configuration from robot settings
 */

import { RobotConfig } from '../App';

export interface JointAngles {
  joint1: number; // degrees
  joint2: number; // degrees
}

export interface CartesianPosition {
  x: number; // mm
  y: number; // mm
}

export interface SCARAJointPositions {
  base: CartesianPosition;
  joint1: CartesianPosition;
  endEffector: CartesianPosition;
}

export interface WorkspaceDimensions {
  width: number;
  height: number;
  description: string;
}

export type GeometryType = 'singlearmscara' | 'xy_cartesian' | 'polar' | 'delta' | 'custom';

interface SCARAConfig {
  link1Length: number;
  link2Length: number;
  scale: number;
  invertJoint1: boolean;
  invertJoint2: boolean;
  invertAngleDirection: boolean;
  invertJoint1Sensor: boolean;
  invertJoint2Sensor: boolean;
}

interface CartesianConfig {
  xRange: number;
  yRange: number;
  scale: number;
}

// Default configurations
const DEFAULT_SCARA_CONFIG: SCARAConfig = {
  link1Length: 150,
  link2Length: 150,
  scale: 0.8,
  invertJoint1: false,
  invertJoint2: false,
  invertAngleDirection: false,
  invertJoint1Sensor: false,
  invertJoint2Sensor: false,
};

const DEFAULT_CARTESIAN_CONFIG: CartesianConfig = {
  xRange: 200,
  yRange: 200,
  scale: 0.8,
};

export class RobotGeometry {
  private geometryType: GeometryType = 'singlearmscara';
  private scaraConfig: SCARAConfig = { ...DEFAULT_SCARA_CONFIG };
  private cartesianConfig: CartesianConfig = { ...DEFAULT_CARTESIAN_CONFIG };
  private robotConfig: RobotConfig | null = null;

  /**
   * Configure geometry from robot settings
   */
  public configure(robotConfig: RobotConfig): void {
    this.robotConfig = robotConfig;
    
    // Determine geometry type from config
    const geomLower = robotConfig.geometry.toLowerCase();
    if (geomLower.includes('singlearmscara') || geomLower.includes('scara')) {
      this.geometryType = 'singlearmscara';
      this.scaraConfig = {
        link1Length: robotConfig.arm1LengthMM,
        link2Length: robotConfig.arm2LengthMM,
        scale: 0.8,
        invertJoint1: false,
        invertJoint2: false,
        invertAngleDirection: false,
        invertJoint1Sensor: this.scaraConfig.invertJoint1Sensor,
        invertJoint2Sensor: this.scaraConfig.invertJoint2Sensor,
      };
    } else if (geomLower.includes('cartesian') || geomLower.includes('xyz')) {
      this.geometryType = 'xy_cartesian';
      if (robotConfig.axes && robotConfig.axes.length >= 2) {
        this.cartesianConfig = {
          xRange: robotConfig.axes[0]?.unitsPerRot || DEFAULT_CARTESIAN_CONFIG.xRange,
          yRange: robotConfig.axes[1]?.unitsPerRot || DEFAULT_CARTESIAN_CONFIG.yRange,
          scale: 0.8,
        };
      }
    }
  }

  /**
   * Get current geometry type
   */
  public getGeometryType(): GeometryType {
    return this.geometryType;
  }

  /**
   * Set geometry type manually
   */
  public setGeometryType(type: GeometryType): void {
    this.geometryType = type;
  }

  /**
   * Get current SCARA configuration
   */
  public getSCARAConfig(): SCARAConfig {
    return { ...this.scaraConfig };
  }

  /**
   * Get current Cartesian configuration
   */
  public getCartesianConfig(): CartesianConfig {
    return { ...this.cartesianConfig };
  }

  /**
   * Get active configuration for currently selected geometry
   */
  public getActiveConfig(): SCARAConfig | CartesianConfig {
    if (this.geometryType === 'singlearmscara') {
      return this.getSCARAConfig();
    } else if (this.geometryType === 'xy_cartesian') {
      return this.getCartesianConfig();
    }
    return this.getSCARAConfig();
  }

  /**
   * Calculate forward kinematics - convert joint angles to Cartesian position
   */
  public forwardKinematics(angles: JointAngles): CartesianPosition {
    if (this.geometryType === 'singlearmscara') {
      return this.calculateSCARAKinematics(angles);
    } else if (this.geometryType === 'xy_cartesian') {
      return this.calculateXYCartesian(angles);
    }
    return { x: 0, y: 0 };
  }

  /**
   * Calculate SCARA forward kinematics
   */
  private calculateSCARAKinematics(angles: JointAngles): CartesianPosition {
    const { link1Length, link2Length, invertJoint1, invertJoint2 } = this.scaraConfig;
    
    // Convert to radians and apply inversion
    const theta1 = ((invertJoint1 ? -angles.joint1 : angles.joint1) * Math.PI) / 180;
    const theta2 = ((invertJoint2 ? -angles.joint2 : angles.joint2) * Math.PI) / 180;
    
    // Both theta1 and theta2 are measured from the positive X axis
    const x = link1Length * Math.cos(theta1) + link2Length * Math.cos(theta2);
    const y = link1Length * Math.sin(theta1) + link2Length * Math.sin(theta2);
    
    return { x, y };
  }

  /**
   * Calculate XY Cartesian position (direct angle to position mapping)
   */
  private calculateXYCartesian(angles: JointAngles): CartesianPosition {
    const { xRange, yRange } = this.cartesianConfig;
    
    // Map angle (0-360°) to linear position
    // Center at 0, so range goes from -xRange/2 to +xRange/2
    const x = ((angles.joint1 / 360) * xRange) - (xRange / 2);
    const y = ((angles.joint2 / 360) * yRange) - (yRange / 2);
    
    return { x, y };
  }

  /**
   * Calculate SCARA joint positions for rendering (includes scale)
   */
  public calculateSCARAJointPositions(angles: JointAngles): SCARAJointPositions {
    const { link1Length, link2Length, scale, invertJoint1, invertJoint2 } = this.scaraConfig;
    
    // Convert to radians and apply inversion
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
    const endEffector = {
      x: (link1Length * Math.cos(theta1) + link2Length * Math.cos(theta2)) * scale,
      y: (link1Length * Math.sin(theta1) + link2Length * Math.sin(theta2)) * scale,
    };
    
    return { base, joint1, endEffector };
  }

  /**
   * Get workspace dimensions based on current geometry and configuration
   */
  public getWorkspaceDimensions(): WorkspaceDimensions {
    if (!this.robotConfig) {
      return { width: 200, height: 200, description: 'Default 200mm x 200mm' };
    }

    const geomLower = this.robotConfig.geometry.toLowerCase();
    
    if (geomLower.includes('cartesian') || geomLower.includes('xyz')) {
      // For XY Cartesian, use axes configuration
      if (this.robotConfig.axes && this.robotConfig.axes.length >= 2) {
        const xRange = this.robotConfig.axes[0]?.unitsPerRot || 360;
        const yRange = this.robotConfig.axes[1]?.unitsPerRot || 360;
        return { 
          width: xRange * 0.9, 
          height: yRange * 0.9,
          description: `${(xRange * 0.9).toFixed(0)}mm x ${(yRange * 0.9).toFixed(0)}mm (90% of axis range)` 
        };
      }
      return { width: 180, height: 180, description: '180mm x 180mm (default)' };
    } else {
      // For SCARA, use maxRadiusMM
      const arm1Len = this.robotConfig.arm1LengthMM || 150;
      const arm2Len = this.robotConfig.arm2LengthMM || 150;
      const maxRadius = this.robotConfig.maxRadiusMM || Math.min(290, arm1Len + arm2Len);
      const workspaceSize = maxRadius * 0.9;
      return { 
        width: workspaceSize * 2, 
        height: workspaceSize * 2,
        description: `Radius ${workspaceSize.toFixed(0)}mm (90% of max ${maxRadius.toFixed(0)}mm)` 
      };
    }
  }

  /**
   * Get usable workspace dimensions for patterns
   * For circular workspaces (SCARA), returns dimensions that ensure rectangles fit within the circle
   */
  public getPatternWorkspaceDimensions(): WorkspaceDimensions {
    if (!this.robotConfig) {
      return { width: 200, height: 200, description: 'Default 200mm x 200mm' };
    }

    const geomLower = this.robotConfig.geometry.toLowerCase();
    
    if (geomLower.includes('cartesian') || geomLower.includes('xyz')) {
      // For Cartesian, same as regular workspace
      return this.getWorkspaceDimensions();
    } else {
      // For SCARA circular workspace, ensure rectangles fit within the circle
      // Maximum square inscribed in a circle: side = radius * sqrt(2)
      // Scaling works as: (normalized - 0.5) * width maps [0,1] to [-width/2, +width/2]
      const arm1Len = this.robotConfig.arm1LengthMM || 150;
      const arm2Len = this.robotConfig.arm2LengthMM || 150;
      const maxRadius = this.robotConfig.maxRadiusMM || Math.min(290, arm1Len + arm2Len);
      const workspaceRadius = maxRadius * 0.9; // 10% margin from max radius
      // For a square to fit in a circle: side = radius * sqrt(2)
      const maxRectSide = workspaceRadius * Math.sqrt(2);
      return { 
        width: maxRectSide, 
        height: maxRectSide,
        description: `Max rect ${maxRectSide.toFixed(0)}mm x ${maxRectSide.toFixed(0)}mm in radius ${workspaceRadius.toFixed(0)}mm` 
      };
    }
  }

  /**
   * Get workspace dimensions for proportionate mode
   * This returns the full axis range that the firmware uses for proportionate coordinate conversion
   * For SCARA: maps to full -maxRadius to +maxRadius range (diameter = 2*maxRadius)
   * For Cartesian: maps to the configured axis ranges
   */
  public getProportionateWorkspaceDimensions(): WorkspaceDimensions {
    if (!this.robotConfig) {
      return { width: 400, height: 400, description: 'Default 400mm x 400mm' };
    }

    const geomLower = this.robotConfig.geometry.toLowerCase();
    
    if (geomLower.includes('cartesian') || geomLower.includes('xyz')) {
      // For Cartesian, use axis min/max ranges
      return this.getWorkspaceDimensions();
    } else {
      // For SCARA: firmware maps proportionate 0-1 to -maxRadius to +maxRadius
      const arm1Len = this.robotConfig.arm1LengthMM || 150;
      const arm2Len = this.robotConfig.arm2LengthMM || 150;
      // Physical max reach is arm1 + arm2
      const physicalMaxRadius = arm1Len + arm2Len;
      // Use the smaller of: configured maxRadius (if valid) or physical max
      // This handles cases where firmware returns a bogus default (e.g., 290) larger than physical reach
      const configuredMax = this.robotConfig.maxRadiusMM || physicalMaxRadius;
      const maxRadius = Math.min(configuredMax, physicalMaxRadius);
      // Full diameter is 2*maxRadius
      const diameter = maxRadius * 2;
      console.log(`[RobotGeometry] getProportionateWorkspaceDimensions: arm1=${arm1Len}, arm2=${arm2Len}, physicalMax=${physicalMaxRadius}, configuredMax=${configuredMax}, maxRadius=${maxRadius}, diameter=${diameter}`);
      return { 
        width: diameter, 
        height: diameter,
        description: `Full range ±${maxRadius.toFixed(0)}mm (diameter ${diameter.toFixed(0)}mm)` 
      };
    }
  }

  /**
   * Normalize angle to 0-360 range, handling negatives
   */
  public normalizeAngle(angle: number): number {
    return ((angle % 360) + 360) % 360;
  }

  /**
   * Get gear factor for a specific axis
   */
  public getGearFactor(axisIndex: number): number {
    if (!this.robotConfig || !this.robotConfig.axes || axisIndex >= this.robotConfig.axes.length) {
      return 1;
    }
    return this.robotConfig.axes[axisIndex]?.gearFactor || 1;
  }

  /**
   * Check if angle direction should be inverted
   */
  public shouldInvertAngleDirection(): boolean {
    if (this.geometryType === 'singlearmscara') {
      return this.scaraConfig.invertAngleDirection;
    }
    return false;
  }

  /**
   * Check if joint 1 sensor should be inverted
   */
  public shouldInvertJoint1Sensor(): boolean {
    if (this.geometryType === 'singlearmscara') {
      return this.scaraConfig.invertJoint1Sensor;
    }
    return false;
  }

  /**
   * Check if joint 2 sensor should be inverted
   */
  public shouldInvertJoint2Sensor(): boolean {
    if (this.geometryType === 'singlearmscara') {
      return this.scaraConfig.invertJoint2Sensor;
    }
    return false;
  }

  /**
   * Set joint 1 sensor inversion
   */
  public setInvertJoint1Sensor(invert: boolean): void {
    this.scaraConfig.invertJoint1Sensor = invert;
  }

  /**
   * Set joint 2 sensor inversion
   */
  public setInvertJoint2Sensor(invert: boolean): void {
    this.scaraConfig.invertJoint2Sensor = invert;
  }

  /**
   * Get scale factor for rendering
   */
  public getScale(): number {
    if (this.geometryType === 'singlearmscara') {
      return this.scaraConfig.scale;
    } else if (this.geometryType === 'xy_cartesian') {
      return this.cartesianConfig.scale;
    }
    return 1.0;
  }

  /**
   * Get robot configuration
   */
  public getRobotConfig(): RobotConfig | null {
    return this.robotConfig;
  }
}

// Singleton instance
let geometryInstance: RobotGeometry | null = null;

/**
 * Get the singleton RobotGeometry instance
 */
export function getRobotGeometry(): RobotGeometry {
  if (!geometryInstance) {
    geometryInstance = new RobotGeometry();
  }
  return geometryInstance;
}
