/**
 * SensorAngleTracker - Manages multi-turn angle accumulation for magnetic rotation sensors
 * 
 * This service handles:
 * - Tracking rotations across 360° boundaries for magnetic encoders
 * - Applying gear factors correctly AFTER accumulation
 * - Providing reset capability for calibration/homing
 * - Persisting state across component re-renders
 * 
 * The tracker maintains accumulated angles per sensor device ID, detecting 360° wraparounds
 * and accounting for gearing ratios. This separates the concern of angle tracking from
 * visualization components.
 */

interface SensorTrackerState {
  accumulated: number;      // Accumulated rotations in degrees (motor shaft)
  prevRaw: number | null;    // Previous raw angle reading
  gearFactor: number;        // Gear ratio (motor rotations per joint rotation)
}

export class SensorAngleTracker {
  private static instance: SensorAngleTracker | null = null;
  private trackers: Map<string, SensorTrackerState> = new Map();
  
  // Thresholds for wraparound detection
  private static readonly WRAPAROUND_HIGH_THRESHOLD = 270;
  private static readonly WRAPAROUND_LOW_THRESHOLD = 90;

  private constructor() {
    // Private constructor for singleton
  }

  /**
   * Get the singleton instance of the tracker
   */
  public static getInstance(): SensorAngleTracker {
    if (!SensorAngleTracker.instance) {
      SensorAngleTracker.instance = new SensorAngleTracker();
    }
    return SensorAngleTracker.instance;
  }

  /**
   * Configure a sensor tracker with its gear factor
   * @param deviceId Unique identifier for the sensor (e.g., '1_6' for MT6701)
   * @param gearFactor Gear ratio (default 1.0)
   */
  public configure(deviceId: string, gearFactor: number = 1.0): void {
    if (!this.trackers.has(deviceId)) {
      this.trackers.set(deviceId, {
        accumulated: 0,
        prevRaw: null,
        gearFactor: gearFactor
      });
    } else {
      // Update gear factor for existing tracker
      const tracker = this.trackers.get(deviceId)!;
      tracker.gearFactor = gearFactor;
    }
  }

  /**
   * Update with a new raw angle reading and return the accumulated angle
   * @param deviceId Unique identifier for the sensor
   * @param rawAngle Raw angle reading from sensor (0-360 degrees)
   * @returns Accumulated angle accounting for multi-turn and gear factor (joint angle in degrees)
   */
  public update(deviceId: string, rawAngle: number): number {
    // Ensure tracker exists
    if (!this.trackers.has(deviceId)) {
      this.configure(deviceId, 1.0);
    }

    const tracker = this.trackers.get(deviceId)!;

    // Detect wraparound and update accumulated angle
    if (tracker.prevRaw !== null) {
      // Transition from >270° to <90° means we crossed 0° going forward
      if (tracker.prevRaw > SensorAngleTracker.WRAPAROUND_HIGH_THRESHOLD && 
          rawAngle < SensorAngleTracker.WRAPAROUND_LOW_THRESHOLD) {
        tracker.accumulated += 360;
      }
      // Transition from <90° to >270° means we crossed 0° going backward
      else if (tracker.prevRaw < SensorAngleTracker.WRAPAROUND_LOW_THRESHOLD && 
               rawAngle > SensorAngleTracker.WRAPAROUND_HIGH_THRESHOLD) {
        tracker.accumulated -= 360;
      }
    }

    tracker.prevRaw = rawAngle;

    // Return absolute motor shaft angle divided by gear factor to get joint angle
    // Example: If motor rotated 1080° (3 turns) and gearFactor=3, joint angle = 360°
    return (tracker.accumulated + rawAngle) / tracker.gearFactor;
  }

  /**
   * Get the current accumulated angle for a sensor (without updating)
   * @param deviceId Unique identifier for the sensor
   * @returns Current accumulated angle in degrees (joint angle), or 0 if not tracked
   */
  public getAccumulatedAngle(deviceId: string): number {
    const tracker = this.trackers.get(deviceId);
    if (!tracker || tracker.prevRaw === null) {
      return 0;
    }
    return (tracker.accumulated + tracker.prevRaw) / tracker.gearFactor;
  }

  /**
   * Get the raw accumulated motor shaft angle (before gear factor division)
   * @param deviceId Unique identifier for the sensor
   * @returns Motor shaft angle in degrees, or 0 if not tracked
   */
  public getRawAccumulatedAngle(deviceId: string): number {
    const tracker = this.trackers.get(deviceId);
    if (!tracker || tracker.prevRaw === null) {
      return 0;
    }
    return tracker.accumulated + tracker.prevRaw;
  }

  /**
   * Get the rotation count (number of complete 360° rotations)
   * @param deviceId Unique identifier for the sensor
   * @returns Number of full rotations (can be negative)
   */
  public getRotationCount(deviceId: string): number {
    const tracker = this.trackers.get(deviceId);
    if (!tracker) {
      return 0;
    }
    return Math.floor(tracker.accumulated / 360);
  }

  /**
   * Reset angle tracking for one or all sensors
   * @param deviceId Optional device ID to reset; if omitted, resets all trackers
   */
  public reset(deviceId?: string): void {
    if (deviceId) {
      // Reset specific sensor
      const tracker = this.trackers.get(deviceId);
      if (tracker) {
        tracker.accumulated = 0;
        tracker.prevRaw = null;
      }
    } else {
      // Reset all sensors
      this.trackers.forEach((tracker) => {
        tracker.accumulated = 0;
        tracker.prevRaw = null;
      });
    }
  }

  /**
   * Check if a sensor is being tracked
   * @param deviceId Unique identifier for the sensor
   * @returns True if sensor has been configured and has received at least one reading
   */
  public isTracking(deviceId: string): boolean {
    const tracker = this.trackers.get(deviceId);
    return tracker !== undefined && tracker.prevRaw !== null;
  }

  /**
   * Get all tracked device IDs
   * @returns Array of device IDs currently being tracked
   */
  public getTrackedDevices(): string[] {
    return Array.from(this.trackers.keys());
  }

  /**
   * Get debug information for a sensor
   * @param deviceId Unique identifier for the sensor
   * @returns Debug info object or null if not tracked
   */
  public getDebugInfo(deviceId: string): {
    accumulated: number;
    prevRaw: number | null;
    gearFactor: number;
    rotations: number;
    jointAngle: number;
  } | null {
    const tracker = this.trackers.get(deviceId);
    if (!tracker) {
      return null;
    }
    return {
      accumulated: tracker.accumulated,
      prevRaw: tracker.prevRaw,
      gearFactor: tracker.gearFactor,
      rotations: Math.floor(tracker.accumulated / 360),
      jointAngle: tracker.prevRaw !== null 
        ? (tracker.accumulated + tracker.prevRaw) / tracker.gearFactor 
        : 0
    };
  }
}

// Export singleton instance getter
export const getSensorAngleTracker = () => SensorAngleTracker.getInstance();
