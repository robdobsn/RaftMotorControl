#!/usr/bin/env python3
"""
SCARA Kinematics Calculator
Computes forward and inverse kinematics for single-arm SCARA robot
where both theta1 and theta2 are measured from positive X-axis
"""

import math

class SCARAKinematics:
    def __init__(self, L1, L2):
        """
        Initialize SCARA robot with arm lengths
        L1: Length of first arm (mm)
        L2: Length of second arm (mm)
        """
        self.L1 = L1
        self.L2 = L2
        self.max_radius = L1 + L2
        self.min_radius = abs(L1 - L2)
    
    def forward_kinematics(self, theta1_deg, theta2_deg):
        """
        Compute end effector position from joint angles
        theta1_deg: Angle of joint 1 from +X axis (degrees)
        theta2_deg: Angle of joint 2 from +X axis (degrees)
        Returns: (x, y) position in mm
        """
        theta1_rad = math.radians(theta1_deg)
        theta2_rad = math.radians(theta2_deg)
        
        x = self.L1 * math.cos(theta1_rad) + self.L2 * math.cos(theta2_rad)
        y = self.L1 * math.sin(theta1_rad) + self.L2 * math.sin(theta2_rad)
        
        return (x, y)
    
    def inverse_kinematics(self, x, y):
        """
        Compute joint angles from end effector position
        x, y: Target position (mm)
        Returns: Two solutions (theta1, theta2) in degrees, or None if unreachable
        """
        # Distance from origin to target
        L3 = math.sqrt(x*x + y*y)
        
        # Check if position is reachable
        if L3 > self.max_radius or L3 < self.min_radius:
            return None
        
        # Angle to target point
        target_angle = math.atan2(y, x)
        
        # Use law of cosines to find angles
        # cos(angle_at_target) = (L1^2 + L2^2 - L3^2) / (2*L1*L2)
        cos_angle_at_elbow = (self.L1**2 + self.L2**2 - L3**2) / (2 * self.L1 * self.L2)
        
        # Clamp to valid range to handle floating point errors
        cos_angle_at_elbow = max(-1.0, min(1.0, cos_angle_at_elbow))
        
        # Two elbow configurations
        elbow_angle_pos = math.acos(cos_angle_at_elbow)
        elbow_angle_neg = -elbow_angle_pos
        
        # For each elbow configuration, compute joint angles
        solutions = []
        
        for elbow_angle in [elbow_angle_pos, elbow_angle_neg]:
            # cos(angle_at_base) = (L1^2 + L3^2 - L2^2) / (2*L1*L3)
            cos_angle_at_base = (self.L1**2 + L3**2 - self.L2**2) / (2 * self.L1 * L3)
            cos_angle_at_base = max(-1.0, min(1.0, cos_angle_at_base))
            
            if elbow_angle > 0:
                # Elbow up configuration
                angle_at_base = math.acos(cos_angle_at_base)
                theta1 = target_angle - angle_at_base
            else:
                # Elbow down configuration
                angle_at_base = math.acos(cos_angle_at_base)
                theta1 = target_angle + angle_at_base
            
            # theta2 = theta1 + elbow_angle (in robot frame)
            # But we need theta2 as absolute angle from X-axis
            # Vector to elbow: L1 at angle theta1
            # Vector from elbow to end effector: L2 at angle theta2
            # theta2 = theta1 + pi - elbow_angle (for standard SCARA)
            theta2 = theta1 + math.pi - elbow_angle
            
            # Convert to degrees and normalize to [0, 360)
            theta1_deg = math.degrees(theta1) % 360
            theta2_deg = math.degrees(theta2) % 360
            
            solutions.append((theta1_deg, theta2_deg))
        
        return solutions
    
    def normalize_angle(self, angle_deg):
        """Normalize angle to [0, 360) range"""
        return angle_deg % 360
    
    def wrap_degrees(self, angle_deg):
        """Wrap angle to [0, 360) range like firmware does"""
        while angle_deg < 0:
            angle_deg += 360
        while angle_deg >= 360:
            angle_deg -= 360
        return angle_deg


def compare_with_firmware():
    """Test with positions from firmware log and compare"""
    
    print("=" * 80)
    print("SCARA Kinematics Test - Comparing with Firmware")
    print("=" * 80)
    print(f"Arm lengths: L1 = 150mm, L2 = 150mm")
    print(f"Max radius: {150 + 150}mm")
    print()
    
    robot = SCARAKinematics(150, 150)
    
    # Test positions from the WebUI log
    test_positions = [
        (261.00, 0.00),
        (258.94, 32.71),
        (252.80, 64.91),
        (242.67, 96.08),
        (228.72, 125.74),
        (211.15, 153.41),
        (190.26, 178.67),
        (166.37, 201.10),
        (139.85, 220.37),
        (111.13, 236.16),
        (80.65, 248.23),
        (48.91, 256.38),
        (16.39, 260.48),
    ]
    
    # Firmware results for first position (from log)
    firmware_results = {
        (261.00, 0.00): (330.46, 29.54),  # Alternative: (29.54, 330.46)
        (258.94, 32.71): (337.66, 36.74),
        (252.80, 64.91): (344.86, 43.94),
        (242.67, 96.08): (352.06, 51.14),
        (228.72, 125.74): (359.26, 58.34),
        (211.15, 153.41): (6.46, 65.54),
        (190.26, 178.67): (13.66, 72.74),
        (166.37, 201.10): (20.86, 79.94),
        (139.85, 220.37): (28.06, 87.14),
        (111.13, 236.16): (35.26, 94.34),
        (80.65, 248.23): (42.46, 101.54),
        (48.91, 256.38): (49.66, 108.74),
        (16.39, 260.48): (56.86, 115.94),
    }
    
    print("-" * 80)
    print(f"{'Position (x, y)':<20} {'Firmware Angles':<25} {'Computed Angles':<25} {'Error':<10}")
    print("-" * 80)
    
    for x, y in test_positions:
        # Compute inverse kinematics
        solutions = robot.inverse_kinematics(x, y)
        
        if solutions is None:
            print(f"({x:7.2f}, {y:7.2f})  UNREACHABLE")
            continue
        
        # Get firmware result
        firmware = firmware_results.get((x, y), (None, None))
        
        # Find which solution matches firmware
        sol1, sol2 = solutions
        
        # Check which solution is closer to firmware
        if firmware[0] is not None:
            error1 = min(abs(sol1[0] - firmware[0]), 360 - abs(sol1[0] - firmware[0]))
            error2 = min(abs(sol2[0] - firmware[0]), 360 - abs(sol2[0] - firmware[0]))
            
            if error1 < error2:
                chosen_sol = sol1
                error_t1 = min(abs(sol1[0] - firmware[0]), 360 - abs(sol1[0] - firmware[0]))
                error_t2 = min(abs(sol1[1] - firmware[1]), 360 - abs(sol1[1] - firmware[1]))
            else:
                chosen_sol = sol2
                error_t1 = min(abs(sol2[0] - firmware[0]), 360 - abs(sol2[0] - firmware[0]))
                error_t2 = min(abs(sol2[1] - firmware[1]), 360 - abs(sol2[1] - firmware[1]))
            
            print(f"({x:7.2f}, {y:7.2f})  θ1={firmware[0]:6.2f}° θ2={firmware[1]:6.2f}°  "
                  f"θ1={chosen_sol[0]:6.2f}° θ2={chosen_sol[1]:6.2f}°  "
                  f"Δ={error_t1:.2f}°,{error_t2:.2f}°")
        else:
            print(f"({x:7.2f}, {y:7.2f})  No firmware data          "
                  f"θ1={sol1[0]:6.2f}° θ2={sol1[1]:6.2f}°")
        
        # Verify with forward kinematics
        if firmware[0] is not None:
            x_check, y_check = robot.forward_kinematics(firmware[0], firmware[1])
            print(f"  Firmware FK check: ({x_check:7.2f}, {y_check:7.2f}) - "
                  f"Error: Δx={abs(x-x_check):.3f}mm, Δy={abs(y-y_check):.3f}mm")
    
    print("-" * 80)
    print()
    
    # Test forward kinematics with firmware angles
    print("=" * 80)
    print("Forward Kinematics Test with Firmware Angles")
    print("=" * 80)
    print(f"{'Firmware Angles':<25} {'Computed Position':<25} {'Expected Position':<25}")
    print("-" * 80)
    
    for (x_exp, y_exp), (t1, t2) in firmware_results.items():
        x_calc, y_calc = robot.forward_kinematics(t1, t2)
        error_x = abs(x_calc - x_exp)
        error_y = abs(y_calc - y_exp)
        print(f"θ1={t1:6.2f}° θ2={t2:6.2f}°  "
              f"({x_calc:7.2f}, {y_calc:7.2f})  "
              f"({x_exp:7.2f}, {y_exp:7.2f})  "
              f"Δ=({error_x:.3f}, {error_y:.3f})")
    
    print("-" * 80)


if __name__ == "__main__":
    compare_with_firmware()
