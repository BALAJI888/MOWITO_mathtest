import numpy as np
import math

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.
    
    Args:
        roll: Rotation around x-axis in radians
        pitch: Rotation around y-axis in radians  
        yaw: Rotation around z-axis in radians
    
    Returns:
        numpy array: Quaternion [w, x, y, z]
    """
    # Handle gimbal lock (pitch = ±pi/2)
    if abs(pitch - math.pi/2) < 1e-10:
        # At north pole, roll and yaw become coupled
        roll += yaw
        yaw = 0
    elif abs(pitch + math.pi/2) < 1e-10:
        # At south pole, roll and yaw become coupled  
        roll -= yaw
        yaw = 0
    
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])

def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        q: Quaternion [w, x, y, z]
    
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Normalize quaternion
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def normalize_angles(roll, pitch, yaw):
    """
    Normalize Euler angles to [-pi, pi] range.
    """
    roll = (roll + math.pi) % (2 * math.pi) - math.pi
    pitch = (pitch + math.pi) % (2 * math.pi) - math.pi  
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return roll, pitch, yaw

# Example usage and testing
if __name__ == "__main__":
    # Test case 1: Regular angles
    print("Test 1: Regular angles")
    roll, pitch, yaw = math.radians(30), math.radians(45), math.radians(60)
    print(f"Input Euler: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")
    
    q = euler_to_quaternion(roll, pitch, yaw)
    print(f"Quaternion: w={q[0]:.4f}, x={q[1]:.4f}, y={q[2]:.4f}, z={q[3]:.4f}")
    
    roll_back, pitch_back, yaw_back = quaternion_to_euler(q)
    roll_back, pitch_back, yaw_back = normalize_angles(roll_back, pitch_back, yaw_back)
    print(f"Converted back: roll={math.degrees(roll_back):.1f}°, pitch={math.degrees(pitch_back):.1f}°, yaw={math.degrees(yaw_back):.1f}°")
    
    # Test case 2: Gimbal lock (pitch = 90°)
    print("\nTest 2: Gimbal lock case")
    roll, pitch, yaw = math.radians(30), math.radians(90), math.radians(60)
    print(f"Input Euler: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")
    
    q = euler_to_quaternion(roll, pitch, yaw)
    print(f"Quaternion: w={q[0]:.4f}, x={q[1]:.4f}, y={q[2]:.4f}, z={q[3]:.4f}")
    
    roll_back, pitch_back, yaw_back = quaternion_to_euler(q)
    roll_back, pitch_back, yaw_back = normalize_angles(roll_back, pitch_back, yaw_back)
    print(f"Converted back: roll={math.degrees(roll_back):.1f}°, pitch={math.degrees(pitch_back):.1f}°, yaw={math.degrees(yaw_back):.1f}°")
    
    # Test case 3: Identity rotation
    print("\nTest 3: Identity rotation")
    roll, pitch, yaw = 0, 0, 0
    q = euler_to_quaternion(roll, pitch, yaw)
    print(f"Quaternion for zero rotation: w={q[0]:.4f}, x={q[1]:.4f}, y={q[2]:.4f}, z={q[3]:.4f}")