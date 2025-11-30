import numpy as np
import math

def dh_matrix(alpha, a, d, theta):
    """
    Create a Denavit-Hartenberg transformation matrix.
    
    Args:
        alpha: link twist (radians)
        a: link length
        d: link offset
        theta: joint angle (radians)
    
    Returns:
        4x4 transformation matrix
    """
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)
    
    return np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(j1, j2, j3, j4, link_lengths=[1.0, 1.0, 1.0, 1.0]):
    """
    Calculate forward kinematics for a 4-DOF robot arm with perpendicular joints.
    
    Args:
        j1, j2, j3, j4: joint angles in radians
        link_lengths: list of four link lengths [L1, L2, L3, L4]
    
    Returns:
        tuple: (x, y, z) coordinates of end-effector
    """
    L1, L2, L3, L4 = link_lengths
    
    # DH parameters for 4-DOF robot with perpendicular joints
    # Format: [alpha, a, d, theta]
    dh_params = [
        [math.pi/2, 0, 0, j1],           # Joint 1: rotation about z, next joint perpendicular
        [0, L1, 0, j2],                  # Joint 2: rotation about new x
        [math.pi/2, 0, 0, j3],           # Joint 3: rotation about new z, next joint perpendicular  
        [0, L2, 0, j4],                  # Joint 4: rotation about new x
    ]
    
    # Start with identity matrix
    T = np.identity(4)
    
    # Multiply all transformation matrices
    for params in dh_params:
        alpha, a, d, theta = params
        T_i = dh_matrix(alpha, a, d, theta)
        T = np.dot(T, T_i)
    
    # Apply the remaining links (L3 and L4 as tool length)
    # After joint 4, we have two more links in the same direction
    tool_transform = np.array([
        [1, 0, 0, L3 + L4],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    T_final = np.dot(T, tool_transform)
    
    # Extract position from transformation matrix
    x = T_final[0, 3]
    y = T_final[1, 3]
    z = T_final[2, 3]
    
    return x, y, z

def forward_kinematics_alternative(j1, j2, j3, j4, link_lengths=[1.0, 1.0, 1.0, 1.0]):
    """
    Alternative implementation using homogeneous transformation matrices directly.
    This method explicitly shows the chain of transformations.
    """
    L1, L2, L3, L4 = link_lengths
    
    # Base to Joint 1: Rotation about Z
    T01 = np.array([
        [math.cos(j1), -math.sin(j1), 0, 0],
        [math.sin(j1), math.cos(j1), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Joint 1 to Joint 2: Rotation about Y, then translate X by L1
    T12 = np.array([
        [math.cos(j2), 0, math.sin(j2), L1 * math.cos(j2)],
        [0, 1, 0, 0],
        [-math.sin(j2), 0, math.cos(j2), -L1 * math.sin(j2)],
        [0, 0, 0, 1]
    ])
    
    # Joint 2 to Joint 3: Rotation about X, then translate Y by L2
    T23 = np.array([
        [1, 0, 0, 0],
        [0, math.cos(j3), -math.sin(j3), L2 * math.cos(j3)],
        [0, math.sin(j3), math.cos(j3), L2 * math.sin(j3)],
        [0, 0, 0, 1]
    ])
    
    # Joint 3 to Joint 4: Rotation about Y, then translate X by L3
    T34 = np.array([
        [math.cos(j4), 0, math.sin(j4), L3 * math.cos(j4)],
        [0, 1, 0, 0],
        [-math.sin(j4), 0, math.cos(j4), -L3 * math.sin(j4)],
        [0, 0, 0, 1]
    ])
    
    # Joint 4 to End-effector: Translate X by L4
    T4e = np.array([
        [1, 0, 0, L4],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Combine all transformations
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T0e = np.dot(T04, T4e)
    
    # Extract end-effector position
    x = T0e[0, 3]
    y = T0e[1, 3]
    z = T0e[2, 3]
    
    return x, y, z

# Test and demonstration
if __name__ == "__main__":
    # Test cases
    test_cases = [
        # (j1, j2, j3, j4) in degrees, description
        ([0, 0, 0, 0], "All joints at 0°"),
        ([90, 0, 0, 0], "Only base rotated 90°"),
        ([0, 90, 0, 0], "Shoulder joint at 90°"),
        ([0, 0, 90, 0], "Elbow joint at 90°"),
        ([45, 30, 15, 10], "All joints at various angles"),
    ]
    
    link_lengths = [1.0, 1.0, 1.0, 1.0]  # All links 1m each
    
    print("Forward Kinematics Results:")
    print("=" * 60)
    
    for joint_angles_deg, description in test_cases:
        # Convert to radians
        j1, j2, j3, j4 = [math.radians(angle) for angle in joint_angles_deg]
        
        # Calculate forward kinematics using both methods
        pos1 = forward_kinematics(j1, j2, j3, j4, link_lengths)
        pos2 = forward_kinematics_alternative(j1, j2, j3, j4, link_lengths)
        
        print(f"\nTest: {description}")
        print(f"Joint angles: {joint_angles_deg}°")
        print(f"DH Method:     x={pos1[0]:.3f}m, y={pos1[1]:.3f}m, z={pos1[2]:.3f}m")
        print(f"Alt Method:    x={pos2[0]:.3f}m, y={pos2[1]:.3f}m, z={pos2[2]:.3f}m")
        
        # Verify both methods give same result
        diff = np.linalg.norm(np.array(pos1) - np.array(pos2))
        print(f"Difference between methods: {diff:.6f}m")
    
    # Edge case: Verify with known configuration
    print("\n" + "=" * 60)
    print("Edge Case Verification:")
    
    # When all joints are 0, end effector should be at (L1+L2+L3+L4, 0, 0)
    j1, j2, j3, j4 = 0, 0, 0, 0
    expected_x = sum(link_lengths)
    pos = forward_kinematics(j1, j2, j3, j4, link_lengths)
    print(f"All joints at 0° - Expected: ({expected_x}, 0, 0)")
    print(f"Calculated: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    
    # Function to calculate for custom inputs
    def calculate_custom():
        print("\n" + "=" * 60)
        print("Custom Calculation:")
        try:
            j1 = math.radians(float(input("Enter joint 1 angle (degrees): ")))
            j2 = math.radians(float(input("Enter joint 2 angle (degrees): ")))
            j3 = math.radians(float(input("Enter joint 3 angle (degrees): ")))
            j4 = math.radians(float(input("Enter joint 4 angle (degrees): ")))
            
            pos = forward_kinematics(j1, j2, j3, j4, link_lengths)
            print(f"End-effector position: x={pos[0]:.3f}m, y={pos[1]:.3f}m, z={pos[2]:.3f}m")
        except ValueError:
            print("Invalid input. Please enter numeric values.")
    
    # Uncomment the line below to enable custom input
    # calculate_custom()