import numpy as np

# Define constants
OFFSET = 120
L1 = 220
L2 = 273

def inverse_kinematics_dual(x, y):
    def calculate_angle_right(x, y):
        r = np.sqrt(x**2 + y**2)
        cos_theta_2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta_2 = np.arccos(np.clip(cos_theta_2, -1.0, 1.0))

        k1 = L1 + L2 * np.cos(theta_2)
        k2 = L2 * np.sin(theta_2)
        theta_1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return np.degrees(theta_1)
    
    def calculate_angle_left(x, y):
        r = np.sqrt(x**2 + y**2)
        cos_theta_2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta_2 = np.arccos(np.clip(cos_theta_2, -1.0, 1.0))
        theta_2 = -theta_2

        k1 = L1 + L2 * np.cos(theta_2)
        k2 = L2 * np.sin(theta_2)
        theta_1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return np.degrees(theta_1)
    

    theta1_1 = calculate_angle_left(x, y)
    theta1_2 = calculate_angle_right(x - OFFSET, y)

    return theta1_1, theta1_2

#example
angle_left, angle_right = inverse_kinematics_dual(0, 400)

print(angle_left, angle_right)