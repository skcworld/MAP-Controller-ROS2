import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.logging import get_logger

def find_nearest(array, value):
    """
    Find the nearest value in an array
    """
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx

def find_closest_neighbors(array, value):
    """
    Find the two closest values in an array
    """
    idx = (np.abs(array - value)).argmin()
    
    if idx == len(array) - 1 and array[idx] < value:
        return array[idx], idx, array[idx], idx
    elif idx == 0 and array[idx] > value:
        return array[idx], idx, array[idx], idx
    elif array[idx] > value and idx > 0:
        return array[idx-1], idx-1, array[idx], idx
    elif array[idx] < value and idx < len(array) - 1:
        return array[idx], idx, array[idx+1], idx+1
    else:
        return array[idx], idx, array[idx], idx

class LookupSteerAngle:
    """
    LookupSteerAngle: Lookup steering angle from a lookup table based on acceleration and velocity
    """
    def __init__(self, model_name):
        """
        Initialize the LookupSteerAngle class
        """
        try:
            # ROS2 방식으로 패키지 경로 찾기
            path = get_package_share_directory('steering_lookup')
            file_path = os.path.join(path, 'cfg', f'{model_name}_lookup_table.csv')
            self.lu = np.loadtxt(file_path, delimiter=",")
            self.logger = get_logger('lookup_steer_angle')
        except Exception as e:
            if rclpy.ok():
                self.logger = get_logger('lookup_steer_angle')
                self.logger.error(f"Error loading lookup table: {str(e)}")
            print(f"Error loading lookup table: {str(e)}")
            raise

    def lookup_steer_angle(self, accel, vel):
        """
        Lookup a steering angle based on acceleration and velocity
        
        Parameters:
        - accel: Lateral acceleration (m/s^2)
        - vel: Velocity (m/s)
        
        Returns:
        - Steering angle (rad)
        """
        if accel > 0.0:
            sign_accel = 1.0
        else:
            sign_accel = -1.0
        
        # lookup only for positive accelerations
        accel = abs(accel)
        lu_vs = self.lu[0, 1:]
        lu_steers = self.lu[1:, 0]
        
        if (vel > lu_vs[-1]):
            txt = f"Velocity exceeds lookup table, generating steering angle for v: {lu_vs[-1]}"
            if rclpy.ok():
                self.logger.warn(txt, once=True)
            vel = lu_vs[-1]
        
        # find closest velocity
        c_v, c_v_idx = find_nearest(lu_vs, vel)
        
        # find two closest accelerations to accel
        c_a, c_a_idx, s_a, s_a_idx = find_closest_neighbors(self.lu[1:, c_v_idx + 1], accel)
        
        if c_a_idx == s_a_idx:
            steer_angle = lu_steers[c_a_idx]
        else:
            # interpolate between two closest accelerations to find steering angle
            steer_angle = np.interp(accel, [c_a, s_a], [lu_steers[c_a_idx], lu_steers[s_a_idx]])
        
        return steer_angle * sign_accel


if __name__ == "__main__":
    rclpy.init()
    detective = LookupSteerAngle("NUC1_pacejka")
    steer_angle = detective.lookup_steer_angle(9, 7)
    print(steer_angle)
    rclpy.shutdown()