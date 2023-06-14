from pid import PIDController
import numpy as np

class Controller:

    def __init__(self, args) -> None:
        self.linear_kp, self.linear_ki, self.linear_kd = args.linear_kp, args.linear_ki, args.linear_kd
        self.angular_kp, self.angular_ki, self.angular_kd = args.angular_kp, args.angular_ki, args.angular_kd
        self.dt = args.dt
        self.wheel_radius = args.wheel_radius
        self.wheel_separation = args.wheel_separation

        self.linear_pid = PIDController(self.linear_kp, self.linear_ki, self.linear_kd, self.dt)
        self.angular_pid = PIDController(self.angular_kp, self.angular_ki, self.angular_kd, self.dt)
    
    @staticmethod
    def check_arrived(ex, ey):
        if ex < 0.2 and ey < 0.2:
            return True
        else:
            return False
        
    def move_to_position(self, x, y, target_x, target_y, theta, linear_pid, angular_pid):
        dx = target_x - x
        dy = target_y - y

        if self.check_arrived(dx, dy):
            return 0, 0
        
        target_theta = np.arctan2(dy, dx)

        linear_distance = np.sqrt(dx**2 + dy**2)
        angular_distance = target_theta - theta

        v = linear_pid.update(0, -linear_distance)
        w = angular_pid.update(0, -angular_distance)

        return v, w
    

    def compute_wheel_velocities(self, x, y, theta, target_x, target_y):
        v, w = self.move_to_position(x, y, target_x, target_y, theta, self.linear_pid, self.angular_pid)
        
        left_wheel_velocity = (v - w * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_velocity = (v + w * self.wheel_separation / 2) / self.wheel_radius
        
        return left_wheel_velocity, right_wheel_velocity
