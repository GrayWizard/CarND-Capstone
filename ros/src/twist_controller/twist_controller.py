from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, **kwargs):
        self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'], 0.0, kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])
        self.throttle_pid = PID(0.5, 0.002, 0.0, kwargs['decel_limit'], kwargs['accel_limit'])

        self.previous_time = None

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity, dbw_enabled):
        if not dbw_enabled:
            self.throttle_pid.reset()
            return 0.0, 0.0, 0.0

        linear_velocity_error = target_linear_velocity - current_linear_velocity
        current_time = rospy.get_time()
        sample_time = current_time - self.previous_time if self.previous_time is not None else 0.0
        self.previous_time = current_time

        throttle = self.throttle_pid.step(linear_velocity_error, sample_time)

        brake = 0.0
        if throttle < 0.0:
            brake = -throttle
            throttle = 0.0

        steering = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        return throttle, brake, steering
