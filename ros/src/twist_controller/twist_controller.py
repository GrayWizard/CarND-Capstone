from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, **kwargs):
        vehicle_mass = kwargs['vehicle_mass']
        steer_ratio = kwargs['steer_ratio']
        min_speed = 0
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        decel_limit = kwargs['decel_limit']
        accel_limit = kwargs['accel_limit']
        wheel_radius = kwargs['wheel_radius']

        self.brake_deadband = kwargs['brake_deadband']
        self.brake_tourque_const = vehicle_mass * wheel_radius
        self.yaw_controller = YawController(vehicle_mass, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.linear_pid = PID(0.9, 0.0005, 0.07, decel_limit, accel_limit)
        self.lowpass_filter = LowPassFilter(0.2,0.1)

        self.previous_time = None
        self.dbw_enabled = False


    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        if dbw_enabled and not self.dbw_enabled:
            self.dbw_enabled = True
            self.linear_pid.reset()
            self.previous_time = None
        else:
            self.dbw_enabled = False

        linear_velocity_error = linear_velocity - current_velocity
        current_time = rospy.get_time()
        sample_time = current_time - self.previous_time if self.previous_time else 0.05
        self.previous_time = current_time

        throttle = self.linear_pid.step(linear_velocity_error, sample_time)
        throttle = self.lowpass_filter.filt(throttle)

        brake = 0
        if throttle < 0:
            decel = abs(throttle)
            brake = self.brake_tourque_const * decel if decel > self.brake_deadband else 0
            throttle = 0

        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        return throttle, brake, steering
