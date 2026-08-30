#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('previous_speed_error', 0.0)
        self.vars.create_var('integral_speed_error', 0.0)
        self.vars.create_var('previous_timestamp', 0.0)
        self.vars.create_var('pid_initialized', False)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            ######################################################
            # HIGH LEVEL LONGITUDINAL PID
            ######################################################
            # The PID output is a requested longitudinal acceleration, not a
            # throttle command.  These gains and limits are intentionally kept
            # together so they can be tuned from simulation results.
            kp = 1.00
            ki = 0.08
            kd = 0.05
            max_acceleration = 3.0       # m/s^2
            max_deceleration = -5.0      # m/s^2
            integral_error_limit = 10.0  # (m/s) * s
            derivative_limit = 20.0      # m/s^2

            speed_error = v_desired - v
            dt = 0.0
            derivative_error = 0.0

            if self.vars.pid_initialized:
                dt = t - self.vars.previous_timestamp
                # A normal run is near 1/30 s.  Ignore duplicate, reversed,
                # or very large timestamp steps instead of injecting a spike.
                if dt > 0.0 and dt <= 1.0:
                    derivative_error = (
                        (speed_error - self.vars.previous_speed_error) / dt)
                    derivative_error = np.fmax(
                        np.fmin(derivative_error, derivative_limit),
                        -derivative_limit)
                else:
                    dt = 0.0

            candidate_integral = self.vars.integral_speed_error
            if dt > 0.0:
                candidate_integral += speed_error * dt
                candidate_integral = np.fmax(
                    np.fmin(candidate_integral, integral_error_limit),
                    -integral_error_limit)

            acceleration_unclamped = (kp * speed_error +
                                      ki * candidate_integral +
                                      kd * derivative_error)

            # Conditional integration prevents the integral term from winding
            # up while the acceleration request is saturated in the same
            # direction as the speed error.
            is_saturated_high = acceleration_unclamped > max_acceleration
            is_saturated_low = acceleration_unclamped < max_deceleration
            if ((is_saturated_high and speed_error > 0.0) or
                    (is_saturated_low and speed_error < 0.0)):
                candidate_integral = self.vars.integral_speed_error
                acceleration_unclamped = (kp * speed_error +
                                          ki * candidate_integral +
                                          kd * derivative_error)

            self.vars.integral_speed_error = candidate_integral
            a_desired = np.fmax(
                np.fmin(acceleration_unclamped, max_acceleration),
                max_deceleration)

            ######################################################
            # LOW LEVEL LONGITUDINAL DYNAMIC MODEL
            ######################################################
            # Approximate passenger-car parameters.  CARLA 0.8.4 does not
            # expose the selected vehicle's exact engine/brake force here, so
            # the force limits serve as tunable actuator normalizations.
            vehicle_mass = 1500.0          # kg
            gravity = 9.81                  # m/s^2
            rolling_resistance = 0.015      # dimensionless Crr
            air_density = 1.225             # kg/m^3
            drag_coefficient = 0.30         # dimensionless Cd
            frontal_area = 2.20             # m^2
            maximum_drive_force = 5000.0    # N
            maximum_brake_force = 12000.0   # N
            force_deadband = 50.0            # N

            forward_speed = np.fmax(v, 0.0)
            rolling_force = rolling_resistance * vehicle_mass * gravity
            drag_force = (0.5 * air_density * drag_coefficient *
                          frontal_area * forward_speed * forward_speed)
            required_force = (vehicle_mass * a_desired +
                              rolling_force + drag_force)

            # The branches are mutually exclusive, so throttle and brake are
            # never applied together.
            if required_force > force_deadband:
                throttle_output = required_force / maximum_drive_force
                brake_output = 0.0
            elif required_force < -force_deadband:
                throttle_output = 0.0
                brake_output = -required_force / maximum_brake_force
            else:
                throttle_output = 0.0
                brake_output = 0.0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            ######################################################
            # PURE PURSUIT LATERAL CONTROLLER
            ######################################################
            # Measurements are at the vehicle center.  Assuming the front
            # axle is 1.5 m ahead and the vehicle is approximately symmetric,
            # the wheelbase is 3.0 m and the rear axle is 1.5 m behind center.
            wheelbase = 3.0
            center_to_rear_axle = 1.5
            minimum_lookahead = 3.0       # m
            lookahead_velocity_gain = 0.35  # s
            maximum_lookahead = 10.0      # m
            maximum_steering_angle = 1.22 # rad

            lookahead_distance = (minimum_lookahead +
                                  lookahead_velocity_gain * forward_speed)
            lookahead_distance = np.fmax(
                np.fmin(lookahead_distance, maximum_lookahead),
                minimum_lookahead)

            # Shift the control point from the measured center to the rear
            # axle, which is the point used by the Pure Pursuit bicycle model.
            rear_x = x - center_to_rear_axle * np.cos(yaw)
            rear_y = y - center_to_rear_axle * np.sin(yaw)

            if len(waypoints) > 0:
                waypoint_array = np.asarray(waypoints)
                delta_x = waypoint_array[:, 0] - rear_x
                delta_y = waypoint_array[:, 1] - rear_y
                squared_distances = delta_x * delta_x + delta_y * delta_y
                closest_index = int(np.argmin(squared_distances))

                # Walk forward along the path from its closest point until the
                # accumulated arc length reaches the speed-dependent lookahead.
                target_index = closest_index
                path_distance = 0.0
                while (target_index < len(waypoints) - 1 and
                       path_distance < lookahead_distance):
                    segment_x = (waypoints[target_index + 1][0] -
                                 waypoints[target_index][0])
                    segment_y = (waypoints[target_index + 1][1] -
                                 waypoints[target_index][1])
                    path_distance += np.sqrt(segment_x * segment_x +
                                             segment_y * segment_y)
                    target_index += 1

                target_x = waypoints[target_index][0]
                target_y = waypoints[target_index][1]
                target_delta_x = target_x - rear_x
                target_delta_y = target_y - rear_y

                # In CARLA's left-handed x-y plane, +yaw and +local-y point
                # toward a right turn.  Positive steering is also right, so no
                # additional sign inversion is required.
                local_x = (np.cos(yaw) * target_delta_x +
                           np.sin(yaw) * target_delta_y)
                local_y = (-np.sin(yaw) * target_delta_x +
                           np.cos(yaw) * target_delta_y)
                alpha = np.arctan2(local_y, local_x)
                alpha = (alpha + self._pi) % self._2pi - self._pi

                actual_target_distance = np.sqrt(
                    target_delta_x * target_delta_x +
                    target_delta_y * target_delta_y)
                actual_target_distance = np.fmax(actual_target_distance, 0.001)

                steer_output = np.arctan2(
                    2.0 * wheelbase * np.sin(alpha),
                    actual_target_distance)
                steer_output = np.fmax(
                    np.fmin(steer_output, maximum_steering_angle),
                    -maximum_steering_angle)
            else:
                steer_output = 0.0

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.previous_speed_error = v_desired - v
        self.vars.previous_timestamp = t
        self.vars.pid_initialized = True
