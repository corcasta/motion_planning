#!/usr/bin/env python3

import sys
import rospy
import time

class TactileSensor():
    def __init__(self):
        self.u = 0
        self.v = 0


class PID():
    def __init__(self, gains):
        """
        gains: type dict, {"kp": float, "ki": float, "kd": float}
        """

        self.accum = 0.0
        self.prev_time = 0.0
        self.delta_time = 0.0
        self.prev_error = 0.0
        
        if gains:
            self.kp = gains["kp"]
            self.ki = gains["kd"]
            self.kd = gains["ki"]
        else:
            self.kp = 1.0
            self.ki = 0.0
            self.kd = 0.0

    def update_time_variables(self):
        self.delta_time = time.time() - self.prev_time
        self.prev_time += self.delta_time

    def update_gains(self, kp, kd, ki):
        """
        kp: type float, CONTROLLER GAINS
        ki: type float, CONTROLLER GAINS
        kd: type float, CONTROLLER GAINS
        """
        if kp:
            self.kp = kp
        if ki:
            self.ki = ki
        if kd:
            self.kd = kd

    def propotional_block(self, error):
        return self.kp*error

    def integral_block(self, error):
        self.accum += self.ki*(error*self.delta_time)
        return self.accum

    def derivative_block(self, error):
        return self.kd * (error - self.prev_error)/self.delta_time
    
    def forward(self, error):
        # MANDATORY to call first "update_time_variables" before calling PID blocks
        self.update_time_variables()
        return self.proportional_block(error) + self.integral_block(error) + self.derivative_block(error)

        
        


    
class ForceConroller():
    def __init__(self, friction_coeff, max_output_val, gains):
        """
        friction_coeff
        max_output_val
        gains
        
        """
        self.friction_coeff = friction_coeff
        self.max_output_val = max_output_val  
        self.kp = gains["kp"]
        self.ki = gains["kd"]
        self.kd = gains["ki"]

        self.pid_layer
        self.gripper_layer
        self.tactile_sensor_layer
        
    # setpoint, measure_varible, external_tracking_signal
    def pid_layer(self, error):

        output = self.kp*error + self.ki + self.kd

    def forward(self, error):
        x = self.pid_layer(error)
        x = self.gripper_layer(x)
        x = self.tactile_sensor_layer(x)
        return x
        

    def backward(self):
        pass






def main():
    pass

if __name__ == "__main__":
    main()