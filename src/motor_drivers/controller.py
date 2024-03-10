"""!
@file controller.py
This file contains the class implementation for controlling a plant based on a sensor.
The controller class implements a PID controller to control a sensor input based on a setpoint.
This file also creates a test file to run an Ametek Pittman Motor to a set position based on
an encoder input reading.

@author Jared Sinasohn, Sydney Ulvick, Sean Nakashimo
@date 22-Feb-2024
"""

import micropython
import pyb
import utime
from motor_drivers.encoder_reader import Encoder
from motor_drivers.motor_driver import MotorDriver

class CLController:
    """! 
    This class implements a closed loop controller based on an input sensor.  This class uses previously created . 
    """

    def __init__ (self, kp, ki, kd, setpoint):
        """! 
        Creates a motor driver by initializing GPIO
        pins and turning off the motor for safety. 
        @param sensor - the sensor the controller will be using and reading to calculate error
        @param kp - proportional controller constant
        @param ki - integral controller constant
        @param kd - derivative controller constant
        @param setpoint - the target postition for the controller to aim for
        @param eff - the effort the controller aims to send to the plant in percentage
        @param curr - the current position/value of the plant
        @param err_acc - the accumulation of the error used for integral control
        @param prev_error - a list of previous errors used to calculate derivative control
        @param self.deriv_amount - the amount of time for each update of the controller (roughly)
        @param self.initial_time - the initial time for when the controller starts
        @param self.curr_time - the current time of control
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        # the effort the controller will send to the motor in percentage
        self.eff = 0
        # current value is the current reading of the sensor
        self.curr = 0
        # error is how far current value is from sensor
        self.err = self.setpoint - self.curr
        self.err_acc = 0
        self.prev_err_list = []
        self.deriv_amount = 10
        self.update_time = 10
        self.initial_time = utime.ticks_ms()
        self.curr_time = utime.ticks_diff(utime.ticks_ms(),self.initial_time)
    
    # why would this method accept the setpoint
    def run(self, measured):
        """!
        This method calculates the the effort to apply to an actuator based
        measured sensor value and a desired setpoint.  This method implements a
        PID Controller approach
        @param measured - the measured value from the sensor
        @returns self.eff - the effort the motor should push at in terms of percentage
        """
        self.curr = measured
        self.curr_time = utime.ticks_diff(utime.ticks_ms(),self.initial_time)
        self.err = self.setpoint - self.curr
        self.err_acc += self.err
        self.eff = self.kp*self.err + self.ki*self.err_acc
        if self.kd > 0:
            self.prev_err_list.append(self.err)
        if self.kd > 0 and len(self.prev_err_list) >= self.deriv_amount:
            err_slope = (self.prev_err_list[self.deriv_amount-1]-self.prev_err_list[0])/(self.deriv_amount*self.update_time)
            self.eff += self.kd*err_slope
            self.prev_err_list.pop(0)
        return self.eff

    def set_setpoint(self, setpoint):
        """!
        This method sets the setpoint based on the function input
        @param setpoint - the setpoint to set the controller to
        """
        self.setpoint = setpoint
        
    def set_kp(self, kp):
        """!
        This method sets the kp based on the function input
        @param kp - the proportional gain for the proportional part of the PID controller
        """
        self.kp = kp
    
    def set_ki(self, ki):
        """!
        This method sets the ki based on the function input
        @param ki - the integral gain for the integral part of the PID controller
        """
        self.ki = self.ki

    def set_kd(self, kd):
        """!
        This method sets the ki based on the function input
        @param kd - the derivative gain for the derivative part of the PID controller
        """
        self.kd = self.kd

    def get_pos(self):
        """!
        This method returns the current position/value of the plant the controller is controlling
        @returns self.curr - the current position/value of the plant
        """
        return self.curr
    
    def get_curr_time(self):
        """!
        This method returns the current time of operation of the controller
        @returns self.time - the current time of operation of the controller
        """
        return self.curr_time
    
    def reset_controller(self):
        """!
        This method resets the controller to run again
        """
        self.err = 0
        self.eff = 0
        self.err_acc = 0
        self.curr = 0
        self.initial_time = utime.ticks_ms()
        self.curr_time = utime.ticks_ms()-self.initial_time
        
        
if __name__ == "__main__":
    # create pin to power motor
    en_pin =  pyb.Pin(pyb.Pin.board.PA10, mode = pyb.Pin.OPEN_DRAIN, pull = pyb.Pin.PULL_UP, value=1)
    
    # create first pwm pin
    in1pin = pyb.Pin(pyb.Pin.board.PB4, pyb.Pin.OUT_PP)
    
    # create first pwm pin
    in2pin = pyb.Pin(pyb.Pin.board.PB5, pyb.Pin.OUT_PP)
    
    # create timer for pwm
    timer = pyb.Timer(3, freq=25000) #setting frequency for motor
    
    # create motor object
    motor = MotorDriver(en_pin,in1pin,in2pin,timer) #call to the motor class you just made!

    # create the pin object to read encoder channel A
    pin1 = pyb.Pin(pyb.Pin.board.PC6, pyb.Pin.IN)
    
    # create the pin object to read encoder channel B
    pin2 = pyb.Pin(pyb.Pin.board.PC7, pyb.Pin.IN)
    
    # create the timer object.  For C6 and C7 use timer 8,
    # set the prescaler to zero and the period to the max 16bit number
    timer = pyb.Timer(8, prescaler = 0, period = 65535)
    
    # create the encoder object
    encoder = Encoder(pin1, pin2, timer)
    
    # create controller object
    con = CLController(1.63, 0, 0.3, 90)
    while True:
        try:
            encoder_reading = encoder.read()
            encoder_angle = encoder_reading/16/256/4*360
            eff = con.run(encoder_angle)
            motor.set_duty_cycle(eff)
        except KeyboardInterrupt:
            motor.set_duty_cycle(0)
            raise KeyboardInterrupt
        except ValueError:
            motor.set_duty_cycle(0)
            raise ValueError
        utime.sleep_ms(1)
