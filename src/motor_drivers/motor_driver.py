"""!
@file motor_driver.py
This file contains the class implementation for powering an ametek pittman motor.
This file also contains testing code to test if the motor runs, and if it can handle
bad pwm levels.

@author Jared Sinasohn, Sydney Ulvick, Sean Nakashimo
@date 15-Feb-2024
"""
import pyb
import utime
import micropython

class MotorDriver:
    """! 
    This class implements a motor driver for an ME405 kit. 
    """

    def __init__ (self, en_pin, in1pin, in2pin, timer):
        """! 
        Creates a motor driver by initializing GPIO
        pins and turning off the motor for safety.
        @param en_pin: enabling pin for the motor
        @param in1pin: where the PWM is being sent for 1
        @param in2pin: where the PWM is being sent for 2
        @param timer: timer that the motor uses for the PWM
        @param ch1: where the timer is being channeled to send to pin1
        @param ch2: where the timer is being channeled to send to pin2
        """
        print ("Creating a motor driver")
        # defining parameters needed to characterize motor
        self.en_pin = en_pin 
        self.in1pin = in1pin
        self.in2pin = in2pin
        self.timer = timer
        self.en_pin.low() #disable the motor (for safety)
        self.ch1 = self.timer.channel(2, pyb.Timer.PWM, pin=self.in1pin)
        self.ch2 = self.timer.channel(4, pyb.Timer.PWM, pin=self.in2pin)
        self.ch1.pulse_width_percent(0)
        self.ch2.pulse_width_percent(0)
    
        
    def set_duty_cycle (self, level):
        """!
        This method sets the duty cycle to be sent
        to the motor to the given level. Positive values
        cause torque in one direction, negative values
        in the opposite direction.  This function also
        saturates the level if the level is greater than 100
        or -100
        @param level A signed integer holding the duty
               cycle of the voltage sent to the motor 
        """
        #setting the duty cycle
        try:
            level = float(level)
            if level < 0: #for negative in range
                self.en_pin.high() #enable the motor
                self.ch1.pulse_width_percent(0) #set in2 to zero
                # if the level is greater than -100, nothing happens
                # if it is less than -100, set the PWM to 100
                # this saturates the level to -100 regardless of how big the level is
                if level >= -100:
                    self.ch2.pulse_width_percent(abs(level))
                else:
                    self.ch2.pulse_width_percent(100)
            elif level > 0: #for positive in range
                self.en_pin.high() #enable the motor
                self.ch2.pulse_width_percent(0) #set in1 to zero
                # if the level is less than 100, nothing happens
                # if it is greater than 100, set the PWM to 100
                # this saturates the level to 100 regardless of how big the level is
                if level <= 100:
                    self.ch1.pulse_width_percent(level)
                else:
                    self.ch1.pulse_width_percent(100)  
            else:
                self.ch1.pulse_width_percent(0)
                self.ch2.pulse_width_percent(0)
        except ValueError:
            self.ch1.pulse_width_percent(0)
            self.ch2.pulse_width_percent(0)
            raise ValueError

if __name__ == "__main__":
    # power the motor for five seconds 
    en_pin =  pyb.Pin(pyb.Pin.board.PG14, mode = pyb.Pin.OPEN_DRAIN, pull = pyb.Pin.PULL_UP, value=1)
    in1pin = pyb.Pin(pyb.Pin.board.PE14, pyb.Pin.OUT_PP)
    in2pin = pyb.Pin(pyb.Pin.board.PE11, pyb.Pin.OUT_PP)
    timer = pyb.Timer(3, freq=5000) #setting frequency for motor 
    motor = MotorDriver(en_pin,in1pin,in2pin,timer,ch1,ch2) #call to the motor class you just made!
    motor.set_duty_cycle(50) #set duty cycle, in range -100 to 100 (not including 0)
    utime.sleep(5)
    motor.set_duty_cycle(0)
    





