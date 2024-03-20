"""!
@file servo_driver.py
This file contains the class implementation for powering our servo.
This file also tests if the servo can function to a setpoint.

@author Jared Sinasohn, Sydney Ulvick, Sean Nakashimo
@date 3-March-2024
"""
from pyb import Pin, Timer
import utime
frequency = 50
class Servo:
    """! 
    This class implements a servo. 
    """
    def __init__(self,servo_pin,timer):
        """! 
        Creates a servo driver.
        @param servo_pin: output pin on GPIO which sends PWM
        @param timer: timer used to setup PWM
        """
        self.servo_pin = servo_pin
        self.timer = timer
        self.channel = self.timer.channel(1, Timer.PWM, pin=servo_pin)
        print("creating servo")

    def set_servo(self,angle):
        """! 
        Sets servo position based on angle.
        @param angle: output angle, 0-180 degrees
        """
        # 40KG servo DS3240MG
        # Positions of 0-180
        # PWM range of 500-2500 microseconds
        
        # Converting degrees into PWM percentage, accounting for range from specs
        duty_cycle = ((angle / 180) * (2500 - 500) + 500)/1000000
        duty_cycle = duty_cycle*frequency*100
        self.channel.pulse_width_percent(duty_cycle)
        
            #print('1')
            #set_servo(0)
            #utime.sleep(1)
if __name__ =='__main__':
    # Defining a different pin with an alternate function for Timer 2
    servo_pin = pyb.Pin(pyb.Pin.cpu.E5, pyb.Pin.OUT_PP)
    # Create a Timer object for PWM
    timer = Timer(15, freq=frequency)  # Timer 2 with a frequency of 50 Hz
    servo = Servo(servo_pin,timer)
    try:
        angle = 15
        servo.set_servo(int(angle))
        utime.sleep(0.5)
        angle = 0
        servo.set_servo(int(angle))
        utime.sleep(0.5)
        pass   

    except KeyboardInterrupt:
        raise KeyboardInterrupt
        pass  # Handling keyboard interrupt gracefully
