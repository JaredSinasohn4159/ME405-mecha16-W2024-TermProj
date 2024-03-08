"""!
@file encoder_reader.py
This file contains the class implementation for reading the values on the ME 405 microcontroller
and ametek pittman motors.  This file also contains testing code to test if the encoder can be read,
if the encoder can handle overflows both positively and negatively, and if the encoder can be zeroed.

@author Jared Sinasohn, Sydney Ulvick, Sean Nakashimo
@date 15-Feb-2024
"""
import micropython
import pyb
import utime

class Encoder:
    """! 
    This class implements an encoder for an ME405 kit. 
    """

    def __init__ (self, pin1, pin2, timer, conversion_factor=1):
        """! 
        Creates a motor driver by initializing GPIO
        pins, setting up timer channels to read encoder values.
        @param pin1 - the pin that encoder channel A will send data to
        @param pin2 - the pin that encoder channel B will send data to
        @param timer - the timer the encoder will use for counting its ticks
        @param conversion_factor - The factor by which the encoder readings should be
                                    divided by to get the desired endcoder output units.
                                    The conversion factor is in encoder counts per degree.
        @param ch1 - the PWM timing channel used by the first input pin
        @param ch2 - the PWM timing channel used by the second input pin
        @param pos - the position of the encoder
        @param prev - the previous reading of the encoder
        @param new - the new value of the encoder that has just been read
        """
        # initialize passed through variables to self variables
        self.pin1 = pin1
        self.pin2 = pin2
        self.timer = timer
        self.conversion_factor = conversion_factor
        # set up each channel for data collection.  Both should trigger on
        # both encoder channel edges, and set to their respective pins
        self.ch1 = self.timer.channel(1, pyb.Timer.ENC_AB, pin=self.pin1)
        self.ch2 = self.timer.channel(2, pyb.Timer.ENC_AB, pin=self.pin2)
        print("\nCreating encoder")
        
        # initialize counter to zero
        self.timer.counter(0)
        
        # set the initial encoder position to the counter
        self.pos = self.timer.counter()
        
        # initialize previous encoder val and new encoder val.  Both are
        # used in delta calculations
        self.prev = 0
        self.new = 0

    def read(self):
        """!
        This method reads the encoder value and returns the new encoder position
        It accounts for overflows and adjusts accordingly.
        @returns self.pos - the position of the encoder
        """
        
        # the AR is the max value of the encoder.  This is equal to the period
        # of the timer
        AR = self.timer.period()
        
        # the new encoder value is the current value of the timer
        self.new = self.timer.counter()
        
        # delta is the difference between the previous encoder value and the new encoder value
        delta = self.new-self.prev
        
        # the overflow value is the value that indicates there is probably an overflow
        # it is half of the maximum encoder value
        overflow = (AR+1)/2
        
        # if the delta is bigger than the overflow value, the value underflowed and so
        # we need to subtract the period + 1 from the delta
        if delta >= overflow:
            delta -= overflow*2
            
        # if the delta is smaller than negative of the overflow value, the value overflowed and so
        # we need to add the period + 1 to the delta
        elif delta <= -1*overflow:
            delta += overflow*2
            
        # if neither of these conditions are met, we good
        else:
            delta = delta
            
        # add the delta to the position
        # note, for some reason the delta when it overflows will convert to a float
        # so i am casting to an int to compensate.  I think it may be because my
        # AR value is using a single backslash but I am not certain
        self.pos += int(delta)
        
        # the current motor count is now the previous motor count for the next reading
        self.prev = self.new
        
        # return the position (originally in counts) converted to degrees
        return self.pos/self.conversion_factor
        
    
    def zero(self):
        """!
        This method resets the current encoder value and the
        position of the encoder to zero.
        """
        # set the value of the counter to zero
        self.timer.counter(0)
        # set the value of the encoder of the position within the class
        # to zero
        self.pos = 0
        # reset previous value and new value so we don't mess up
        # future calculations
        self.prev = 0
        self.new = 0
        
    def set_pos(self, val):
        """!
        This method sets the encoder to the desired value in degrees
        @param val - the value to set the encoder to in degrees
        """
        val_converted = val*self.conversion_factor
        # set the value of the counter to the val converted to encoder count
        self.timer.counter(val_converted)
        # set the value of the encoder of the position within the class
        # to the converted value
        self.pos = val_coverted
        # set previous value and new value so we don't mess up
        # future calculations
        self.prev = val_coverted
        self.new = val_coverted
        
        
if __name__ == "__main__":
    # Testing code to test encoder.  This code does not run the motor, the motor
    # is simply hand spun to determine if the encoder reads properly.
    # This code does not run if the file is imported as a module
    
    # create the pin object to read encoder channel A
    pin1 = pyb.Pin(pyb.Pin.board.PC6, pyb.Pin.IN)
    
    # create the pin object to read encoder channel B
    pin2 = pyb.Pin(pyb.Pin.board.PC7, pyb.Pin.IN)
    
    # create the timer object.  For C6 and C7 use timer 8,
    # set the prescaler to zero and the period to the max 16bit number
    timer = pyb.Timer(8, prescaler = 0, period = 65535)
    
    # create the encoder object
    encoder = Encoder(pin1, pin2, timer)
    
    # initial encoder reading, set it to the previous value
    # (this is just for initializing)
    prev = encoder.read()
    
    # infinite loop
    while(True):
        # set the new encoder value to the encoder value just read
        new = encoder.read()
        
        # only print the encoder value if the encoder value has changed
        # This makes the terminal much more readable
        if new != prev:
            print(f"\nEncoder value: {new}")
            
        # set the previous val to the current val for next loop
        prev = new
        
        # to test zeroing without stopping overflow tests, zero if the
        # encoder has reached 100000 or -100000
        if new >= 100000 or new <= -100000 :
            encoder.zero()

        # sleep for 10 ms before checking again.
        # in actuality we should check this often enough so at the max speed
        # the encoder won't overflow due to a rotation being completed before
        # the code updates.  10ms when pushed by a hand is probably
        # fine.  We haven't overflowed yet.
        utime.sleep_ms(10)

