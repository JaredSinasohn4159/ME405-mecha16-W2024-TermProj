
"""!
@file mlx_cam.py
    This file contains a wrapper that facilitates the use of a Melexis MLX90640
    thermal infrared camera for general use. The wrapper contains a class MLX_Cam
    whose use is greatly simplified in comparison to that of the base class. The code
    (originally from JR Ridgely) has been modified to calibrate sensor output.
@author Sydney Ulvick
@author Jared Sinasohn
@author Sean Nakashimo
@date   2021-Dec-15 JRR Created from the remains of previous example
@copyright (c) 2022-2023 by JR Ridgely and released under the GNU
    Public License, Version 2. 
"""

import utime as time
from machine import Pin, I2C
from mlx90640 import MLX90640
from mlx90640.calibration import NUM_ROWS, NUM_COLS, IMAGE_SIZE, TEMP_K
from mlx90640.image import ChessPattern, InterleavedPattern
from ulab import numpy as np
from cam2setpoint import cam2setpoint


## @brief   Class which wraps an MLX90640 thermal infrared camera driver to
#           make it easier to grab and use an image. 
#  @details This image is in "raw" mode, meaning it has not been calibrated
#           (which takes lots of time and memory) and only gives relative IR
#           emission seen by pixels, not estimates of the temperatures.
class MLX_Cam:
    """! 
    This class implements the MLX90640 for use with our turret. 
    """

    ## @brief   Set up an MLX90640 camera.
    #  @param   i2c An I2C bus which has been set up to talk to the camera;
    #           this must be a bus object which has already been set up
    #  @param   address The address of the camera on the I2C bus (default 0x33)
    #  @param   pattern The way frames are interleaved, as we read only half
    #           the pixels at a time (default ChessPattern)
    #  @param   width The width of the image in pixels; leave it at default
    #  @param   height The height of the image in pixels; leave it at default
    def __init__(self, i2c, address=0x33, pattern=ChessPattern,
                 width=NUM_COLS, height=NUM_ROWS):
        """! 
        Initializes the camera, setting up the I2C address as well as the desired
        csv parameters such as the number of columns and rows.
        @param i2c: the bus which the camera is attached 
        @param address: address of the camera on the bus
        @param pattern: pattern for reading the camera
        @param width: width of the image in pixels
        @param height: height of the image in pixels
        @param ch2: where the timer is being channeled to send to pin2
        """
        self._i2c = i2c #bus object
        self._addr = address #i2c address
        ## The pattern for reading the camera, usually ChessPattern
        self._pattern = pattern
        ## The width of the image in pixels, which should be 32
        self._width = width
        ## The height of the image in pixels, which should be 24
        self._height = height
        ## Tracks whether an image is currently being retrieved
        self._getting_image = False
        ## Which subpage (checkerboard half) of the image is being retrieved
        self._subpage = 0

        # The MLX90640 object that does the work
        self._camera = MLX90640(i2c, address)
        self._camera.set_pattern(pattern)
        self._camera.setup()

        ## A local reference to the image object within the camera driver
        self._image = self._camera.raw
        
    def get_array(self, array, limits=None):
        """! 
        Reads the camera data to form the raw data array of camera data.
        @param array: the bus which the camera is attached 
        @param limits: sets the scale and offset of the array. 
        """
        if limits and len(limits) == 2: #if limits applied/is a tuple
            scale = (limits[1] - limits[0]) / (max(array) - min(array)) #scale factor/ range
            offset = limits[0] - min(array) #min value in array
        else: #if no limits are set, set it to nominal values below:
            offset = 0.0
            scale = 1.0
        arr = np.zeros((self._height, self._width), dtype=np.uint8)
        for row in range(self._height):
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          + offset) * scale)
                arr[row,col] = pix
        return arr
    
    def get_csv(self, array, limits=None):
         """! 
        Reads the camera data to form the raw data csv of camera data.
        @param array: the bus which the camera is attached 
        @param limits: sets the scale and offset of the array. 
        """
        if limits and len(limits) == 2:
            scale = (limits[1] - limits[0]) / (max(array) - min(array))
            offset = limits[0] - min(array)
        else:
            offset = 0.0
            scale = 1.0
        for row in range(self._height):
            line = ""
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          + offset) * scale)
                if col:
                    line += ","
                line += f"{pix}"
            yield line
        return


    ## @brief   Get an image from an MLX90640 camera in a non-blocking way.
    #  @details This function is to be called repeatedly; it will return @c None
    #           until a complete image has been retrieved (this takes around a
    #           quarter to half second) and will then return the image.
    #
    #      @b Example: This code would be inside a task function which yields 
    #      repeatedly as long as there isn't a complete image available.
    #      @code
    #      image = None
    #      while not image:
    #          image = camera.get_image_nonblocking()
    #          yield(state)
    #      @endcode
    #
    def get_image_nonblocking(self):

        # If this is the first recent call, begin the process
        if not self._getting_image:
            self._subpage = 0
            self._getting_image = True
        
        # Read whichever subpage needs to be read, or wait until data is ready
        if not self._camera.has_data:
            return None
        
        image = self._camera.read_image(self._subpage)
        
        # If we just got subpage zero, we need to come back and get subpage 1;
        # if we just got subpage 1, we're done
        if self._subpage == 0:
            self._subpage = 1
            return None
        else:
            self._getting_image = False
            return image


## This test function sets up the sensor, then grabs and shows an image in a
#  terminal every few seconds. By default it shows ASCII art, but it can be
#  set to show better looking grayscale images in some terminal programs such
#  as PuTTY. Unfortunately Thonny's terminal won't show the nice grayscale. 
def test_MLX_cam():

    import gc

    # The following import is only used to check if we have an STM32 board such
    # as a Pyboard or Nucleo; if not, use a different library
    try:
        from pyb import info

    # Oops, it's not an STM32; assume generic machine.I2C for ESP32 and others
    except ImportError:
        # For ESP32 38-pin cheapo board from NodeMCU, KeeYees, etc.
        i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))

    # OK, we do have an STM32, so just use the default pin assignments for I2C1
    else:
        i2c_bus = I2C(1)

    print("MXL90640 Easy(ish) Driver Test")

    # Select MLX90640 camera I2C address, normally 0x33, and check the bus
    i2c_address = 0x33
    scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
    print(f"I2C Scan: {scanhex}")

    # Create the camera object and set it up in default mode
    camera = MLX_Cam(i2c_bus)
    print(f"Current refresh rate: {camera._camera.refresh_rate}")
    camera._camera.refresh_rate = 10.0
    print(f"Refresh rate is now:  {camera._camera.refresh_rate}")

    while True:
        try:
            # Get and image and see how long it takes to grab that image

            # Keep trying to get an image; this could be done in a task, with
            # the task yielding repeatedly until an image is available
            image = None
            while not image:
                image = camera.get_image_nonblocking()
                #time.sleep_ms(50)
            #print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")

            # Can show image.v_ir, image.alpha, or image.buf; image.v_ir best?
            # Display pixellated grayscale or numbers in CSV format; the CSV
            # could also be written to a file. Spreadsheets, Matlab(tm), or
            # CPython can read CSV and make a decent false-color heat plot.
            show_image = False
            show_csv = True
            time.sleep_ms(100)
            #print(camera.get_array(image, limits=(0, 255)))
#             if show_image:
#                 camera.ascii_image(image)
#             elif show_csv:
#                 for line in camera.get_csv(image, limits=(0, 255)):
#                     print(line)
#                 pass
#             else:
#                 camera.ascii_art(image)
#             gc.collect()
            numpy_arr = camera.get_array(image)
            #print(f"Memory: {gc.mem_free()} B free")
            X,Y = cam2setpoint(numpy_arr)
            scale_x = 1
            scale_y = 1
            print(f"{scale_x*X},{scale_y*Y}")

        except KeyboardInterrupt:
            break

    print ("Done.")


if __name__ == "__main__":

    test_MLX_cam()



