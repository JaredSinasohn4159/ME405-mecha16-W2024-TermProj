# YAW A
# EN_PIN = G12

# Timer 1
# IN_PIN_1 = E9
# IN_PIN_2 = E11

# Timer 3
# Encoder_A = A6
# Encoder_B = B5

# PITCH B
# EN_PIN = G14

# Timer 4
# IN_PIN_1 = B6
# IN_PIN_2 = B7

# Timer 2
# Encoder_A = A15
# Encoder_B = B3

# TRIGGER
# Timer 15
# Signal_pin = E5

"""!
@file main.py
    This is the main file used for operating the thermal camera and turret
    for the ME 405 term project. It uses the mlx 90640 thermal camera,
    two motors for yaw and pitch, and a servo to drive the trigger (fire)
@author Sydney Ulvick
@author Jared Sinasohn
@author Sean Nakashimo
@date   2021-Dec-15 JRR Created from the remains of previous example
@copyright (c) 2015-2021 by JR Ridgely and released under the GNU
    Public License, Version 2. 
"""
import utime
import gc
import pyb
import cotask
import task_share
from motor_drivers.encoder_reader import Encoder
from motor_drivers.motor_driver import MotorDriver
from motor_drivers.controller import CLController
from motor_drivers.servo_driver import Servo
from mlx_cam import MLX_Cam as Cam
from ulab import numpy as np
from machine import Pin, I2C
from cam2setpoint import cam2setpoint

WAIT_TIME = 13000 #wait time (ms) to stop program after starting code
class MotorContainer:
    def __init__(self):
        #setting up yaw/ panning axis motor
        en_pin =  pyb.Pin(pyb.Pin.cpu.G12, mode = pyb.Pin.OPEN_DRAIN, pull = pyb.Pin.PULL_UP, value=1)
        in1pin = pyb.Pin(pyb.Pin.cpu.E9, pyb.Pin.OUT_PP)
        in2pin = pyb.Pin(pyb.Pin.cpu.E11, pyb.Pin.OUT_PP)
        timer = pyb.Timer(1, freq=20000) #setting frequency for motor 
        self.yaw_motor = MotorDriver(en_pin,in1pin,in2pin,timer) #create motor object
        self.yaw_motor.set_duty_cycle(0) #turning motor off for safety
        
        #setting up pitch/ uppy downy angle motor
        en_pin =  pyb.Pin(pyb.Pin.cpu.G14, mode = pyb.Pin.OPEN_DRAIN, pull = pyb.Pin.PULL_UP, value=1)
        in1pin = pyb.Pin(pyb.Pin.cpu.B6, pyb.Pin.OUT_PP)
        in2pin = pyb.Pin(pyb.Pin.cpu.B7, pyb.Pin.OUT_PP)
        timer = pyb.Timer(4, freq=20000) #setting frequency for motor 
        self.pitch_motor = MotorDriver(en_pin,in1pin,in2pin,timer) #create motor object
        self.pitch_motor.set_duty_cycle(0) #turning off motor for safety
        
        #setting up servo
        servo_pin = pyb.Pin(pyb.Pin.cpu.E5, pyb.Pin.OUT_PP) # Defining a different pin with an alternate function for Timer 2
        # Create a Timer object for PWM
        timer = pyb.Timer(15, freq=50)  # Timer 2 with a frequency of 50 Hz, common for servos
        self.servo = Servo(servo_pin,timer) #create servo object
        
        # disable turns both channels PWM off and EN pin to low
    def disable_yaw(self):
         self.yaw_motor.disable()
         
    def disable_pitch(self):
         self.pitch_motor.disable()
         
    def disable_servo(self):
        self.servo.set_servo(0)
        
        
def get_conversion_factor(belt_ratio):
    """!
    This function calculates the conversion of the encoder ticks to degrees of the output axis of the motor.
    @param belt_ratio - the ratio of the belt, output/input
    """
    ticks_per_rev = 256 # ticks per rev of the encoder
    motor_gearbox = 16  # ratio of the gearboxes on the ametek pittman motor
    counts_per_tick = 4 # number of edge counts per encoder tick
    rev_per_deg = 1/360 # conversion from revolutions to degrees
    mech_advantage = motor_gearbox*belt_ratio # mechanical advantage of the whole motor-gearbox setup
    co_fac = counts_per_tick*ticks_per_rev*rev_per_deg*mech_advantage # the conversion factor in counts/degree
    return co_fac

def camera_handler_fun():
    """!
    This function implements the camera handler task and its finite state machine
    """
    t1state = 0 #set state variable to 0 to ensure it inits
    #zero camera parameters 
    image = None
    im_arr = None
    camera = None
    regions = None
    target = None
    if t1state == 0: #state zero
        i2c_bus = I2C(1, freq = 400000, timeout=1000000) #creating bus object
        i2c_address = 0x33 #assigning address per data sheet
        camera = Cam(i2c_bus) #creating camera object from MLX_Cam class
        camera._camera.refresh_rate = 10.0 #frequency of image gathering
        t1state = 1
        yield t1state
    else:
        raise ValueError(f"Invalid State in Task 1.  Current state is {t1state}")
    while True:
        if t1state == 1:
            while not image:
                image = camera.get_image_nonblocking()
                yield t1state
            im_arr = camera.get_array(image)
            t1state = 2
            yield t1state
        elif t1state == 2:
            yaw_angle, pitch_angle = cam2setpoint(im_arr)
            print(f"{yaw_angle}, {pitch_angle}")
            if not returning.get() == 1:
                yaw_motor_setpoint.put(16*yaw_angle+11.5)
                pitch_motor_setpoint.put(pitch_angle/2+2)
            image = None
            t1state = 1
            yield t1state
            
            
            
            

def yaw_motor_fun():
    """!
    This function controls the yaw motor as part of the second task
    It runs the motor from 0 degrees to 180 degrees using a proportional controller
    """
    t2state = 0
    yaw_err_list = []
    yaw_motor_threshold = 0.5 #amount of permissible error
    yaw_err_len = 25 #error is avg of 25 position readings
    if t2state == 0:
        # define the encoder conversion factor for the 
        co_fac1 = get_conversion_factor(1)
        # motor setup
        en_pin =  pyb.Pin(pyb.Pin.cpu.G12, mode = pyb.Pin.OPEN_DRAIN, pull = pyb.Pin.PULL_UP, value=1)
        in1pin = pyb.Pin(pyb.Pin.cpu.E9, pyb.Pin.OUT_PP)
        in2pin = pyb.Pin(pyb.Pin.cpu.E11, pyb.Pin.OUT_PP)
        timer = pyb.Timer(1, freq=20000) #setting frequency for motor 
        motor = MotorDriver(en_pin,in1pin,in2pin,timer) #create motor object
        motor.set_duty_cycle(0)
        # encoder setup
        # create the pin object to read encoder channel A
        pin1 = pyb.Pin(pyb.Pin.cpu.A6, pyb.Pin.IN)
        # create the pin object to read encoder channel B
        pin2 = pyb.Pin(pyb.Pin.cpu.B5, pyb.Pin.IN)
        # create the timer object.  For C6 and C7 use timer 8,
        # set the prescaler to zero and the period to the max 16bit number
        timer = pyb.Timer(3, prescaler = 0, period = 65535)
        # create the encoder object
        encoder = Encoder(pin1, pin2, timer, conversion_factor = co_fac1)
        encoder.set_pos(-180)
        # create controller object
        con = CLController(35, 0,0, 180)
        t2state = 1
        yield t2state
    else:
        raise ValueError(f"Invalid State in Task 2.  Current state is {t2state}")
    while True:
        #print("State 2")
        if t2state == 1:
            motor.set_duty_cycle(0)
            if run_motors.get() != 0:
                t2state = 2
            yield t2state
        elif t2state == 2:
            y_sp = yaw_motor_setpoint.get()
            con.set_setpoint(y_sp)
            #print(y_sp)
            encoder_angle = encoder.read()
            #der_angle}")
            yaw_err = y_sp-encoder_angle
            yaw_err_list.append(abs(yaw_err))
            if len(yaw_err_list)>=yaw_err_len:
                avg_yaw_err = sum(yaw_err_list)/yaw_err_len
                if abs(avg_yaw_err)<yaw_motor_threshold:
                    yaw_motor_done.put(1)
                yaw_err_list = []
            eff = con.run(encoder_angle)
            motor.set_duty_cycle(eff)
            yield t2state
        else:
            raise ValueError(f"Invalid State in Task 2.  Current state is {t2state}")
        motor.set_duty_cycle(0)


def pitch_motor_fun():
    """!
    This function controls the yaw motor as part of the second task
    It runs the motor from 0 degrees to 180 degrees using a proportional controller
    """
    t3state = 0
    pitch_err_list = []
    pitch_motor_threshold = 1
    if t3state == 0:
        # define the encoder conversion factor for the 
        co_fac2 = get_conversion_factor(4)
        # motor setup
        en_pin =  pyb.Pin(pyb.Pin.cpu.G14, mode = pyb.Pin.OPEN_DRAIN, pull = pyb.Pin.PULL_UP, value=1)
        in1pin = pyb.Pin(pyb.Pin.cpu.B6, pyb.Pin.OUT_PP)
        in2pin = pyb.Pin(pyb.Pin.cpu.B7, pyb.Pin.OUT_PP)
        timer = pyb.Timer(4, freq=20000) #setting frequency for motor 
        motor = MotorDriver(en_pin,in1pin,in2pin,timer) #create motor object
        motor.set_duty_cycle(0)
        # encoder setup
        # create the pin object to read encoder channel A
        pin1 = pyb.Pin(pyb.Pin.cpu.A15, pyb.Pin.IN)
        # create the pin object to read encoder channel B
        pin2 = pyb.Pin(pyb.Pin.cpu.B3, pyb.Pin.IN)
        # create the timer object.  For C6 and C7 use timer 8,
        # set the prescaler to zero and the period to the max 16bit number
        timer = pyb.Timer(2, prescaler = 0, period = 65535)
        # create the encoder object
        encoder = Encoder(pin1, pin2, timer, conversion_factor = co_fac2)
        encoder.set_pos(30)
        # create controller object
        con = CLController(25, 0.1, 2, 180)
        t3state = 1
        yield t3state
    else:
        raise ValueError(f"Invalid State in Task 3.  Current state is {t3state}")
    while True:
        #print("State 3")
        if t3state == 1:
            motor.set_duty_cycle(0)
            if run_motors.get() != 0:
                t3state = 2
            yield t3state
        elif t3state == 2:
            p_sp = pitch_motor_setpoint.get()
            con.set_setpoint(p_sp)
            encoder_angle = encoder.read()
            pitch_err = p_sp-encoder_angle
            pitch_err_list.append(pitch_err)
            if len(pitch_err_list)>=5:
                avg_pitch_err = sum(pitch_err_list)/5
                if avg_pitch_err<pitch_motor_threshold:
                    pitch_motor_done.put(1)
                pitch_err_list.pop(0)
            eff = con.run(encoder_angle)
            motor.set_duty_cycle(eff)
            yield t3state
        else:
            raise ValueError(f"Invalid State in Task 3.  Current state is {t3state}")
        motor.set_duty_cycle(0)

def trigger_fun():
    """!
    This function controls the yaw motor as part of the second task
    It runs the motor from 0 degrees to 180 degrees using a proportional controller
    """
    t4state = 0
    counter = 0
    if t4state == 0:
        # Defining a different pin with an alternate function for Timer 2
        servo_pin = pyb.Pin(pyb.Pin.cpu.E5, pyb.Pin.OUT_PP)
        # Create a Timer object for PWM
        timer = pyb.Timer(15, freq=50)  # Timer 2 with a frequency of 50 Hz
        servo = Servo(servo_pin,timer)
        t4state = 1
        yield t4state
    else:
        raise ValueError(f"Invalid State in Task 4.  Current state is {t4state}")
    while True:
        #print("State 4")
        if t4state == 1:
            if yaw_motor_done.get() > 0 and pitch_motor_done.get() > 0:
                print("pew")
                servo.set_servo(25)
                run_motors.put(0)
                t4state = 2
            yield t4state
        elif t4state == 2:
            if counter == 30:
                servo.set_servo(0)
                run_motors.put(1)
                returning.put(1)
                t4state = 3
            counter += 1
            yield t4state
        elif t4state == 3:
            yield t4state
        else:
            raise ValueError(f"Invalid State in Task 4.  Current state is {t4state}")

def timing_handler_fun():
    t5state = 0
    tstart = utime.ticks_ms()
    tend = tstart
    if t5state == 0:
        t5state = 1
        yield t5state
    while True:
        #print("State 5")
        if t5state == 1:
            curr_time = utime.ticks_ms()
            if curr_time >= tend:
                run_motors.put(1)
                tstart = utime.ticks_ms()
                tend = tstart + WAIT_TIME
                t5state = 2
            yield t5state
        elif t5state == 2:
            if utime.ticks_ms() > tend:
                print("here")
                yaw_motor_done.put(1)
                pitch_motor_done.put(1)
                t5state = 3
            if yaw_motor_done.get() == 1 and pitch_motor_done.get() == 1:
                tstart = utime.ticks_ms()
                tend = tstart + 1500
                t5state = 3
            yield t5state
        elif t5state == 3:
            if returning.get()==1:
                yaw_motor_setpoint.put(-180)
                pitch_motor_setpoint.put(30)
                if utime.ticks_ms() >= tend:
                    returning.put(0)
                    raise KeyboardInterrupt
            yield t5state
        else:
            raise ValueError(f"Invalid State in Task 5.  Current state is {t5state}")


# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":
    print("Testing ME405 stuff in cotask.py and task_share.py\r\n"
          "Press Ctrl-C to stop and show diagnostics.")
    #set angle values (degrees) for motor 1 and motor 2, labeled accordingly
    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for 
    # debugging and set trace to False when it's not needed
    run_motors = task_share.Share("B", name="Run the motors boolean")
    yaw_motor_done = task_share.Share("B", name="Yaw motor reached location boolean")
    pitch_motor_done = task_share.Share("B", name="Pitch motor reached location boolean")
    returning = task_share.Share("B", name="Returning to home boolean")
    yaw_motor_setpoint = task_share.Share("f", name="Yaw motor setpoint")
    pitch_motor_setpoint = task_share.Share("f", name="Pitch motor setpoint")
    run_motors.put(0)
    yaw_motor_done.put(0)
    pitch_motor_done.put(0)
    yaw_motor_setpoint.put(0)
    pitch_motor_setpoint.put(0)
    returning.put(0)
    task1 = cotask.Task(camera_handler_fun, name="Task 1: Camera Handler", priority=5,period=50,
                        profile=True, trace=False)
    task2 = cotask.Task(yaw_motor_fun, name="Task 2: Yaw Motor Handler", priority=9,period=15,
                        profile=True, trace=False)
    task3 = cotask.Task(pitch_motor_fun, name="Task 3: Pitch Motor Handler", priority=9,period=15,
                        profile=True, trace=False)
    task4 = cotask.Task(trigger_fun, name="Task 4: Trigger Handler", priority=10,period=15,
                        profile=True, trace=False)
    task5 = cotask.Task(timing_handler_fun, name="Task 5: Timing Handler", priority=10,period=15,
                        profile=True, trace=False)  
    # adding the tasks to task list, commenting out task3, used for acquiring data for plotting
    cotask.task_list.append(task1) #add tasks to scheduler list
    cotask.task_list.append(task2) #add tasks to scheduler list
    cotask.task_list.append(task3)
    cotask.task_list.append(task4)
    cotask.task_list.append(task5)
    
    
    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()
    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    motors = MotorContainer()
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            motors.disable_yaw()
            motors.disable_pitch()
            motors.disable_servo()
            break

    # Print a table of task data and a table of shared information data
    print('\n' + str (cotask.task_list))
    print(task_share.show_all())
    print(task1.get_trace())
    print('')
