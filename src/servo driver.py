from pyb import Pin, Timer
import utime
frequency = 50
class Servo:
    def __init__(self,servo_pin,timer):
        self.servo_pin = servo_pin
        self.timer = timer
        self.channel = self.timer.channel(1, Timer.PWM, pin=servo_pin)
        print("creating servo")

    def set_servo(self,angle):
        # 40KG servo DS3240MG
        # Positions of 0-180
        # PWM range of 500-2500 microseconds
        
        # Converting degrees into PWM percentage, accounting for range from specs
        duty_cycle = ((angle / 180) * (2500 - 500) + 500)/1000000
        duty_cycle = duty_cycle*frequency*100
        print(duty_cycle)
        self.channel.pulse_width_percent(duty_cycle)
        
            #print('1')
            #set_servo(0)
            #utime.sleep(1)
if __name__ =='__main__':
    print('peepee')
    # Defining a different pin with an alternate function for Timer 2
    #servo_pin = Pin(Pin.board.PA0, Pin.OUT_PP)
    servo_pin = machine.Pin(Pin.board.PA0, Pin.OUT_PP)
    # Create a Timer object for PWM
    timer = Timer(2, freq=frequency)  # Timer 2 with a frequency of 50 Hz
    servo = Servo(servo_pin,timer)
    while True:
        try:
            angle = input("Enter an Angle from 0-180: ")
            servo.set_servo(int(angle))
            utime.sleep(0.5)
        #utime.sleep(1)

        #set_servo(180)
        #utime.sleep(1)

        #set_servo(90)
        #utime.sleep(1)
            pass   

        except KeyboardInterrupt:
            raise KeyboardInterrupt
            pass  # Handling keyboard interrupt gracefully