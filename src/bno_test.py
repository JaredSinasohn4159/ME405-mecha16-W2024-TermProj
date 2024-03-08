import pyb
import utime
import struct
import adafruit-board-toolkit as cam

if __name__ == "__main__":
    raw_2_deg = 1/16  # conversion to go from raw euler angle values to degrees.
    
    i2c1 = pyb.I2C (1, pyb.I2C.CONTROLLER, baudrate = 100000)  # set up i2c controller
    sensor_locs = i2c1.scan()    # Check for devices on the bus and get locations
    print(f"List of sensor locations: {sensor_locs}")
    
    # get the imu sensor location
    sensor_location = sensor_locs[0]
    
    # Send 12 to sensor at sensor_location, OPR_MODE register to set the BNO055 in NDOF Mode
    i2c1.mem_write ('\x0C', sensor_location, 0x3D)
    
    # sleep for 3 seconds before beginnning to read
    utime.sleep_ms(3000)
    while True:
        # get the x,y,z euler angles from the BNO055 as byte arrays
        euler_byte = i2c1.mem_read (6, sensor_location, 0x1A)
        
        # unpack the byte arrays into readable data in little endian mode
        euler_x_raw, euler_y_raw, euler_z_raw = struct.unpack("<hhh",euler_byte)
        
        # convert euler angles from raw data to degrees 
        euler_x, euler_y, euler_z = raw_2_deg*float(euler_x_raw), raw_2_deg*float(euler_y_raw), raw_2_deg*float(euler_z_raw),
        
        # print out data
        print(f"raw bytes: {euler_byte}")
        print(f"unpacked raw data: {euler_x_raw}, {euler_y_raw}, {euler_z_raw}")
        print(f"Euler angles in degrees: {euler_x}, {euler_y}, {euler_z}")
        
        utime.sleep_ms(1000)
