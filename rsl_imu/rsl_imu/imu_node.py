import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

import time
import board
import adafruit_bno055


i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

import math

fileName = "/home/pioneer3/pioneerTest/src/rsl_imu/rsl_imu/calData.txt"


def set_calibration(self:adafruit_bno055.BNO055_I2C, calData:list):
    """Set the sensor's calibration data using a list of 22 bytes that
    represent the sensor offsets and calibration data.  This data should be
    a value that was previously retrieved with get_calibration (and then
    perhaps persisted to disk or other location until needed again).
    """

    # Check that 22 bytes were passed in with calibration data.
    if calData is None or len(calData) != 22:
        raise ValueError('Expected a list of 22 bytes for calibration data.')
    
    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    self._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.CONFIG_MODE)  # Empirically necessary
    time.sleep(0.02)  # Datasheet table 3.6

    # Write 22 bytes of calibration data.
    registerAddr = 0x6A #Start with Magnometer radius MSB register

    for i in range(22):
            self._write_register(registerAddr,calData[i])
            #Update register Address:
            registerAddr-=0x1;

    # Go back to normal operation mode.
    time.sleep(0.01)  # Table 3.6
    self._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.NDOF_MODE)



#Set Calibration offsets/radii
f = open(fileName, 'r')
# calData =[int(line.rstrip()) for line in f]
calData = [2, 48, 3, 232, 0, 0, 255, 254, 255, 255, 1, 148, 254, 13, 254, 171, 255, 232, 0, 42, 255, 198];
time.sleep(0.1)
set_calibration(sensor,calData)


class readIMU(Node):
    def __init__(self):
        super().__init__('imu')
        self.publisherQ_ = self.create_publisher(Quaternion, 'imu/quaternion', 1)
        self.publisherE_ = self.create_publisher(Float32MultiArray, 'imu/eulerAngle', 3)
        self.publisherC_ = self.create_publisher(Int16MultiArray, 'imu/calibrationStatus', 4)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):

        #Publish Quaternion Data
        msgQ = Quaternion()
        msgQ.x, msgQ.y, msgQ.z, msgQ.w = sensor.quaternion[0], sensor.quaternion[1], sensor.quaternion[2], sensor.quaternion[3]
        self.publisherQ_.publish(msgQ)

        #Publish Euler Angle Data
        msgE = Float32MultiArray()
        msgE.data=sensor.euler
        self.publisherE_.publish(msgE)

        #Publish Calibration Status
        msgC = Int16MultiArray()
        msgC.data=sensor.calibration_status
        self.publisherC_.publish(msgC)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    get_imu_data = readIMU()

    rclpy.spin(get_imu_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_imu_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()