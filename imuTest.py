# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055


i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_bno055.BNO055_I2C(i2c)

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

fileName = 'calData.txt'

def printOffsets(sensor: adafruit_bno055.BNO055_I2C):
    print(sensor.offsets_accelerometer)
    print(sensor.offsets_gyroscope)
    print(sensor.offsets_magnetometer)
    print(sensor.radius_accelerometer)
    print(sensor.radius_magnetometer)

def saveData(calData):
    f = open(fileName, 'w')
    calData = [str(elem)+'\n' for elem in calData] #Convert to string for writing data
    f.writelines(calData)
    f.close()

def get_calibration(self:adafruit_bno055.BNO055_I2C):
        """Return the sensor's calibration data and return it as an array of
        22 bytes. Can be saved and then reloaded with the set_calibration function
        to quickly calibrate from a previously calculated set of calibration data.
        """
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.CONFIG_MODE)  # Empirically necessary
        time.sleep(0.02)  # Datasheet table 3.6

        # Read the 22 bytes of calibration data and put it in a list
        calData = [];
        registerAddr = 0x6A #Start with Magnometer radius MSB register

        for i in range(22):
             calData.append(self._read_register(registerAddr))
             #Update register Address:
             registerAddr-=0x1;

        # Go back to normal operation mode.
        time.sleep(0.01)  # Table 3.6
        self._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.NDOF_MODE)
        return calData

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



runCalibration = True #run calibration until good calibration and then store offsets/radii and exit
printData = True # Print Euler and Calibration Status data
setCalibration = False #Set calibration offsets/radii in file  

if runCalibration:
     while True:
        calStatus = sensor.calibration_status
        print("Calibrated:", sensor.calibration_status)
        if calStatus== (3,3,3,3) or calStatus==(3,3,3,2):
            print("OK")
            goodCalData = get_calibration(sensor);
            print(goodCalData)
            saveData(goodCalData)
            quit()
        time.sleep(0.5)

if setCalibration:
    f = open(fileName, 'r')
    caliData =[int(line.rstrip()) for line in f]
    set_calibration(sensor,caliData)


if printData:     
    while True:
        # print("Temperature: {} degrees C".format(sensor.temperature))
        """
        print(
            "Temperature: {} degrees C".format(temperature())
        )  # Uncomment if using a Raspberry Pi
        """
        #print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        #print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        #print("Gyroscope (rad/sec): {}".format(sensor.gyro))    
        # print("Quaternion: {}".format(sensor.euler))
        
        #print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
        #print("Gravity (m/s^2): {}".format(sensor.gravity))
        # print(type(sensor.quaternion), type(sensor.quaternion[1]))
        print("Euler angle: {}".format(sensor.euler))
        print("Calibrated:", sensor.calibration_status)

        time.sleep(0.5)




"""

https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration

https://github.com/adafruit/Adafruit_Python_BNO055/blob/master/Adafruit_BNO055/BNO055.py

"""

# Cal Data Pioneer 3:

# [2, 48, 3, 232, 0, 0, 255, 254, 255, 255, 1, 148, 254, 13, 254, 171, 255, 232, 0, 42, 255, 198]