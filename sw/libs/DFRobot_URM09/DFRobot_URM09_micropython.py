'''!
  @file DFRobot_URM09_micropython.py
  @brief MicroPython implementation of DFRobot_URM09 class
  @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license The MIT License (MIT)
  @version V1.0
  @date 2025
  @url https://github.com/DFRobot/DFRobot_URM09
'''
from machine import I2C
import time

class DFRobot_URM09:
    ## automatic mode
    _MEASURE_MODE_AUTOMATIC = 0x80
    ## passive mode
    _MEASURE_MODE_PASSIVE   = 0x00
    ## passive mode configure registers
    _CMD_DISTANCE_MEASURE   = 0x01
    ## Ranging from 500
    _MEASURE_RANG_500       = 0x20
    ## Ranging from 300 
    _MEASURE_RANG_300       = 0x10
    ## Ranging from 150
    _MEASURE_RANG_150       = 0x00
    
    SLAVEADDR_INDEX = 0
    PID_INDEX       = 1
    VERSION_INDEX   = 2
    DIST_H_INDEX    = 3
    DIST_L_INDEX    = 4
    TEMP_H_INDEX    = 5
    TEMP_L_INDEX    = 6
    CFG_INDEX       = 7
    CMD_INDEX       = 8
    REG_NUM         = 9

    def __init__(self, i2c, addr=0x11):
        self.i2c = i2c
        self.addr = addr

    def set_mode_range(self, range, mode):
        '''!
          @brief    set measurement mode and range
          @param range: Measured distance
          @n            _MEASURE_RANG_500             #Ranging from 500
          @n            _MEASURE_RANG_300             #Ranging from 300
          @n            _MEASURE_RANG_150             #Ranging from 150
          @param mode: Set mode
          @n            _MEASURE_MODE_AUTOMATIC       #automatic mode
          @n            _MEASURE_MODE_PASSIVE         #passive mode
        '''
        self.write_reg(self.CFG_INDEX, bytes([range | mode]))

    def measurement_start(self):
        '''!
          @brief    Passive mode ranging command
        '''
        self.write_reg(self.CMD_INDEX, bytes([self._CMD_DISTANCE_MEASURE]))

    def get_temperature(self):
        '''!
          @brief    get Temperature
          @return   Temperature in Celsius
        '''
        rslt = self.read_reg(self.TEMP_H_INDEX, 2)
        if rslt is None:
            return 25.0
        return float(((rslt[0] << 8) + rslt[1]) / 10)

    def get_distance(self):
        '''!
          @brief    get distance
          @return   Distance in cm
        '''
        rslt = self.read_reg(self.DIST_H_INDEX, 2)
        if rslt is None:
            return -1
        value = (rslt[0] << 8) + rslt[1]
        if value < 32768:
            return value
        else: 
            return value - 65536

    def modify_device_number(self, address):
        '''!
          @brief    Modify i2c device number
          @param address: i2c device number 1-127
        '''
        self.write_reg(self.SLAVEADDR_INDEX, bytes([address]))

    def get_device_number(self):
        '''!
          @brief    read i2c device number
          @return   i2c device number
        '''
        rslt = self.read_reg(self.SLAVEADDR_INDEX, 1)
        return rslt[0] if rslt else None

    def write_reg(self, reg, data):
        '''!
          @brief    Write data to register
        '''
        try:
            self.i2c.writeto_mem(self.addr, reg, data)
        except Exception as e:
            print(f"I2C write error: {e}")
            time.sleep(0.1)
    
    def read_reg(self, reg, length):
        '''!
          @brief    Read data from register
        '''
        try:
            return self.i2c.readfrom_mem(self.addr, reg, length)
        except Exception as e:
            print(f"I2C read error: {e}")
            return None
