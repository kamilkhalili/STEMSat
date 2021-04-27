#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
import smbus
import math
import ctypes
import struct
import serial
import string
from datetime import datetime

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
lora=serial.Serial('/dev/ttyUSB1',9600)
ser=serial.Serial('/dev/ttyUSB0',baudrate=9600,timeout=0.5)
port=serial.Serial("/dev/ttyS0",baudrate=9600, timeout=1.0)

#Extension board Setup
LED=22
start=17
buzz=27


GPIO.setup(LED, GPIO.OUT)
GPIO.setup(start, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buzz, GPIO.OUT)
run=0
ledstate=False
buzzstate=False
firstwrite=False
relon=0.0
relat=0.0
sample=0

#-------------------------------
def parseGPS(dat):
    if dat[0:6] == "$GPGGA":
        s = dat.split(",")
        if s[7] == '0' or s[7]=='00':
            lat=0.0
            lon=0.0
            return lat,lon
        time = s[1][0:2] + ":" + s[1][2:4] + ":" + s[1][4:6]
        #print("-----------")
        lat = decode(s[2])
        lon = decode(s[4])
        return  lat,lon
    


def decode(coord):
    l = list(coord)
    for i in range(0,len(l)-1):
        if l[i] == "." :
                    break
    base = l[0:i-2]
    degi = l[i-2:i]
    degd = l[i+1:]
    #print(base,"   ",degi,"   ",degd)
    baseint = int("".join(base))
    degiint = int("".join(degi))
    degdint = float("".join(degd))
    degdint = degdint / (10**len(degd))
    degs = degiint + degdint
    full = float(baseint) + (degs/60)
    #print(full)
    
    return full

#LiPo Usage Setup
def readVoltage(bus):
    
    address = 0x36
    read = bus.read_word_data(address,2)
    swapped = struct.unpack("<H",struct.pack(">H",read))[0]
    voltage = swapped*1.25/1000/16
    return voltage

def readCapacity (bus):
    
    address = 0x36
    read = bus.read_word_data(address,4)
    swapped = struct.unpack("<H",struct.pack(">H",read))[0]
    capacity = swapped/256
    return capacity

bus=smbus.SMBus(1)

#i2c address
LPS22HB_I2C_ADDRESS   =  0x5C
#
LPS_ID                =  0xB1
#Register 
LPS_INT_CFG           =  0x0B        #Interrupt register
LPS_THS_P_L           =  0x0C        #Pressure threshold registers 
LPS_THS_P_H           =  0x0D        
LPS_WHO_AM_I          =  0x0F        #Who am I        
LPS_CTRL_REG1         =  0x10        #Control registers
LPS_CTRL_REG2         =  0x11
LPS_CTRL_REG3         =  0x12
LPS_FIFO_CTRL         =  0x14        #FIFO configuration register 
LPS_REF_P_XL          =  0x15        #Reference pressure registers
LPS_REF_P_L           =  0x16
LPS_REF_P_H           =  0x17
LPS_RPDS_L            =  0x18        #Pressure offset registers
LPS_RPDS_H            =  0x19        
LPS_RES_CONF          =  0x1A        #Resolution register
LPS_INT_SOURCE        =  0x25        #Interrupt register
LPS_FIFO_STATUS       =  0x26        #FIFO status register
LPS_STATUS            =  0x27        #Status register
LPS_PRESS_OUT_XL      =  0x28        #Pressure output registers
LPS_PRESS_OUT_L       =  0x29
LPS_PRESS_OUT_H       =  0x2A
LPS_TEMP_OUT_L        =  0x2B        #Temperature output registers
LPS_TEMP_OUT_H        =  0x2C
LPS_RES               =  0x33        #Filter reset register

Gyro  = [0,0,0]
Accel = [0,0,0]
Mag   = [0,0,0]
pitch = 0.0
roll  = 0.0
yaw   = 0.0
pu8data=[0,0,0,0,0,0,0,0]
U8tempX=[0,0,0,0,0,0,0,0,0]
U8tempY=[0,0,0,0,0,0,0,0,0]
U8tempZ=[0,0,0,0,0,0,0,0,0]
GyroOffset=[0,0,0]
Ki = 1.0
Kp = 4.50
q0 = 1.0
q1=q2=q3=0.0
angles=[0.0,0.0,0.0]
true                                 =0x01
false                                =0x00
# define ICM-20948 Device I2C address
I2C_ADD_ICM20948                     = 0x68
I2C_ADD_ICM20948_AK09916             = 0x0C
I2C_ADD_ICM20948_AK09916_READ        = 0x80
I2C_ADD_ICM20948_AK09916_WRITE       = 0x00
# define ICM-20948 Register
# user bank 0 register
REG_ADD_WIA                          = 0x00
REG_VAL_WIA                          = 0xEA
REG_ADD_USER_CTRL                    = 0x03
REG_VAL_BIT_DMP_EN                   = 0x80
REG_VAL_BIT_FIFO_EN                  = 0x40
REG_VAL_BIT_I2C_MST_EN               = 0x20
REG_VAL_BIT_I2C_IF_DIS               = 0x10
REG_VAL_BIT_DMP_RST                  = 0x08
REG_VAL_BIT_DIAMOND_DMP_RST          = 0x04
REG_ADD_PWR_MIGMT_1                  = 0x06
REG_VAL_ALL_RGE_RESET                = 0x80
REG_VAL_RUN_MODE                     = 0x01 # Non low-power mode
REG_ADD_LP_CONFIG                    = 0x05
REG_ADD_PWR_MGMT_1                   = 0x06
REG_ADD_PWR_MGMT_2                   = 0x07
REG_ADD_ACCEL_XOUT_H                 = 0x2D
REG_ADD_ACCEL_XOUT_L                 = 0x2E
REG_ADD_ACCEL_YOUT_H                 = 0x2F
REG_ADD_ACCEL_YOUT_L                 = 0x30
REG_ADD_ACCEL_ZOUT_H                 = 0x31
REG_ADD_ACCEL_ZOUT_L                 = 0x32
REG_ADD_GYRO_XOUT_H                  = 0x33
REG_ADD_GYRO_XOUT_L                  = 0x34
REG_ADD_GYRO_YOUT_H                  = 0x35
REG_ADD_GYRO_YOUT_L                  = 0x36
REG_ADD_GYRO_ZOUT_H                  = 0x37
REG_ADD_GYRO_ZOUT_L                  = 0x38
REG_ADD_EXT_SENS_DATA_00             = 0x3B
REG_ADD_REG_BANK_SEL                 = 0x7F
REG_VAL_REG_BANK_0                   = 0x00
REG_VAL_REG_BANK_1                   = 0x10
REG_VAL_REG_BANK_2                   = 0x20
REG_VAL_REG_BANK_3                   = 0x30

# user bank 1 register
# user bank 2 register
REG_ADD_GYRO_SMPLRT_DIV              = 0x00
REG_ADD_GYRO_CONFIG_1                = 0x01
REG_VAL_BIT_GYRO_DLPCFG_2            = 0x10  # bit[5:3]
REG_VAL_BIT_GYRO_DLPCFG_4            = 0x20  # bit[5:3]
REG_VAL_BIT_GYRO_DLPCFG_6            = 0x30  # bit[5:3]
REG_VAL_BIT_GYRO_FS_250DPS           = 0x00  # bit[2:1]
REG_VAL_BIT_GYRO_FS_500DPS           = 0x02  # bit[2:1]
REG_VAL_BIT_GYRO_FS_1000DPS          = 0x04  # bit[2:1]
REG_VAL_BIT_GYRO_FS_2000DPS          = 0x06  # bit[2:1]
REG_VAL_BIT_GYRO_DLPF                = 0x01  # bit[0]
REG_ADD_ACCEL_SMPLRT_DIV_2           = 0x11
REG_ADD_ACCEL_CONFIG                 = 0x14
REG_VAL_BIT_ACCEL_DLPCFG_2           = 0x10  # bit[5:3]
REG_VAL_BIT_ACCEL_DLPCFG_4           = 0x20  # bit[5:3]
REG_VAL_BIT_ACCEL_DLPCFG_6           = 0x30  # bit[5:3]
REG_VAL_BIT_ACCEL_FS_2g              = 0x00  # bit[2:1]
REG_VAL_BIT_ACCEL_FS_4g              = 0x02  # bit[2:1]
REG_VAL_BIT_ACCEL_FS_8g              = 0x04  # bit[2:1]
REG_VAL_BIT_ACCEL_FS_16g             = 0x06  # bit[2:1]
REG_VAL_BIT_ACCEL_DLPF               = 0x01  # bit[0]

# user bank 3 register
REG_ADD_I2C_SLV0_ADDR                = 0x03
REG_ADD_I2C_SLV0_REG                 = 0x04
REG_ADD_I2C_SLV0_CTRL                = 0x05
REG_VAL_BIT_SLV0_EN                  = 0x80
REG_VAL_BIT_MASK_LEN                 = 0x07
REG_ADD_I2C_SLV0_DO                  = 0x06
REG_ADD_I2C_SLV1_ADDR                = 0x07
REG_ADD_I2C_SLV1_REG                 = 0x08
REG_ADD_I2C_SLV1_CTRL                = 0x09
REG_ADD_I2C_SLV1_DO                  = 0x0A

# define ICM-20948 Register  end

# define ICM-20948 MAG Register
REG_ADD_MAG_WIA1                     = 0x00
REG_VAL_MAG_WIA1                     = 0x48
REG_ADD_MAG_WIA2                     = 0x01
REG_VAL_MAG_WIA2                     = 0x09
REG_ADD_MAG_ST2                      = 0x10
REG_ADD_MAG_DATA                     = 0x11
REG_ADD_MAG_CNTL2                    = 0x31
REG_VAL_MAG_MODE_PD                  = 0x00
REG_VAL_MAG_MODE_SM                  = 0x01
REG_VAL_MAG_MODE_10HZ                = 0x02
REG_VAL_MAG_MODE_20HZ                = 0x04
REG_VAL_MAG_MODE_50HZ                = 0x05
REG_VAL_MAG_MODE_100HZ               = 0x08
REG_VAL_MAG_MODE_ST                  = 0x10
# define ICM-20948 MAG Register  end

MAG_DATA_LEN                         =6
class SHTC3:
    def __init__(self):
        self.dll = ctypes.CDLL("./SHTC3.so")
        init = self.dll.init
        init.restype = ctypes.c_int
        init.argtypes = [ctypes.c_void_p]
        init(None)

    def SHTC3_Read_Temperature(self):
        temperature = self.dll.SHTC3_Read_TH
        temperature.restype = ctypes.c_float
        temperature.argtypes = [ctypes.c_void_p]
        return temperature(None)

    def SHTC3_Read_Humidity(self):
        humidity = self.dll.SHTC3_Read_RH
        humidity.restype = ctypes.c_float
        humidity.argtypes = [ctypes.c_void_p]
        return humidity(None)
    
class LPS22HB(object):
    def __init__(self,address=LPS22HB_I2C_ADDRESS):
        self._address = address
        self._bus = smbus.SMBus(1)
        self.LPS22HB_RESET()                         #Wait for reset to complete
        self._write_byte(LPS_CTRL_REG1 ,0x02)        #Low-pass filter disabled , output registers not updated until MSB and LSB have been read , Enable Block Data Update , Set Output Data Rate to 0 
    def LPS22HB_RESET(self):
        Buf=self._read_u16(LPS_CTRL_REG2)
        Buf|=0x04                                         
        self._write_byte(LPS_CTRL_REG2,Buf)               #SWRESET Set 1
        while Buf:
            Buf=self._read_u16(LPS_CTRL_REG2)
            Buf&=0x04
    def LPS22HB_START_ONESHOT(self):
        Buf=self._read_u16(LPS_CTRL_REG2)
        Buf|=0x01                                         #ONE_SHOT Set 1
        self._write_byte(LPS_CTRL_REG2,Buf)
    def _read_byte(self,cmd):
        return self._bus.read_byte_data(self._address,cmd)
    def _read_u16(self,cmd):
        LSB = self._bus.read_byte_data(self._address,cmd)
        MSB = self._bus.read_byte_data(self._address,cmd+1)
        return (MSB << 8) + LSB
    def _write_byte(self,cmd,val):
        self._bus.write_byte_data(self._address,cmd,val)
    
class ICM20948(object):
  def __init__(self,address=I2C_ADD_ICM20948):
    self._address = address
    self._bus = smbus.SMBus(1)
    bRet=self.icm20948Check()             #Initialization of the device multiple times after power on will result in a return error
    # while true != bRet:
    #   print("ICM-20948 Error\n" )
    #   time.sleep(0.5)
    # print("ICM-20948 OK\n" )
    time.sleep(0.5)                       #We can skip this detection by delaying it by 500 milliseconds
    # user bank 0 register 
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0)
    self._write_byte( REG_ADD_PWR_MIGMT_1 , REG_VAL_ALL_RGE_RESET)
    time.sleep(0.1)
    self._write_byte( REG_ADD_PWR_MIGMT_1 , REG_VAL_RUN_MODE)  
    #user bank 2 register
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_2)
    self._write_byte( REG_ADD_GYRO_SMPLRT_DIV , 0x07)
    self._write_byte( REG_ADD_GYRO_CONFIG_1 , REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF)
    self._write_byte( REG_ADD_ACCEL_SMPLRT_DIV_2 ,  0x07)
    self._write_byte( REG_ADD_ACCEL_CONFIG , REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF)
    #user bank 0 register
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0) 
    time.sleep(0.1)
    self.icm20948GyroOffset()
    self.icm20948MagCheck()
    self.icm20948WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ)
  def icm20948_Gyro_Accel_Read(self):
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0)
    data =self._read_block(REG_ADD_ACCEL_XOUT_H, 12)
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_2)
    Accel[0] = (data[0]<<8)|data[1]
    Accel[1] = (data[2]<<8)|data[3]
    Accel[2] = (data[4]<<8)|data[5]
    Gyro[0]  = ((data[6]<<8)|data[7]) - GyroOffset[0]
    Gyro[1]  = ((data[8]<<8)|data[9]) - GyroOffset[1]
    Gyro[2]  = ((data[10]<<8)|data[11]) - GyroOffset[2]
    if Accel[0]>=32767:             #Solve the problem that Python shift will not overflow
      Accel[0]=Accel[0]-65535
    elif Accel[0]<=-32767:
      Accel[0]=Accel[0]+65535
    if Accel[1]>=32767:
      Accel[1]=Accel[1]-65535
    elif Accel[1]<=-32767:
      Accel[1]=Accel[1]+65535
    if Accel[2]>=32767:
      Accel[2]=Accel[2]-65535
    elif Accel[2]<=-32767:
      Accel[2]=Accel[2]+65535
    if Gyro[0]>=32767:
      Gyro[0]=Gyro[0]-65535
    elif Gyro[0]<=-32767:
      Gyro[0]=Gyro[0]+65535
    if Gyro[1]>=32767:
      Gyro[1]=Gyro[1]-65535
    elif Gyro[1]<=-32767:
      Gyro[1]=Gyro[1]+65535
    if Gyro[2]>=32767:
      Gyro[2]=Gyro[2]-65535
    elif Gyro[2]<=-32767:
      Gyro[2]=Gyro[2]+65535
  def icm20948MagRead(self):
    counter=20
    while(counter>0):
      time.sleep(0.01)
      self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ , REG_ADD_MAG_ST2, 1)
      if ((pu8data[0] & 0x01)!= 0):
        break
      counter-=1
    if counter!=0:
      for i in range(0,8):
        self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ , REG_ADD_MAG_DATA , MAG_DATA_LEN)
        U8tempX[i] = (pu8data[1]<<8)|pu8data[0]
        U8tempY[i] = (pu8data[3]<<8)|pu8data[2]
        U8tempZ[i] = (pu8data[5]<<8)|pu8data[4]
      Mag[0]=(U8tempX[0]+U8tempX[1]+U8tempX[2]+U8tempX[3]+U8tempX[4]+U8tempX[5]+U8tempX[6]+U8tempX[7])/8
      Mag[1]=-(U8tempY[0]+U8tempY[1]+U8tempY[2]+U8tempY[3]+U8tempY[4]+U8tempY[5]+U8tempY[6]+U8tempY[7])/8
      Mag[2]=-(U8tempZ[0]+U8tempZ[1]+U8tempZ[2]+U8tempZ[3]+U8tempZ[4]+U8tempZ[5]+U8tempZ[6]+U8tempZ[7])/8
    if Mag[0]>=32767:            #Solve the problem that Python shift will not overflow
      Mag[0]=Mag[0]-65535
    elif Mag[0]<=-32767:
      Mag[0]=Mag[0]+65535
    if Mag[1]>=32767:
      Mag[1]=Mag[1]-65535
    elif Mag[1]<=-32767:
      Mag[1]=Mag[1]+65535
    if Mag[2]>=32767:
      Mag[2]=Mag[2]-65535
    elif Mag[2]<=-32767:
      Mag[2]=Mag[2]+65535
  def icm20948ReadSecondary(self,u8I2CAddr,u8RegAddr,u8Len):
    u8Temp=0
    self._write_byte( REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3) #swtich bank3
    self._write_byte( REG_ADD_I2C_SLV0_ADDR, u8I2CAddr)
    self._write_byte( REG_ADD_I2C_SLV0_REG,  u8RegAddr)
    self._write_byte( REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0
    
    u8Temp = self._read_byte(REG_ADD_USER_CTRL)
    u8Temp |= REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)
    time.sleep(0.01)
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)
    
    for i in range(0,u8Len):
      pu8data[i]= self._read_byte( REG_ADD_EXT_SENS_DATA_00+i)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3) #swtich bank3
    
    u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN))
    self._write_byte( REG_ADD_I2C_SLV0_CTRL,  u8Temp)
    
    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0
  def icm20948WriteSecondary(self,u8I2CAddr,u8RegAddr,u8data):
    u8Temp=0
    self._write_byte( REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3) #swtich bank3
    self._write_byte( REG_ADD_I2C_SLV1_ADDR, u8I2CAddr)
    self._write_byte( REG_ADD_I2C_SLV1_REG,  u8RegAddr)
    self._write_byte( REG_ADD_I2C_SLV1_DO,   u8data)
    self._write_byte( REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0

    u8Temp = self._read_byte(REG_ADD_USER_CTRL)
    u8Temp |= REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)
    time.sleep(0.01)
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3) #swtich bank3

    u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN))
    self._write_byte( REG_ADD_I2C_SLV0_CTRL,  u8Temp)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0
  def icm20948GyroOffset(self):
    s32TempGx = 0
    s32TempGy = 0
    s32TempGz = 0
    for i in range(0,32):
      self.icm20948_Gyro_Accel_Read()
      s32TempGx += Gyro[0]
      s32TempGy += Gyro[1]
      s32TempGz += Gyro[2]
      time.sleep(0.01)
    GyroOffset[0] = s32TempGx >> 5
    GyroOffset[1] = s32TempGy >> 5
    GyroOffset[2] = s32TempGz >> 5
  def _read_byte(self,cmd):
    return self._bus.read_byte_data(self._address,cmd)
  def _read_block(self, reg, length=1):
    return self._bus.read_i2c_block_data(self._address, reg, length)
  def _read_u16(self,cmd):
    LSB = self._bus.read_byte_data(self._address,cmd)
    MSB = self._bus.read_byte_data(self._address,cmd+1)
    return (MSB << 8) + LSB
  def _write_byte(self,cmd,val):
    self._bus.write_byte_data(self._address,cmd,val)
    time.sleep(0.0001)
  def imuAHRSupdate(self,gx, gy,gz,ax,ay,az,mx,my,mz):    
    norm=0.0
    hx = hy = hz = bx = bz = 0.0
    vx = vy = vz = wx = wy = wz = 0.0
    exInt = eyInt = ezInt = 0.0
    ex=ey=ez=0.0 
    halfT = 0.024
    global q0
    global q1
    global q2
    global q3
    q0q0 = q0 * q0
    q0q1 = q0 * q1
    q0q2 = q0 * q2
    q0q3 = q0 * q3
    q1q1 = q1 * q1
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q2q2 = q2 * q2   
    q2q3 = q2 * q3
    q3q3 = q3 * q3          

    norm = float(1/math.sqrt(ax * ax + ay * ay + az * az))     
    ax = ax * norm
    ay = ay * norm
    az = az * norm

    norm = float(1/math.sqrt(mx * mx + my * my + mz * mz))      
    mx = mx * norm
    my = my * norm
    mz = mz * norm

    # compute reference direction of flux
    hx = 2 * mx * (0.5 - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2)
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5 - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1)
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2)         
    bx = math.sqrt((hx * hx) + (hy * hy))
    bz = hz     

    # estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2)
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3)
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2)  

    # error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy)
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz)
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx)

    if (ex != 0.0 and ey != 0.0 and ez != 0.0):
      exInt = exInt + ex * Ki * halfT
      eyInt = eyInt + ey * Ki * halfT  
      ezInt = ezInt + ez * Ki * halfT

      gx = gx + Kp * ex + exInt
      gy = gy + Kp * ey + eyInt
      gz = gz + Kp * ez + ezInt

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT  

    norm = float(1/math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3))
    q0 = q0 * norm
    q1 = q1 * norm
    q2 = q2 * norm
    q3 = q3 * norm
  def icm20948Check(self):
    bRet=false
    if REG_VAL_WIA == self._read_byte(REG_ADD_WIA):
      bRet = true
    return bRet
  def icm20948MagCheck(self):
    self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,REG_ADD_MAG_WIA1, 2)
    if (pu8data[0] == REG_VAL_MAG_WIA1) and ( pu8data[1] == REG_VAL_MAG_WIA2) :
        bRet = true
        return bRet
  def icm20948CalAvgValue(self):
    MotionVal[0]=Gyro[0]/32.8
    MotionVal[1]=Gyro[1]/32.8
    MotionVal[2]=Gyro[2]/32.8
    MotionVal[3]=Accel[0]
    MotionVal[4]=Accel[1]
    MotionVal[5]=Accel[2]
    MotionVal[6]=Mag[0]
    MotionVal[7]=Mag[1]
    MotionVal[8]=Mag[2]
    
if __name__ == '__main__':
#   print("\nSense HAT Test Program ...\n")
  MotionVal=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
  icm20948=ICM20948()
  PRESS_DATA = 0.0
  TEMP_DATA = 0.0
  u8Buf=[0,0,0]
#   print("\nPressure Sensor Test Program ...\n")
  lps22hb=LPS22HB()
  while (run==0):
    startval=GPIO.input(start)
    
    if (startval == 1):
        GPIO.output(LED,True)
        
        
    while (startval==0):
        ledstate= not ledstate
        buzzstate= not buzzstate
        GPIO.output(LED,ledstate)
        GPIO.output(buzz,buzzstate)
        startval=GPIO.input(start)
        Vol= readVoltage(bus)
        Cap= readCapacity(bus)
        if (firstwrite==False):
            time.sleep(2)
        try:
            dat = ser.readline().decode()
            mylat,mylon = parseGPS(dat)
            relat=mylat
            relon=mylon
            #print(mylat , "   " , mylon)
        except:
            mylat= relat
            mylon= relon
        
#         print ("Voltage:5.2fV",Vol)
#         print ("Battery:%5i%%",Cap)
#         if readCapacity(bus)==100:
#             print("Battery FULL")
#         if readCapacity(bus)<20:
#             print("Battery LOW")
        time.sleep(0.01)
        lps22hb.LPS22HB_START_ONESHOT()
        if (lps22hb._read_byte(LPS_STATUS)&0x01)==0x01:  # a new pressure data is generated
            u8Buf[0]=lps22hb._read_byte(LPS_PRESS_OUT_XL)
            u8Buf[1]=lps22hb._read_byte(LPS_PRESS_OUT_L)
            u8Buf[2]=lps22hb._read_byte(LPS_PRESS_OUT_H)
            PRESS_DATA=((u8Buf[2]<<16)+(u8Buf[1]<<8)+u8Buf[0])/4096.0
        if (lps22hb._read_byte(LPS_STATUS)&0x02)==0x02:   # a new pressure data is generated
            u8Buf[0]=lps22hb._read_byte(LPS_TEMP_OUT_L)
            u8Buf[1]=lps22hb._read_byte(LPS_TEMP_OUT_H)
            TEMP_DATA=((u8Buf[1]<<8)+u8Buf[0])/100.0
        shtc3 = SHTC3()
        icm20948.icm20948_Gyro_Accel_Read()
        icm20948.icm20948MagRead()
        icm20948.icm20948CalAvgValue()
        icm20948.imuAHRSupdate(MotionVal[0] * 0.0175, MotionVal[1] * 0.0175,MotionVal[2] * 0.0175,
                    MotionVal[3],MotionVal[4],MotionVal[5], 
                    MotionVal[6], MotionVal[7], MotionVal[8])
        pitch = math.asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3
        roll  = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3
        yaw   = math.atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3
        currentdatetime=time.strftime("%c")

            
        rcv=None
        gotData=False
        if port.inWaiting()>=32:
            rcv=port.read(port.inWaiting())
            #print(repr(rcv))
            
        if rcv is not None:
            if(len(rcv)>31):
                
                try:
                    bytestream=rcv.split(b'BM')[1][0:31]
                except IndexError:
                    print("IndexError")
                    break
                if len(bytestream)==30:
                    if bytestream[0:2]==b'\x00\x1c':
                        checksum=143
                        validated=0
                        for i in range(0,28):
                            checksum+=int(bytestream[i])
                        validated=int(bytestream[28])*256+int(bytestream[29])
                        if checksum==validated:
                            pm25=int(bytestream[4])*256+int(bytestream[5])
                            pm10=int(bytestream[6])*256+int(bytestream[7])
                            currentdatetime=time.strftime("%c")
                            #print(currentdatetime,' pm2.5um:',pm25,' pm10um:',pm10,'\n')
                            gotData=True
                        else:
                            print('checksum failed')
                    else:
                        print('invalid packet')
                else:
                    print('incomplete packet')
            else:
                print('incomplete packet')

            #gps="Latitude=" +str(lat) + " and Longitude=" +str(lng)

#         print("\r\n /-------------------------------------------------------------/ \r\n")
#         print('\rTemperature = %6.2f°C , Humidity = %6.2f%%\r\n' % (shtc3.SHTC3_Read_Temperature(), shtc3.SHTC3_Read_Humidity()))
#         print('\r\nRoll = %.2f , Pitch = %.2f , Yaw = %.2f\r\n'%(roll,pitch,yaw))
#         print('\r\nAcceleration:  X = %d , Y = %d , Z = %d\r\n'%(Accel[0],Accel[1],Accel[2]))  
#         print('\r\nGyroscope:     X = %d , Y = %d , Z = %d\r\n\n'%(Gyro[0],Gyro[1],Gyro[2]))
#         #print('\r\nMagnetic:      X = %d , Y = %d , Z = %d'%((Mag[0]),Mag[1],Mag[2]))
#         print('Pressure = %6.2f hPa \r\n'%(PRESS_DATA))
        if firstwrite==False:
            filename=currentdatetime
            f=open('/home/pi/Desktop/' + str(filename) +'.csv','a+')
            f.write('Origin, Sample, Unix Time, Voltage, Capacity(%), Temp (C), Humidity (%),\
Roll,Pitch, Yaw, Accel0, Accel1, Accel2,Gyro0, Gyro1, Gyro2, Pressure(KPa), Altitude, Latitude,\
Longitude,pm2.5,pm10\n')
            firstwrite=True
            initialpress=(PRESS_DATA/10)
            
        Pressure=(PRESS_DATA/10)
        Altitude=(((((initialpress/Pressure)**(1/5.257))-1)*(shtc3.SHTC3_Read_Temperature()))/0.0065)
        data= (str(83764549) + ',' + str(sample) + ',' + str(time.time())  + ',' + str(Vol) + ',' + str(Cap) + ','
            + str(shtc3.SHTC3_Read_Temperature()) + ',' + str(shtc3.SHTC3_Read_Humidity())+ ','\
            + str(roll) + ',' + str(pitch) + ',' + str(yaw) + ',' + str(Accel[0])+ ',' + str(Accel[1])+','\
            + str(Accel[2])+',' + str(Gyro[0])+ ',' + str(Gyro[1])+ ',' + str(Gyro[2])+ ',' + str(Pressure)\
            + ',' + str(Altitude) + ',' + str(mylat) + ',' + str(mylon) + ',' + str(pm25) +',' + str(pm10) +'\n')
        print(data)
        sample=sample+1
        endata=data.encode("utf-8")   
        f.write(data)
        lora.write(endata)
        time.sleep(0.3)
        run=1
        x=0
        
if (run==1):
    f.close()
    GPIO.output(LED,True)        
    GPIO.output(buzz,True)
    time.sleep(3)
    GPIO.output(buzz,False)
    GPIO.output(LED,False)
