import pwmio
import board
import time
import struct
import math
import usb_cdc
ser = usb_cdc.data

i2c = board.I2C() # inter-integrated circuit protocol

from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
imu = LSM6DS33(i2c)

# initialization
t0 = time.monotonic(); t1 = 0
dt = 1e-2
tick2rad = 2*math.pi/1400

def doLogging ():
    global t, alpha, h, u
    s = "(%0.3f, %0.3f, %0.3f, %0.3f)" % (t, alpha, h, u)
    ser.write(s+'\n')
    
def initialize():
    global g_bias, posL0, posR0, posL_, posR_, rw, alpha, velL, velR, dalpha
    rw = 0.021
    a = 0.75
    g_bias_ = 0
    velL = 0
    velR = 0
    
    
    for _ in range(100):
        g_bias = a * g_bias_ + (1 - a) * imu.gyro[1] # dps
        g_bias_ = g_bias
        time.sleep(1e-3)
    print('g_bias = %0.3f' % g_bias)

    acc = imu.acceleration
    gyr = imu.gyro
    alpha_a = math.atan2(-acc[2], acc[0])
    dalpha_g = - ( gyr[1] - -0.067 )
    dalpha = dalpha_g
    
    a = .99
    alpha = 0

    # angle measurement initialization
    i2c = board.I2C()
    while not i2c.try_lock():
        pass
    data = bytearray(8)
    i2c.readfrom_into(0x8, data)
    posL0,posR0 = struct.unpack('ll',data)
    posL_ = posL0
    posR_ = posR0
    i2c.unlock()
    
def update():
    global u, posL0, posR0, posL_, posR_, velL, velR, alpha, dalpha, rw, h
    acc = imu.acceleration
    gyr = imu.gyro
    alpha_a = math.atan2(-acc[2], acc[0])
    dalpha_g = - ( gyr[1] - -0.067 )
    
    a = .99
    alpha = a * (alpha + dt * dalpha_g) + (1 - a) * alpha_a
    dalpha = dalpha_g
# velocity measurement
    i2c = board.I2C()
    while not i2c.try_lock():
        pass
    data = bytearray(8)
    i2c.readfrom_into(0x8, data)
    posL,posR = struct.unpack('ll',data)
    i2c.unlock()
    
    posL = - (posL - posL0) * tick2rad
    posR = - (posR - posR0) * tick2rad
    a = .25
    velL = a * velL + (1 - a) * (posL - posL_) / dt
    velR = a * velR + (1 - a) * (posR - posR_) / dt
    posL_ = posL; posR_ = posR

    # combine angle measurements
    beta = .5 * (posL + posR)
    dbeta = .5 * (velL + velR)

    # horizontal position, velocity
    h = -1 * rw * (beta + alpha)
    dh = -1 * rw * (dbeta + dalpha)

    x = [h, dh, alpha, dalpha]
    K = [ -3.16227766, -13.62534159,   8.36052798,   0.48831056]
    
    u = -.2 * (K[0] * x[0] + K[1] * x[1] + K[2] * x[2] + K[3] * x[3])
    #print(u,x)

import pwmio
ccL = pwmio.PWMOut(board.D10, frequency=5000, duty_cycle=0)
cwL = pwmio.PWMOut(board.D12, frequency=5000, duty_cycle=0)
ccR = pwmio.PWMOut(board.D5, frequency=5000, duty_cycle=0)
cwR = pwmio.PWMOut(board.D6, frequency=5000, duty_cycle=0)

def doMotors():
    global alpha,u,cwL,ccL,cwR,ccR,u
    # disable control if robot has toppled
    if abs(alpha) > 40 * math.pi / 180:
        u = 0
    # saturate control input
    if u > 1:
        u = 1
    if u < -1:
        u = -1
        
    dutyL = u; dutyR = u
    if dutyL >= 0:
        ccL.duty_cycle = int(65535*(1-dutyL))
        cwL.duty_cycle = 65535
    else:
        ccL.duty_cycle = 65535
        cwL.duty_cycle = int(65535*(1+dutyL))
    if dutyR >= 0:
        ccR.duty_cycle = int(65535*(1-dutyR))
        cwR.duty_cycle = 65535
    else:
        ccR.duty_cycle = 65535
        cwR.duty_cycle = int(65535*(1+dutyR))
        

initialize()

while True:
    t = time.monotonic() - t0
    if t >= t1 + dt:
        t1 = t
        update()
        doMotors()
        doLogging()
        