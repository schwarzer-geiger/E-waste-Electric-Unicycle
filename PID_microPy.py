import machine
from machine import PWM, Pin, I2C
from ina219 import INA219
import time
from micropython_lsm6dsox import lsm6dsox
import math

# logging
powerLogging = 0
angleLogging = 1
if powerLogging == 1:
    powerFile=open("powerData.csv","w")
if angleLogging == 1:
    angleFile=open("angleData.csv","w")
    
# INA219 power measurement
if powerLogging == 1:
    SHUNT_OHMS = 0.1  # Check value of shunt used with your INA219
    ina = INA219(SHUNT_OHMS, I2C(0, scl=Pin(19), sda=Pin(18)))
    ina.configure()

# debugging
debug = 1

# constants
RAD2DEG = 57.2957795
CW = 1
CCW = 0

# Driver setup
pwmPin  = Pin(4, Pin.OUT)
dirPin = Pin(7, Pin.OUT)
pwmPin.value(0)
dirPin.value(1)
pwmFreq = 5000
dutyCycle = 0
pwmInstance = PWM(pwmPin, freq=pwmFreq, duty_u16=0)

# IMU setup
IMU = lsm6dsox.LSM6DSOX(I2C(0, scl=Pin(13), sda=Pin(12)))
filterGain = 0.1

IMU.gyro_data_rate = lsm6dsox.RATE_104_HZ
dtIMU = 10

# Controller setup

integratorLimit = 200
PIDError = 0
previousPIDError = 0
Kp = 1
Ki = 0
Kd = 0
I  = 0
previousPIDError = 0
maxxAngle = 15 # in deg
maxDeltaxAngle = 1000 * dtIMU # in deg/s

xrestAngle = 22
yrestAngle = 9
deactivated = 0

# PID to dutyCycle
maxPID = Kp*maxxAngle + Ki*maxxAngle*integratorLimit + Kd*maxDeltaxAngle

def dutyCycleCalc(percent):
    return round(percent / 100 * 65535)

def getRawData():
    rawAccelx, rawAccely, rawAccelz = IMU.acceleration
    rawGyrox, rawGyroy, rawGyroz = IMU.gyro
    
    return rawAccelx, rawAccely, rawAccelz, rawGyrox, rawGyroy, rawGyroz

def getAngle(rawAccelx, rawAccely, rawAccelz, rawGyrox, rawGyroy, rawGyroz, filterGain, dtIMU):

    # gyro angle via integration
    xdt = (90-abs(rawGyrox))/90 * dtIMU; # x rotation integration gives y and z angle displacement
    ydt = (90-abs(rawGyroy))/90 * dtIMU; # y rotation integration gives x and z angle displacement
    zdt = (90-abs(rawGyroz))/90 * dtIMU; # z rotation integration gives x and y angle displacement
    angleGyrox = rawGyrox + zdt - ydt;
    angleGyroy = rawGyroy + xdt - zdt;

    # accel angle via trig
    angleAccelx = math.atan(rawAccelx / math.sqrt(rawAccely**2 + rawAccelz**2 + rawAccelx**2)) * RAD2DEG;
    xAngle = filterGain * angleGyrox + (1 - filterGain) * angleAccelx;
    angleAccely = math.atan(rawAccely / math.sqrt(rawAccelx**2 + rawAccelz**2 + rawAccely**2)) * RAD2DEG;
    yAngle = filterGain * angleGyroy + (1 - filterGain) * angleAccely;

    if debug == 1:
        print("\nxAngle: ", end = '')
        print(xAngle, end = '')
        print(" yAngle: ", end = '')
        print(yAngle)
    
    return xAngle, yAngle
        
def computePID(xAngle, Kp, Ki, Kd, integratorLimit):
    PIDError = xAngle;
    P = Kp * PIDError
    global I
    I += Ki * PIDError
    
    if I > integratorLimit:
        I = integratorLimit # limit integration accumulation
    if I < -integratorLimit:
        I = -integratorLimit
    
    global previousPIDError
    D = Kd * (PIDError - previousPIDError);
    previousPIDError = PIDError;
    PID = P+I+D;
    
    if debug == 1:
        print("Error: ", end = '');
        print(PIDError, end = '');
        print(" P: ", end = '');
        print(P, end = '');
        print(" I: ", end = '');
        print(I, end = '');
        print(" D: ", end = '');
        print(D, end = '');
        print(" PID: ", end = '');
        print(PID);
    
    return PID
        
def computeDutyCycle(PID, maxPID):
    dutyCycle = abs(PID) / maxPID * 100;
    dutyCapped = 0;
    if dutyCycle > 100: 
        dutyCycle = 100;
        dutyCapped = 1;
    
    return dutyCycle, dutyCapped
        
def computeDir(PID):
    if PID >= 0:
        if debug == 1:
            print("CW ", end = '')
        return 1
    else:
        if debug == 1:
            print("CCW ", end = '')
        return 0
            
# main program

while True:
    #time.sleep(0.1)
    
    rawAccelx, rawAccely, rawAccelz, rawGyrox, rawGyroy, rawGyroz = getRawData()
    xAngle, yAngle = getAngle(rawAccelx, rawAccely, rawAccelz, rawGyrox, rawGyroy, rawGyroz, filterGain, dtIMU)
    
    # deactivate if vehicle unused
    while abs(xAngle) > xrestAngle or abs(yAngle) > yrestAngle:
        if deactivated == 0:
            deactivated = 1
            pwmInstance.duty_u16(0)
        rawAccelx, rawAccely, rawAccelz, rawGyrox, rawGyroy, rawGyroz = getRawData()
        xAngle, yAngle = getAngle(rawAccelx, rawAccely, rawAccelz, rawGyrox, rawGyroy, rawGyroz, filterGain, dtIMU)

        if debug == 1:
            print("Inactive", end = '')    
        if abs(xAngle) < xrestAngle and abs(yAngle) < yrestAngle:
            time.sleep(1)
            deactivated = 0
            break

    PID = computePID(xAngle, Kp, Ki, Kd, integratorLimit)
    dirPin.value(computeDir(PID))
    dutyCycle, dutyCapped = computeDutyCycle(PID, maxPID)
    if debug == 1:
        print("Duty Cycle: ", end = '')
        print(dutyCycle, end = '')
        if dutyCapped: 
            print(" (capped)", end = '')
        else:
            print("")
    if powerLogging == 1:
        powerFile.write(str(ina.voltage()) + "," + str(ina.current()) + "," + str(ina.power()) + str(dutyCycle) + "\n")
    if angleLogging == 1:
        angleFile.write(str(xAngle) + "\n")
    pwmInstance.duty_u16(dutyCycleCalc(dutyCycle))
