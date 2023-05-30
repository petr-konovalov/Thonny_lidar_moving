from pyrplidar import PyRPlidar as RPLidar
import os
from math import cos, sin, pi, floor, atan2
import pygame
import RPi.GPIO as GPIO
from time import sleep
from threading import Thread
from numpy import median

lidarShowing = False
lidarIsActive = False

leftDegree = 90
forwardDegree = 180
rightDegree = 270
leftMinDistance = 6000
rightMinDistance = 6000
maxNearestDistance = 6000
maxDistance = 4000
nearestDistance = maxNearestDistance
nearestAngle = 0
minYplus = 0
minYminus = 0
degRange = 30
directionEstimate = 0
agentX = 0
agentY = 0
agentXInt = 0

if lidarShowing:
    os.putenv('SDL_FBDEV', '/dev/fb0')
    pygame.init()
    lcd = pygame.display.set_mode((320, 240))
    pygame.mouse.set_visible(False)
    lcd.fill((0, 0, 0))
    pygame.display.update()
    max_distance = 0

# Pins for Motor Driver Inputs
rightMotorId = 1
leftMotorId = 0

MPins = [{'FPin': 21, 'BPin': 20, 'SPin': 16},
         {'FPin': 19, 'BPin': 26, 'SPin': 13}] #Forward pin, backward pin, speed pin
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)        # GPIO Numbering
for pins in MPins:
    for pin in pins.values():
        GPIO.setup(pin, GPIO.OUT) # All motor pins as Outputs
sPins = [GPIO.PWM(pins['SPin'], 1000) for pins in MPins]        
for p in sPins:
    p.start(0) #Arguments is float and must be between 0 and 100
    
lidar = RPLidar()
# lidar.connect('/dev/ttyUSB0', timeout = 3, baudrate = 115200)
# lidar.set_motor_pwm(500)
# sleep(2)
# info = lidar.get_info()
# print("info: ", info)
# print("health: ", lidar.get_health())
# print(lidar.get_scan_modes())
# sleep(2)
# scan_generator = lidar.start_scan_express(4)
    
def sign(arg):
    if arg == 0:
        return 0
    elif arg < 0:
        return -1
    elif arg > 0:
        return 1
    return 0

def setMotorSpeed(id, speed):
    speed = 0 if speed == 0 else speed / abs(speed) * min([abs(speed), 100])
    sPins[id].ChangeDutyCycle(abs(speed))
    if speed == 0:
        GPIO.output(MPins[id]['FPin'],GPIO.LOW)
        GPIO.output(MPins[id]['BPin'],GPIO.LOW)
    elif speed > 0:
        GPIO.output(MPins[id]['FPin'],GPIO.HIGH)
        GPIO.output(MPins[id]['BPin'],GPIO.LOW)
    elif speed < 0:
        GPIO.output(MPins[id]['FPin'],GPIO.LOW)
        GPIO.output(MPins[id]['BPin'],GPIO.HIGH)

def dataRecognize(scans):
    global maxDistance
    distThreshold = 100
    sets = [[id, s[0]] for id, s in enumerate(scans)]
    setsAgg = []
    for id, s in enumerate(scans):
        dist = s[1]
        prevDist = scans[id-1][1]
        if dist <= maxDistance and abs(dist - prevDist) < distThreshold:
            sets[id] = sets[id-1]
    for id, s in enumerate(scans):
        if sets[sets[id][0]][0] != sets[id][0]:
            sets[id][0] = sets[sets[id][0]][0]
    for id, s in enumerate(scans):
        dist = s[1]
        if dist <= maxDistance:
            if not setsAgg or setsAgg[-1][0] != sets[id][0]:
                setsAgg.append([sets[id][0], sets[id][0]])
            else:
                setsAgg[-1][1] = id
    return [(scans[k[0]], scans[k[1]]) for k in setsAgg]
        

def showLidar(data, setsAgg):
    global max_distance    
    lcd.fill((255, 255, 255))
    for angle in range(360):
        distance = data[angle]
        if distance > 0 and distance < 1000:
            max_distance = max([min([1000, distance]), max_distance])
            radians = angle * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            point = (160 + int(x/max_distance * 119), 120 + int(y / max_distance * 119))
            lcd.set_at(point, pygame.Color(0, 0, 0))
    for s in setsAgg:
        Xa = s[0][1] * cos(s[0][0]*pi/180.0)
        Ya = s[0][1] * sin(s[0][0]*pi/180.0)
        Xb = s[1][1] * cos(s[1][0]*pi/180.0)
        Yb = s[1][1] * sin(s[1][0]*pi/180.0)
        pygame.draw.line(lcd, (255, 0, 0), (160+Xa/max_distance * 119, 120+Ya/ max_distance * 119), (160+Xb/ max_distance * 119, 120+Yb/ max_distance * 119), 1)
        sqrtLen = (Xa - Xb) ** 2 + (Ya - Yb) ** 2 
        if 10**2 <= sqrtLen and sqrtLen <= 69**2:
            pygame.draw.circle(lcd, (0, 255, 0), (int(160 + (Xa + Xb) * 0.5/max_distance * 119), int(120 + (Ya + Yb) * 0.5/ max_distance * 119)), 4, 3)
    pygame.display.update()

def lidarProcessing():
    global lidarIsActive, leftMinDistance, rightMinDistance, directionEstimate, minYplus, minYminus, nearestDistance, nearestAngle, maxNearestDistance, agentX, agentY
    maxAttemptsCount = 15
    attemptsCount = 0
    testingMeasurmentsCount = 10
    while attemptsCount < maxAttemptsCount: 
#         try:
        lidar.connect('/dev/ttyUSB0', timeout = 3, baudrate = 115200)
        lidar.set_motor_pwm(500)
        sleep(0.1)
        print("Lidar motor started")
        scan_generator = lidar.start_scan_express(4)
        print("Lidar scan generator created")
        scans = []
        i = 0
        for _, scan in enumerate(scan_generator()):
            if not scan.start_flag or len(scans) < 100:
                if scan.quality != 0:
                    scans.append((scan.angle, scan.distance))
            else:
                i += 1
                leftMinDistance = 6000
                rightMinDistance = 6000
                nearestDistance = maxNearestDistance
                nearestAngle = 0
                minYplus = 1500
                minYminus = -1500
                scan_data = [0]*360
                scanStep = 5
                leftScan = []
                rightScan = []
                setsAgg = dataRecognize(scans)
                agentX = 0
                agentY = 0                
                for a in scans:
                    angle = a[0]
                    distance = a[1]
                    scan_data[min([359, floor(angle)])] = distance
                    y = distance * sin((angle + directionEstimate)*pi/180)
                    if y > 20:
                        minYplus = min([minYplus, y])
                    elif y < -20:
                        minYminus = max([minYminus, y])
                    if abs(forwardDegree - angle - directionEstimate) < 2 * degRange and distance < nearestDistance:
                        nearestDistance = distance
                        nearestAngle = angle
                    if abs(leftDegree - angle - directionEstimate) < degRange and leftMinDistance > distance:
                        leftMinDistance = distance
                    if abs(rightDegree - angle - directionEstimate) < degRange and rightMinDistance > distance:
                        rightMinDistance = distance
                    if leftDegree < angle and angle < (forwardDegree + leftDegree) * 0.5:
                        if not leftScan or abs(leftScan[-1][0] - angle) > scanStep:
                            leftScan.append([angle, distance, distance * cos(angle/180*pi), distance * sin(angle/180*pi)])
                    if (forwardDegree + rightDegree) * 0.5 < angle and angle < rightDegree:
                        if not rightScan or abs(rightScan[-1][0] - angle) > scanStep:
                            rightScan.append([angle, distance, distance * cos(angle/180*pi), distance * sin(angle/180*pi)])
                leftDirEstimate = median([atan2(leftScan[k-1][3] - leftScan[k][3], leftScan[k-1][2] - leftScan[k][2]) for k in range(1, len(leftScan))]) if len(leftScan) >= 2 else 0
                rightDirEstimate = median([atan2(rightScan[k][3] - rightScan[k-1][3], rightScan[k][2] - rightScan[k-1][2]) for k in range(1, len(rightScan))]) if len(rightScan) >= 2 else 0
                if len(leftScan) < 2:  
                    directionEstimate = rightDirEstimate * 180 / pi
                elif len(rightScan) < 2:
                    directionEstimate = leftDirEstimate * 180 / pi
                else:
                    directionEstimate = (leftDirEstimate + rightDirEstimate) * 90 / pi
                for s in setsAgg:
                    Xa = s[0][1] * cos((s[0][0] + directionEstimate)*pi/180.0)
                    Ya = s[0][1] * sin((s[0][0] + directionEstimate)*pi/180.0)
                    Xb = s[1][1] * cos((s[1][0] + directionEstimate)*pi/180.0)
                    Yb = s[1][1] * sin((s[1][0] + directionEstimate)*pi/180.0)
                    sqrtLen = (Xa - Xb) ** 2 + (Ya - Yb) ** 2 
                    if (Xa**2+Ya**2) < 1000**2 and (Xb**2+Yb**2) < 1000**2 and 10**2 <= sqrtLen and sqrtLen <= 69**2:
                        agentX = (Xa+Xb) * 0.5
                        agentY = (Ya+Yb) * 0.5
                if i == testingMeasurmentsCount:
                    print("Lidar is succesfully initialized and tested")
                    attemptsCount = 0
                    lidarIsActive = True
                if lidarShowing:
                    print('%d: Got %d measurments; L distance: %f, R distance: %f' % (i, len(scans), leftMinDistance, rightMinDistance))
                    print([leftDirEstimate * 180 / pi, rightDirEstimate * 180 / pi, directionEstimate, minYplus, minYminus])                        
                    showLidar(scan_data, setsAgg)
                scans = []
            attemptsCount = 0
#         except:
#             print("Lidar initialization failed")
#             lidarIsActive = False
#             attemptsCount += 1
#             lidar.stop()
#             lidar.disconnect()
#             sleep(0.2)

def motorTesting():
    setMotorSpeed(leftMotorId, 100)
    setMotorSpeed(rightMotorId, 100)
    print("Going forwards")
    sleep(1)
    
    setMotorSpeed(leftMotorId, -100)
    setMotorSpeed(rightMotorId, -100)
    print("Going backwards")
    sleep(1)
    
    setMotorSpeed(leftMotorId, -100)
    setMotorSpeed(rightMotorId, 100)
    print("Going left")
    sleep(1)
    
    setMotorSpeed(leftMotorId, 100)
    setMotorSpeed(rightMotorId, -100)
    print("Going right")
    sleep(1)
    
    setMotorSpeed(leftMotorId, 0)
    setMotorSpeed(rightMotorId, 0)
    print("Stop")
    
def movingControl():
    global directionEstimate, minYplus, minYminus, leftMinDistance, rightMinDistance, nearestDistance, nearestAngle, forwardDegree, agentXInt
    coefDirect = 70/30/1.5#0.001/6 * 30
    coefPosition = 50/1000
    coefNearest = 70/1.5
    maxtu = 30
    minSpeed = 5
    agentXInt += agentX / 1000 * 30
    if abs(agentXInt) > 20:
        agentXInt = agentXInt / abs(agentXInt) * 20
    normalSpeed = 50-(agentX / 1000 * 30) - agentXInt
    setMotorSpeed(leftMotorId, normalSpeed)
    setMotorSpeed(rightMotorId, normalSpeed)
    
    maxDirectionDeviance = 40
    if nearestDistance < maxNearestDistance:
        un = coefNearest * sign(forwardDegree - nearestAngle) / (abs(forwardDegree - nearestAngle)/degRange + 1)
    else:
        un = 0
    un = 0
    u = coefDirect * directionEstimate + coefPosition * (rightMinDistance*2/3 - leftMinDistance*1/3) + un
    tu = minSpeed * sign(u) + u
    tu = tu if abs(tu) < maxtu else tu/abs(tu) * maxtu
    if abs(directionEstimate) <= maxDirectionDeviance:
        setMotorSpeed(leftMotorId, normalSpeed + tu)
        setMotorSpeed(rightMotorId, normalSpeed - tu)
    elif directionEstimate > maxDirectionDeviance:
        setMotorSpeed(leftMotorId, normalSpeed + min(tu, 0))
        setMotorSpeed(rightMotorId, normalSpeed - min(tu, 0))
    elif directionEstimate < -maxDirectionDeviance:
        setMotorSpeed(leftMotorId, normalSpeed + max(tu, 0))
        setMotorSpeed(rightMotorId, normalSpeed - max(tu, 0))
        
def loop():
    global lidarIsActive, leftMinDistance, rightMinDistance, directionEstimate
    lidarThread = Thread(target = lidarProcessing, args = [])
    lidarThread.start()
    #motorTesting()
    while True:
        while not lidarIsActive:
            sleep(0.1)
            setMotorSpeed(leftMotorId, 0)
                
            setMotorSpeed(rightMotorId, 0)        
        movingControl()        
        sleep(0.05)

def destroy():
    setMotorSpeed(leftMotorId, 0)
    setMotorSpeed(rightMotorId, 0)
    GPIO.cleanup()
    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()

if __name__ == '__main__':     # Program start from here
    #setup()
    try:
        loop()
    except KeyboardInterrupt:
        print('End of programm. Destroying.')
        destroy()

