import time
import numpy as np
import cv2
from queue import PriorityQueue
import sim
import sys
import ctypes

im = None
allblocks = []
finish = []

def getPoints(x, y):
    return 0.001458 * x - 0.1702 , -0.001456 * y + 	2.420

def getCountours(lower, upper, hsv):
    lower = np.array(lower)
    upper = np.array(upper)
    mask = cv2.inRange(hsv, lower, upper)
    cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return cnts

def getPosition(box):
    return ((box[0][0]+box[2][0])/2, (box[0][1]+box[2][1])/2)

def getAngle(box):
    print("------")
    print(box)
    qu = PriorityQueue()
    for point in box:
        print(point)
        qu.put((point[1], point))
    p1 = qu.get()[1]
    p2 = qu.get()[1]

    angle = np.arctan( abs(p1[1]-p2[1]) / abs(p1[0]-p2[0]))

    if p1[0] > p2[0]:
        return -np.radians(360 - np.degrees(angle))
    if p1[0] < p2[0]:
        return -angle

def getPose(c):
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    angle = getAngle(box)
    posx, posy = getPosition(box)
    return angle, posx, posy

def checksurr(curr):
    for pt in allblocks:
        if pt[2] not in finish and pt[1][2] == curr[1][2] and abs(pt[1][1] - curr[1][1]) < 0.06:
            finish.append(pt[2])
            return pt
    return None

def calculateBlockPoses(q, rangeLower, rangeUpper, hsv, index, color):
    mask = cv2.inRange(hsv, np.array(rangeLower), np.array(rangeUpper))
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    for c in contours:
        angle, cx, cy = getPose(c)
        print(angle)
        cx, cy = getPoints(cx, cy)
        allblocks.append((cy, (cx, cy, color, angle), index))
        q.put((cy, (cx, cy, color, angle), index))
        index += 1
    return index


def fillBlockPriorityQ(q):
    e, cam = sim.simxGetObjectHandle(clientID, "Vision_sensor", sim.simx_opmode_blocking)
    error, res, i = sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_streaming)
    time.sleep(2)
    error, resolution, image = sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_buffer)
    print(resolution)
    img2 = np.array(image, dtype=np.uint8)
    img2.resize([resolution[1], resolution[0], 3])
    img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2BGR)
    img2 = cv2.flip(img2, 1)
    hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
    index = 0
    index = calculateBlockPoses(q, [0,50,20], [5,255,255], hsv, index, "red")
    index = calculateBlockPoses(q, [40, 40,40], [70, 255,255], hsv, index, "green")
    index = calculateBlockPoses(q, [110,50,50], [130,255,255], hsv, index, "blue")


print ('Program started')
sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')

    while True:
        q = PriorityQueue()
        fillBlockPriorityQ(q)

        curr = q.get()
        if curr[2] in finish:
            continue
        else:
            finish.append(curr[2])

        print(curr)
        time.sleep(5)   
        emptyBuff = bytearray()
        code = 1
        if curr[1][2] == "red":
            code = 1
        if curr[1][2] == "blue":
            code = 2
        if curr[1][2] == "green":
            code = 3
        
        second = checksurr(curr)
        gotsec = 1
        if second == None:
            gotsec = 0
        if gotsec == 0:
            res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'controlserver',sim.sim_scripttype_childscript,'goandpickup',[code, gotsec],[curr[1][0], curr[1][1], curr[1][3], 0, 0, 0],['redRectangle1'],emptyBuff,sim.simx_opmode_oneshot_wait)

        else:
            print("got second")
            res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'controlserver',sim.sim_scripttype_childscript,'goandpickup',[code, gotsec],[curr[1][0], curr[1][1], curr[1][3], second[1][0], second[1][1], second[1][3]],['redRectangle1'],emptyBuff,sim.simx_opmode_oneshot_wait)

        cod, stri = sim.simxGetStringSignal(clientID, "pick", sim.simx_opmode_streaming)

        cod, stri = sim.simxGetStringSignal(clientID, "pick", sim.simx_opmode_buffer)
        s = bytearray.decode(stri)
        while s != "pick":
            cod, stri = sim.simxGetStringSignal(clientID, "pick", sim.simx_opmode_buffer)
            s = bytearray.decode(stri)
            time.sleep(0.1)
        if res==sim.simx_return_ok:
            print ('Return string: ',retStrings[0])
        else:
            print ('Remote function call failed')

        print ("picked")

        while s != "finish":
            cod, stri = sim.simxGetStringSignal(clientID, "pick", sim.simx_opmode_buffer)
            s = bytearray.decode(stri)
            time.sleep(0.1)
        print ("finished")
        #time.sleep(10)

        while not q.empty():
            tempp = q.get()
        allblocks.clear()
        finish.clear()
        print("emptied")
    
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
