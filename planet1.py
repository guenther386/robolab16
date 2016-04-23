#!/usr/bin/env python3
import ev3dev.ev3 as ev3
import math
import time
import paho.mqtt.client as mqtt

motorA = ev3.LargeMotor('outA')
motorB = ev3.LargeMotor('outB')
motorA.command = 'reset'
motorB.command = 'reset'
motorA.speed_regulation_enabled = 'on'
motorB.speed_regulation_enabled = 'on'
motorA.command = 'run-direct'
motorB.command = 'run-direct'
cs = ev3.ColorSensor()
cs.mode = 'RGB-RAW'
gs = ev3.GyroSensor()
gs.mode = 'GYRO-CAL'
gs.mode = 'GYRO-ANG'
posx = 0
posy = 0
head = 0
last = 0
lastx = 0
lasty = 0
current = 0
middle = 330
string = ""
nodes = []
travel = []
offsetx = 0
offsety = 0
target = None
sent = ""

class node:
    def __init__(self, num, pos):
        self.num=num
        self.pos=pos
        self.list = [None, None, None, None]
    def add(self, nod, d):
        self.list[d] = nod
        return None

def init():
    global middle
    white = 0
    black = 600
    motorA.speed_sp = -150
    motorB.speed_sp = 150
    motorA.command = 'run-forever'
    motorB.command = 'run-forever'
    while (angle()-360) < 0:
        n=cs.bin_data("hhh")
        if sum(n) > white:
            white = sum(n)
        elif sum(n) < black:
            black = sum(n)
    motorA.stop()
    motorB.stop()
    middle = (black + white)/2
    motorA.command = 'run-direct'
    motorB.command = 'run-direct'
    motorA.position = 0
    motorB.position = 0
    x = 0
    while x < 100000:
        x += 1
    gs.mode = 'GYRO-CAL'
    gs.mode = 'GYRO-ANG'
    return None

def sum(tupel):
    r,g,b = tupel
    if r > (g + b):
        return 0
    else:
        return r + g + b

def sqrtb(n):
    x = 1
    if n > 0:
        x = 1
    else:
        x = -1
    n=int(math.sqrt(math.sqrt(n*n)*30)) * x
    return n

def path(n):
    global current
    dictl = {}
    dictp = {}
    i=0
    while i < len(nodes):
        dictl[i] = 999999
        i += 1
    dictl[current] = 0
    dictp[current] = current
    if nodes[current].list[0] is not None and nodes[current].list[0] > -1:
        dictl[nodes[current].list[0]] = 1
        dictp[nodes[current].list[0]] = current
    if nodes[current].list[1] is not None and nodes[current].list[1] > -1:
        dictl[nodes[current].list[1]] = 1
        dictp[nodes[current].list[1]] = current
    if nodes[current].list[2] is not None and nodes[current].list[2] > -1:
        dictl[nodes[current].list[2]] = 1
        dictp[nodes[current].list[2]] = current
    if nodes[current].list[3] is not None and nodes[current].list[3] > -1:
        dictl[nodes[current].list[3]] = 1
        dictp[nodes[current].list[3]] = current
    temp = 1
    while temp < len(nodes):
        i = 0
        while i < len(nodes):
            if dictl[i] == temp:
                if nodes[i].list[0] is not None and nodes[i].list[0] > -1 and dictl[nodes[i].list[0]] > (temp + 1):
                    dictl[nodes[i].list[0]] = temp + 1
                    dictp[nodes[i].list[0]] = i
                if nodes[i].list[1] is not None and nodes[i].list[1] > -1 and dictl[nodes[i].list[1]] > (temp + 1):
                    dictl[nodes[i].list[1]] = temp + 1
                    dictp[nodes[i].list[1]] = i
                if nodes[i].list[2] is not None and nodes[i].list[2] > -1 and dictl[nodes[i].list[2]] > (temp + 1):
                    dictl[nodes[i].list[2]] = temp + 1
                    dictp[nodes[i].list[2]] = i
                if nodes[i].list[3] is not None and nodes[i].list[3] > -1 and dictl[nodes[i].list[3]] > (temp + 1):
                    dictl[nodes[i].list[3]] = temp + 1
                    dictp[nodes[i].list[3]] = i
            i += 1
        temp += 1
    while n != current:
        travel.append(n)
        n = dictp[n]
    return None

def next():
    global current
    if nodes[current].list[0] == -1:
        return 0
    if nodes[current].list[1] == -1:
        return 1
    if nodes[current].list[2] == -1:
        return 2
    if nodes[current].list[3] == -1:
        return 3
    return None

def mini():
    global current
    n = 999999
    if nodes[current].list[0] is not None and nodes[current].list[0] < n:
        n = nodes[current].list[0]
    if nodes[current].list[1] is not None and nodes[current].list[1] < n:
        n = nodes[current].list[1]
    if nodes[current].list[2] is not None and nodes[current].list[2] < n:
        n = nodes[current].list[2]
    if nodes[current].list[3] is not None and nodes[current].list[3] < n:
        n = nodes[current].list[3]
    return n

def search():
    global last
    global current
    global travel
    if target is not None:
        if len(travel) == 0:
            return None
        n = travel.pop()
        last = current
        if nodes[current].list[0] is not None and nodes[current].list[0] == n:
            follow_to(0)
        if nodes[current].list[1] is not None and nodes[current].list[1] == n:
            follow_to(1)
        if nodes[current].list[2] is not None and nodes[current].list[2] == n:
            follow_to(2)
        if nodes[current].list[3] is not None and nodes[current].list[3] == n:
            follow_to(3)
    elif next() is not None:
        last = current
        travel = [] #might have stumbled upon an unexplored node
        follow_to(next())
    else:
        travelto = None
        if len(travel) == 0:
            i = len(nodes)-1
            while i > -1 and travelto is None:
                if nodes[i].list[0] == -1:
                    travelto = i
                if nodes[i].list[1] == -1:
                    travelto = i
                if nodes[i].list[2] == -1:
                    travelto = i
                if nodes[i].list[3] == -1:
                    travelto = i
                i -= 1
            path(travelto)
        n = travel.pop()
        last = current
        if nodes[current].list[0] is not None and nodes[current].list[0] == n:
            follow_to(0)
        if nodes[current].list[1] is not None and nodes[current].list[1] == n:
            follow_to(1)
        if nodes[current].list[2] is not None and nodes[current].list[2] == n:
            follow_to(2)
        if nodes[current].list[3] is not None and nodes[current].list[3] == n:
            follow_to(3)
    return None

def start():
    global middle
    global client
    global string
    n = middle + 1
    while n > 0:
        n-=middle
        motorA.speed_sp = 100 + 0.3*n
        motorB.speed_sp = 100 - 0.3*n
        motorA.command = 'run-forever'
        motorB.command = 'run-forever'
        n=sum(cs.bin_data("hhh"))
    nodes.append(node(len(nodes),(0,0)))
    nodes[0].list[2]=-2
    n = 0
    while n == 0:
        motorA.speed_sp = 100
        motorB.speed_sp = 100
        motorA.command = 'run-forever'
        motorB.command = 'run-forever'
        n=sum(cs.bin_data("hhh"))
    time.sleep(0.8)
    motorA.stop()
    motorB.stop()
    ev3.Sound.tone([(400,100,100), (333,200)])
    return None

def pos(posA, posB):
    global posx
    global posy
    global head
    global lastx
    global lasty
    posx += (posA-lastx+posB-lasty)/2 * math.sin((head+angle()/180*3.1415926535)/2)
    posy += (posA-lastx+posB-lasty)/2 * math.cos((head+angle()/180*3.1415926535)/2)
    head = angle()/180*3.14159265358
    lastx = motorA.position
    lasty = motorB.position
    return None

def position():
    global posx
    global posy
    posx = 250 * (round(posx/250))
    posy = 250 * (round(posy/250))
    return (int(posx/250),int(posy/250))

def done():
    i = 0
    while i < len(nodes):
        if nodes[i].list[0] == -1:
            return 0
        if nodes[i].list[1] == -1:
            return 0
        if nodes[i].list[2] == -1:
            return 0
        if nodes[i].list[3] == -1:
            return 0
        i += 1
    return 1 #returns 1 if there are no unkown edges

def follow_to(i):
    global last
    global current
    global posx
    global posy
    global head
    global lastx
    global lasty
    start = gs.value()
    motorA.speed_sp = -150
    motorB.speed_sp = 150
    motorA.command = 'run-forever'
    motorB.command = 'run-forever'
    while start > (gs.value()-(i*93)):
        x=1 #do nothing
    motorA.stop()
    motorB.stop()
    lastx = motorA.position
    lasty = motorB.position
    n = middle + 1
    while n > 0:
        n-=middle #384
        motorA.speed_sp = 75 + 0.3*n
        motorB.speed_sp = 75 - 0.3*n
        motorA.command = 'run-forever'
        motorB.command = 'run-forever'
        n=sum(cs.bin_data("hhh"))
        pos(motorA.position, motorB.position)
    print("posx =", end=" ")
    print(posx, end=" ")
    print("posy =", end=" ")
    print(posy, end=" ")
    pos_ = position()
    print(pos_)
    j = 0
    while j < len(nodes) and nodes[j].pos != pos_:
        j += 1
    current = j
    direct = 0
    if 22 <= angle()%360 < 67:
        direct = 0
    elif 67 <= angle()%360 < 157:
        direct = 1
    elif 157 <= angle()%360 < 247:
        direct = 2
    elif 247 <= angle()%360 < 337:
        direct = 3
    n = 0
    while n == 0:
        motorA.speed_sp = 100
        motorB.speed_sp = 100
        motorA.command = 'run-forever'
        motorB.command = 'run-forever'
        n=sum(cs.bin_data("hhh"))
    time.sleep(0.8)
    motorA.stop()
    motorB.stop()
    if j == len(nodes):
        nodes.append(node(len(nodes),pos_))
        explore()
    nodes[current].add(last, direct-2)
    nodes[last].add(current, i)
    time.sleep(2)
    direct = 0
    if 0 <= angle()%360 < 23:
        direct = 0
    elif 23 <= angle()%360 < 68:
        direct = 1
    elif 68 <= angle()%360 < 113:
        direct = 0
    elif 113 <= angle()%360 < 158:
        direct = 1
    elif 158 <= angle()%360 < 203:
        direct = 0
    elif 203 <= angle()%360 < 248:
        direct = 1
    elif 248 <= angle()%360 < 293:
        direct = 0
    elif 293 <= angle()%360 < 338:
        direct = 1
    motorA.speed_sp = -150
    motorB.speed_sp = 150
    while angle()%360 > 6:
        motorA.command = 'run-forever'
        motorB.command = 'run-forever'
    if direct == 1:
        while angle()%360 < 52:
            motorA.command = 'run-forever'
            motorB.command = 'run-forever'
    motorA.stop()
    motorB.stop()
    if target is not None:
        i = 0
        while i < len(nodes) and nodes[i].pos != target:
            i += 1
        if i < len(nodes):
            path(i)
            search()
    elif done() == 0:
        search()
    elif current != 0 and target is None:
        path(0)
        search()
    return None

def angle():
    return gs.value() + (gs.value()/90)*3.75

def explore():
    global last
    global current
    direct = 0
    motorA.stop()
    motorB.stop()
    if 0 <= angle()%360 < 23:
        direct = 0
    elif 23 <= angle()%360 < 68:
        direct = 1
    elif 68 <= angle()%360 < 113:
        direct = 0
    elif 113 <= angle()%360 < 158:
        direct = 1
    elif 158 <= angle()%360 < 203:
        direct = 0
    elif 203 <= angle()%360 < 248:
        direct = 1
    elif 248 <= angle()%360 < 293:
        direct = 0
    elif 293 <= angle()%360 < 338:
        direct = 1
    motorA.speed_sp = -150
    motorB.speed_sp = 150
    motorA.command = 'run-forever'
    motorB.command = 'run-forever'
    if direct == 0: #gerader Knoten
        start = angle()
        while start > (angle()-360):
            while angle()%360 < 45 or angle()%360 > 315:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[0] is None:
                        nodes[current].list[0] = -1
            while angle()%360 < 135:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[1] is None:
                        nodes[current].list[1] = -1
            while angle()%360 < 225:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[2] is None:
                        nodes[current].list[2] = -1
            while angle()%360 < 315:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[3] is None:
                        nodes[current].list[3] = -1
            while angle()%360 < 359:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[0] is None:
                        nodes[current].list[0] = -1
    if direct == 1: #45Â° versetzter Knoten
        start = angle()
        while start > (angle()-358):
            while angle()%360 < 90:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[0] is None:
                        nodes[current].list[0] = -1
            while angle()%360 < 180:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[1] is None:
                        nodes[current].list[1] = -1
            while angle()%360 < 270:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[2] is None:
                        nodes[current].list[2] = -1
            while angle()%360 < 359:
                if sum(cs.bin_data("hhh")) < 100:
                    if nodes[current].list[3] is None:
                        nodes[current].list[3] = -1
    while angle()%360 > 1:
        x=1 #do nothing
    if direct == 1:
        while angle()%360 < 46:
            x=1 #do nothing
    motorA.stop()
    motorB.stop()
    print("current =", end=" ")
    print(current, end=",")
    print(" last =", end=" ")
    print(last)
    print(nodes[current].list)
    print(nodes[current].pos)
    print("-------------------")
    return None

init()
start()
explore()
search()
i=0
while i < len(nodes):
    print(nodes[i].list)
    print(nodes[i].pos)
    i +=1
motorA.stop()
motorB.stop()
