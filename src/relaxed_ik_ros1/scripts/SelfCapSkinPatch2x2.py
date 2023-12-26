#! /usr/bin/env python3
# Robotic Skin link to Franka arm
# Carson Kohlbrenner
# 7/12/2023

# Making the serial connection
import serial
import rospy 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import String
from std_msgs.msg import Int8MultiArray


# Main Parameters
ser = serial.Serial('/dev/ttyACM0', 115200) #Serial port /dev/ttyACM0, 9600 baud
threshold = 100
tCount = 1
tPrev = 0
plotData = [[0,0], [0,0], [0,0], [0,0], [0,0]]

######### Helper Functions ##########
def readSkin(s):
    sepData = s.split(',')
    value = []
    value = [0 for i in range(len(sepData))]
    for i in range(len(sepData)):
        if len(sepData)==5:
            value[i] = int(sepData[i])
    return value

def readSkinReset(s, plotData, tCount):

    sepData = s.split(',')
    value = []
    value = [0 for i in range(len(sepData))]
    if len(sepData)==5:
        for i in range(len(sepData)):
            value[i] = int(sepData[i])
            plotData[i].append(value[i])
        tCount+=1
    return tCount

def isTouching(vals, threshold):
    isTouched = []
    isTouched = [0 for i in range(len(vals)-1)]
    for i in (range(len(vals)-1)):
        if vals[i] > threshold:
            isTouched[i] = 1
    return isTouched
            

# Function to animate the graph
def animate(i, xs, ys):
    s = ser.readline().decode('utf-8').rstrip()
    vals = readSkin(s)


# Ros Connection
pub = rospy.Publisher('chatter', String, queue_size=10)
pub = rospy.Publisher('/skin_touch', Int8MultiArray, queue_size=1)
rospy.init_node('robot_skin', anonymous=True)
rate = rospy.Rate(100)

########## MAIN LOOP ###########
# Flush bad data
while ser.is_open:
    s = ser.readline().decode('utf-8').rstrip()
    tPrev = tCount
    tCount = readSkinReset(s, plotData, tCount)
    if tCount != tPrev:
        print(tCount)
        print(plotData[4][tCount]) 
        if plotData[4][tCount] < 200:
            break 

while ser.is_open:
    s = ser.readline().decode('utf-8').rstrip()
    vals = readSkin(s)
    isTouched = isTouching(vals, threshold)
    touch_msg = Int8MultiArray()
    touch_msg.data = isTouched
    pub.publish(touch_msg)
    rate.sleep()


