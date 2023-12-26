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
def animate(i, xs, ys1, ys2, ys3, ys4, threshold):
    s = ser.readline().decode('utf-8').rstrip()
    vals = readSkin(s)
    xs.append(vals[4]/1000)
    ys1.append(vals[0])
    ys2.append(vals[1])
    ys3.append(vals[2])
    ys4.append(vals[3])

    # Limit x and y lists to 20 items
    xs = xs[-200:]
    ys1 = ys1[-200:]
    ys2 = ys2[-200:]
    ys3 = ys3[-200:]
    ys4 = ys4[-200:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys1, label = "Sensor 1", linewidth=2)
    ax.plot(xs, ys2, label = "Sensor 2", linewidth=2)
    ax.plot(xs, ys3, label = "Sensor 3", linewidth=2)
    ax.plot(xs, ys4, label = "Sensor 4", linewidth=2)
    ax.plot([xs[0], xs[len(xs)-1]], [threshold, threshold], label = "Threshold", linestyle="--")
    plt.ylim(0, 300)

    #Plot Labeling
    ax.set_ylabel('Clock Cycles')
    ax.set_xlabel('Time (s)')
    ax.set_title('Sensed Capacitance')
    legend = plt.legend()

    #Connecting to ROS
    isTouched = isTouching(vals, threshold)
    touch_msg = Int8MultiArray()
    touch_msg.data = isTouched
    pub.publish(touch_msg)
    rate.sleep()


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
        if plotData[4][tCount] < 200:
            break 
plotData = [[0,0], [0,0], [0,0], [0,0], [0,0]]

#Plotting 
# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys1 = []
ys2 = []
ys3 = []
ys4 = []
ax.set_ylabel('Clock Cycles')
ax.set_xlabel('Time (s)')
ax.set_title('Sensed Capacitance')

# Create the animation
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys1, ys2, ys3, ys4, threshold), interval=10)
# Display the graph
plt.show()

#while ser.is_open:
    #s = ser.readline().decode('utf-8').rstrip()
    #vals = readSkin(s)
    #isTouched = isTouching(vals, threshold)
    #plt.show()
    #touch_msg = Int8MultiArray()
    #touch_msg.data = isTouched
    #pub.publish(touch_msg)
    #rate.sleep()


