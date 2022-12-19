#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math as m
import numpy as np

up, down, first_callback = True, False, False

def joint_callback(msg):
    global th1_i, th2_i, th3_i
    th1_i, th2_i, th3_i = msg.position[0], msg.position[1], msg.position[2]
    global first_callback
    first_callback = True
    # print(th1_i, th2_i, th3_i)
        
joint1_publisher = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
joint2_publisher = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)
joint3_publisher = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size=10)

joint_subscriber = rospy.Subscriber('/rrbot/joint_states', JointState, joint_callback)
rospy.init_node('joint_publishing_node', anonymous=True)
rate = rospy.Rate(200) # 10hz

def inverse_kinematics(x, y, z, up_down):
    L1, L2, L3 = 0.5, 1, 1
    theta1 = m.atan2(y,x)
    if theta1 <= -m.pi:
        theta1 += m.pi

    Ld = m.sqrt(x**2 + y**2 + (z - L1)**2)

    if up_down:
        theta3 = -(m.pi - m.acos((L2**2 + L3**2 - Ld**2) / (2*L2*L3)))
        theta2 = -(m.atan2(m.sqrt(x**2 + y**2), z - L1) + theta3/2)
    else:
        theta3 = m.pi - m.acos((L2**2 + L3**2 - Ld**2) / (2*L2*L3))
        theta2 = -(m.atan2(m.sqrt(x**2 + y**2), z - L1) + theta3/2)
    
    print("Target Joint Values :", theta1*180/m.pi, theta2*180/m.pi, theta3*180/m.pi)

    return theta1, theta2, theta3

def Rz(theta):
    return np.matrix([[np.cos(theta), -np.sin(theta), 0, 0],
                    [np.sin(theta), np.cos(theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

def Rx(theta):
    return np.matrix([[1, 0, 0, 0],
                    [0, np.cos(theta), -np.sin(theta), 0],
                    [0, np.sin(theta), np.cos(theta), 0],
                    [0, 0, 0, 1]])

def Tx(a):
    return np.matrix([[1, 0, 0, a],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

def Tz(d):
    return np.matrix([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1]])

def forward_kinematics(th1, th2, th3):
    L1, L2, L3 = 0.5, 1, 1
    H01 = Rz(th1) * Tz(L1) * Rx(m.pi/2) * Tx(0)
    H12 = Rz(th2 + m.pi/2) * Tz(0) * Rx(0) * Tx(L2)
    H23 = Rz(th3) * Tz(0) * Rx(0) * Tx(L3)

    H = H01 * H12 * H23

    print(H)


while not rospy.is_shutdown():
    x, y, z = input("Enter the x, y, z position : ").split()

    try:
        th1, th2, th3 = inverse_kinematics(float(x), float(y), float(z), up)
    except:
        print("Out of Joint Configuration!")
        continue
    forward_kinematics(th1,th2,th3)
    th1_t, th2_t, th3_t = [], [], []
    
    step = 1000

    initial_state_read = True
    error = 0.01 # degree

    joint1_msg = Float64()
    joint2_msg = Float64()
    joint3_msg = Float64()

    if first_callback:
        if initial_state_read:
            initial_state_read = False
            print("initial_pose : ", th1_i, th2_i, th3_i)
            for i in range(step):
                th1_t.append((i + 1) * (th1 - th1_i) / step + th1_i)
                th2_t.append((i + 1) * (th2 - th2_i) / step + th2_i)
                th3_t.append((i + 1) * (th3 - th3_i) / step + th3_i)

        for i in range(step):
            
            joint1_msg.data = th1_t[i]
            joint2_msg.data = th2_t[i]
            joint3_msg.data = th3_t[i]

            joint1_publisher.publish(joint1_msg)
            joint2_publisher.publish(joint2_msg)
            joint3_publisher.publish(joint3_msg)

            # print(i)
            rate.sleep()
    else:
        print("Waiting Joint States...")
# rospy.spin()