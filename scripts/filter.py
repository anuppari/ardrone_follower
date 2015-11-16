#!/usr/bin/env python

import rospy
import numpy as np
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import PoseStamped, PointStamped
import threading
import quaternionFunctions as qf

lock = threading.Lock()

def filter():
    global statePub, lastTime
    global F, H, Q, R, x, P, index
    
    rospy.init_node('filter') # initialize node
    
    # filter properties
    F = np.eye(3)
    H = np.eye(3)
    Q = np.eye(3)
    R = 0.05*np.eye(3)
    x = np.zeros((3,1))
    P = np.eye(3)
    index = 0
    
    lastTime = 0
    
    poseSub = rospy.Subscriber('markers',PoseStamped,poseCallback)
    
    # wait until state estimates are initialized by first aruco update
    while (not rospy.is_shutdown()) and lastTime == 0:
        pass
    
    velSub = rospy.Subscriber('ardrone/navdata',Navdata,velUpdateCallback)
    statePub = rospy.Publisher('filteredState',PointStamped,queue_size=1)
    
    rospy.spin()


def velUpdateCallback(navData):
    global lastTime
    global x, P, index
    
    v = 0.001*np.transpose(np.array([[navData.vx,navData.vy,navData.vz]])) # velocity in [m/s]
    
    timeNow = rospy.Time.now()
    dt = (timeNow - lastTime)
    dt = dt.secs + dt.nsecs*1e-9
    
    lock.acquire()
    try:
        x = np.dot(F,x) + v*dt
        P = np.dot(np.dot(F,P),np.transpose(F))
        lastTime = timeNow
        
        pointMsg = PointStamped()
        pointMsg.header.stamp = timeNow
        (pointMsg.point.x,pointMsg.point.y,pointMsg.point.z) = (x[0],x[1],x[2])
        
        statePub.publish(pointMsg)
    finally:
        lock.release()


def poseCallback(data):
    global lastTime
    global x, P, index
    
    pos = data.pose.position # pose of marker w.r.t. camera
    pos = np.transpose(np.array([[pos.x,pos.y,pos.z]]))
    quat = np.array([[data.pose.orientation.w,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z]]).transpose()
    pos = -1*qf.rotateVec(pos,qf.inverse(quat)) # position of camera w.r.t. marker
    
    index+=1
    
    if lastTime == 0: # initialize state estimate
        x = pos
        lastTime = rospy.Time.now()
    else:
        z = pos
        lock.acquire()
        try:
            y = z - np.dot(H,x)
            S = np.dot(np.dot(H,P),np.transpose(H)) + R
            K = np.dot(np.dot(P,np.transpose(H)),np.linalg.inv(S))
            x = x + np.dot(K,y)
            P = np.dot((np.eye(3) - np.dot(K,H)),P)
            
            pointMsg = PointStamped()
            pointMsg.header.stamp = rospy.Time.now()
            (pointMsg.point.x,pointMsg.point.y,pointMsg.point.z) = (x[0],x[1],x[2])
            
            statePub.publish(pointMsg)
        finally:
            lock.release()


if __name__ == '__main__':
    try:
        filter()
    except rospy.ROSInterruptException:
        pass
