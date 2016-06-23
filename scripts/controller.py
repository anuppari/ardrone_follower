#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from aruco_ros.msg import Center
from numpy.linalg import inv

qMult = tf.transformations.quaternion_multiply # quaternion multiplication function handle
qInv = tf.transformations.quaternion_inverse # quaternion inverse function handle
q2m = tf.transformations.quaternion_matrix # quaternion to 4x4 transformation matrix

kv = 0.3
kd = 0.3
kw = 1
kx = 1.0/17.46 # x map
ky = 1.0/13.75 # y map
kz = 1.0/1.02 # z map
kyaw = 1.0/1.72 # yaw map
kuGimbal = np.array([1.75,1.5])
TuGimbal = 2.0/3.0
kpGimbal = np.diag(np.array([1.0,1.0,0.0]))
kdGimbal = np.diag(np.array([1,1,0.0]))
kiGimbal = np.diag(np.array([0.15,0.15,0.0]))
alphaFilters = np.array([0.4, 0.3, 0.2, 0.1])
kmap = np.diag(np.array([kx,ky,kz]))

leaderID = "ugv0"
bebopID = "bebop"
ugvID = "ugv0"

desiredGimbalAngle = Twist()
desiredGimbalAngle.angular.y = -60.0
currentGimbalAngle = Twist()
offsetGimbalAngle = Twist()
outGimbalAngle = Twist()
gimbalAngleSet = False
cameraInfoRecieved = False
useGimbalOffset = True
firstRun = True
newMarkerCenter = False
A = np.zeros((3,3))
Po = np.ones(3)
frameSize = np.ones(3)
markerCenter = np.ones(3)
stackSize = 5
errorStack = np.zeros((stackSize,3))
ugvVel = Twist() # ugv vel expressed in ugv frame

def controller():
    global velPub, launchPub, landPub, resetPub, gimbalAngleSub, gimbalAnglePub
    global gimbalAngleSet, desiredGimbalAngle, ugvOdomSub, ugvVel, offsetGimbalAngle
    global currentGimbalAngle, cameraInfoRecieved, cameraInfoSub, A, frameSize, markerCenter
    global autonomy, firstRun, kpT, kdT, startTime, markerInViewTimer, Po
    
    rospy.init_node('controller') # initialize node
    
    cameraInfoSub = rospy.Subscriber(bebopID+'/camera_info',CameraInfo,cameraInfoCallback)
    r1 = rospy.Rate(30)
    print "Getting camera parameters on topic: ",bebopID,"/camera_info\n"
    print "Waiting for camera parameters ..."
    while (not cameraInfoRecieved) and (not rospy.is_shutdown()):
            r1.sleep()
    print "Got camera parameters"
    
    # Initialize stuff
    override = True
    autonomy = False
    lastTrans = np.array([0,0,0])
    lastTime = rospy.Time.now()
    startTime = lastTime
    
    joySub = rospy.Subscriber(bebopID+'/joy',Joy,joyCallback)
    gimbalAngleSub = rospy.Subscriber(bebopID+'/camera_control',Twist,gimbalAngleCallback)
    markerCenterSub = rospy.Subscriber('/markerCenters',Center,markerCenterCallback)
    ugvOdomSub = rospy.Subscriber(ugvID+'/odom',Odometry,ugvVelCallback)
    velPub = rospy.Publisher(bebopID+'/cmd_vel',Twist,queue_size=1)
    velPub.publish(Twist()) # zero out previous commands
    launchPub = rospy.Publisher(bebopID+'/takeoff',Empty,queue_size=1)
    landPub = rospy.Publisher(bebopID+'/land',Empty,queue_size=1)
    resetPub = rospy.Publisher(bebopID+'/reset',Empty,queue_size=1)
    gimbalAnglePub = rospy.Publisher(bebopID+'/camera_control',Twist,queue_size=1)
    tfListener = tf.TransformListener() # get transform listener
    markerInViewTimer = rospy.Timer(rospy.Duration(3.0),timerCallback)
    
    # Mocap based follower control
    r = rospy.Rate(30) # command frequency
    while not rospy.is_shutdown():       
        try:
            now = rospy.Time.now()
            tfListener.waitForTransform(bebopID,leaderID,now,rospy.Duration(0.5))
            (trans,quat) = tfListener.lookupTransform(bebopID,leaderID,now)
            trans = np.array(trans)
            
            #angles = np.dot(np.arctan(np.array([trans[0]/trans[2],trans[1]/trans[2]])),180.0/np.pi)
            #offsetGimbalAngle.angular.z = -1*angles[0]
            #offsetGimbalAngle.angular.y = -1*angles[1]
    
            ugvVelBebopLin = rotateVec(np.array([ugvVel.linear.x,0,0]),quat) # ugv linear vel expressed in bebop frame
            ugvVelBebopAng = rotateVec(np.array([0.0,0.0,ugvVel.angular.z]),quat) # ugv angular vel expressed in bebop frame
            
            desOrientVec = rotateVec(np.array([1,0,0]),quat)
            angVelVec = np.cross([1,0,0],desOrientVec)
            angVelCmd = kw*angVelVec[2] + kyaw*ugvVelBebopAng[2]
        
            trans[0] -= 0.0 # Hover 1m behind bot
            trans[2] += 1.0 # Hover 1m above bot
            transDeriv = (trans-lastTrans)/(now-lastTime).to_sec()
            linVelCmd = kv*trans + kd*transDeriv + np.dot(ugvVelBebopLin,kmap)
            
            lastTime = now
            lastTrans = trans
            
            if autonomy:
                velCmd = Twist()
                (velCmd.linear.x,velCmd.linear.y,velCmd.linear.z) = linVelCmd
                velCmd.angular.z = -1.0*angVelCmd
                velPub.publish(velCmd)
        except:
            if autonomy:
                velPub.publish(Twist())
            pass
        finally:
            r.sleep()
    
    rospy.spin()
    velPub.publish(Twist())

def timerCallback(event):
    global desiredGimbalAngle, gimbalAnglePub, firstRun
    gimbalAnglePub.publish(desiredGimbalAngle)
    firstRun = True

def cameraInfoCallback(data):
    global cameraInfoRecieved, A, frameSize, cameraInfoSub, Po
    #get camera info
    A = data.K
    A = np.reshape(A,(3,3))
    Po[0] = A[0][2]
    Po[1] = A[1][2]
    frameSize[0] = data.width
    frameSize[1] = data.height
    cameraInfoRecieved = True
    cameraInfoSub.unregister()
    
def markerCenterCallback(data):
    global markerCenter, offsetGimbalAngle, frameSize, gimbalAnglePub, markerInViewTimer, outGimbalAngleLast, outGimbalAngle
    global alphaFilter, firstRun, errorLast, errorStack, stackSize, kpGimbal, kdGimbal, kiGimbal, newMarkerCenter
    markerInViewTimer.shutdown()
    markerCenter[0] = data.x
    markerCenter[1] = data.y
    errorPixel = markerCenter + Po
    print "\n\nerrorPixel: ",errorPixel
    print "errorNormEuclid: ",np.dot(inv(A),errorPixel)
    error = 180.0/np.pi*np.arctan(np.dot(inv(A),errorPixel))
    print "error: ", error
    print "error*kp: ", np.dot(kpGimbal,error)
    if firstRun:
        firstRun = False
        for ii in xrange(0,stackSize-1):
            errorStack[ii][:] = error
    errorStack[0:stackSize][:] = np.append(errorStack[1:stackSize][:],[error],axis=0)
    #print "errorStack: ",errorStack
    #alphaSizeTemp = alphaFilters.shape
    #errorSum = np.trapz(errorStack,axis=0)
    #errorDot = alphaFilters[0]*(errorStack[stackSize-1][:]-errorStack[stackSize-2][:])
    errorDot = alphaFilters[0]*(errorStack[stackSize-1][:]-errorStack[stackSize-2][:])+alphaFilters[1]*(errorStack[stackSize-2][:]-errorStack[stackSize-3][:])+alphaFilters[2]*(errorStack[stackSize-3][:]-errorStack[stackSize-4][:])+alphaFilters[3]*(errorStack[stackSize-4][:]-errorStack[stackSize-5][:])
    print "errorDot: ",errorDot
    print "errorDot*kd: ",np.dot(kdGimbal,errorDot)
    #u = np.dot(kpGimbal,error) + np.dot(kdGimbal,errorDot) + np.dot(kiGimbal,errorSum)
    u = np.dot(kpGimbal,error) + np.dot(kdGimbal,errorDot)
    print "u: ",u
    if useGimbalOffset:
        outGimbalAngle.angular.y = desiredGimbalAngle.angular.y - u[1]
        outGimbalAngle.angular.z = desiredGimbalAngle.angular.z + u[0]
        gimbalAnglePub.publish(outGimbalAngle)
    else:
        gimbalAnglePub.publish(desiredGimbalAngle)
    markerInViewTimer = rospy.Timer(rospy.Duration(0.5),timerCallback)


def gimbalAngleCallback(data):
    global currentGimbalAngle
    currentGimbalAngle = data
    #print "current gimbal angle\n", currentGimbalAngle

def ugvVelCallback(data):
    global ugvVel
    ugvVel = data.twist.twist
    #print "ugv vel\n", ugvVel

def joyCallback(data):
    global autonomy
    
    if data.buttons[0]: # A - land
        landPub.publish(Empty())
    if data.buttons[3]: # Y - takeoff
        launchPub.publish(Empty())
    if data.buttons[1]: # B - reset
        resetPub.publish(Empty())
    
    autonomy = data.buttons[2] # X - autonomy
    
    #override = data.buttons[4] or data.buttons[5] # LB/RB - override buttons
    if not autonomy: 
        joyVel = Twist()
        (joyVel.linear.x,joyVel.linear.y,joyVel.linear.z) = (deadband(data.axes[4]),deadband(data.axes[3]),deadband(data.axes[1]))
        joyVel.angular.z = -1*deadband(data.axes[0])
        velPub.publish(joyVel)


def deadband(vel):
    if np.absolute(vel) < 0.1:
        outVel = 0.0
    else:
        outVel = vel
    
    return outVel


def rotateVec(p,q):
    return qMult(q,qMult(np.append(p,0),qInv(q)))[0:3]


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
