#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

qMult = tf.transformations.quaternion_multiply # quaternion multiplication function handle
qInv = tf.transformations.quaternion_inverse # quaternion inverse function handle
q2m = tf.transformations.quaternion_matrix # quaternion to 4x4 transformation matrix

kv = 0.2
kd = 0.2
kw = 1
leaderID = "ugv0"

def controller():
    global velPub, launchPub, landPub, resetPub
    global override, autonomy
    
    rospy.init_node('controller') # initialize node
    
    # Initialize stuff
    override = True
    autonomy = False
    lastTrans = np.array([0,0,0])
    lastTime = rospy.Time.now()
    
    joySub = rospy.Subscriber('joy',Joy,joyCallback)
    velPub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    velPub.publish(Twist()) # zero out previous commands
    launchPub = rospy.Publisher('takeoff',Empty,queue_size=1)
    landPub = rospy.Publisher('land',Empty,queue_size=1)
    resetPub = rospy.Publisher('reset',Empty,queue_size=1)
    tfListener = tf.TransformListener() # get transform listener
    
    # Mocap based follower control
    r = rospy.Rate(10) # command frequency
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            tfListener.waitForTransform("ardrone",leaderID,now,rospy.Duration(0.5))
            (trans,quat) = tfListener.lookupTransform("ardrone",leaderID,now)
            
            desOrientVec = rotateVec(np.array([1,0,0]),quat)
            angVelVec = np.cross([1,0,0],desOrientVec)
            angVelCmd = kw*angVelVec[2]
            
            trans = np.array(trans)
            trans[2] += 1 # Hover 1m above bot
            transDeriv = (trans-lastTrans)/(now-lastTime).to_sec()
            print "transDeriv: "+str(transDeriv)
            linVelCmd = kv*trans + kd*transDeriv
            
            lastTime = now
            lastTrans = trans
            
            if autonomy and not override:
                velCmd = Twist()
                (velCmd.linear.x,velCmd.linear.y,velCmd.linear.z) = linVelCmd
                velCmd.angular.z = angVelCmd
                velPub.publish(velCmd)
        except:
            pass
        finally:
            r.sleep()
    
    rospy.spin()
    velPub.publish(Twist())


def joyCallback(data):
    global override, autonomy
    
    if data.buttons[0]: # A - land
        landPub.publish(Empty())
    if data.buttons[3]: # Y - takeoff
        launchPub.publish(Empty())
    if data.buttons[1]: # B - reset
        resetPub.publish(Empty())
    
    autonomy = data.buttons[2] # X - autonomy
    
    override = data.buttons[4] or data.buttons[5] # LB/RB - override buttons
    if override: 
        joyVel = Twist()
        (joyVel.linear.x,joyVel.linear.y,joyVel.linear.z) = (deadband(data.axes[4]),deadband(data.axes[3]),deadband(data.axes[1]))
        joyVel.angular.z = deadband(data.axes[0])
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
