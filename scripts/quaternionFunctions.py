#!/usr/bin/env python

import numpy as np

# quaternion formatted so q[0] is the scalar and
# q[1:3] is vector

# qDot = 1/2*B*[0,omega(1),omega(2),omega(3)]' => omega = 2*B^-1*qDot
# See equation 3.104 in Analytical Mechanics of Aerospace Systems by
# Hanspeter Schaub and John Junkins. If quaternion is orientation of frame
# B w.r.t N, omega is ang. vel of frame B w.r.t. N, i.e. N_w_B

def differentialMat(q):
    b0 = q[0]
    b1 = q[1]
    b2 = q[2]
    b3 = q[3]
    B = np.array([[b0, -b1, -b2, -b3],[b1, b0, -b3, b2],[b2, b3, b0, -b1],[b3, -b2, b1, b0]])
	
    return B


def quat2RotMat(q):
    [q,len] = fix(q);
    b0 = q[0]
    b1 = q[1]
    b2 = q[2]
    b3 = q[3]
    R = np.zeros((3,3))
    R[0,:] = [np.power(b0,2.0)+np.power(b1,2.0)-np.power(b2,2.0)-np.power(b3,2.0), 2*(b1*b2-b0*b3), 2*(b1*b3+b0*b2)]
    R[1,:] = [2*(b1*b2+b0*b3), np.power(b0,2.0)-np.power(b1,2.0)+np.power(b2,2.0)-np.power(b3,2.0), 2*(b2*b3-b0*b1)]
    R[2,:] = [2*(b1*b3-b0*b2), 2*(b2*b3+b0*b1), np.power(b0,2.0)-np.power(b1,2.0)-np.power(b2,2.0)+np.power(b3,2.0)]
	#R = [[np.power(b0,2.0)+np.power(b1,2.0)-np.power(b2,2.0)-np.power(b3,2.0), 2*(b1*b2-b0*b3), 2*(b1*b3+b0*b2)],[2*(b1*b2+b0*b3), np.power(b0,2.0)-np.power(b1,2.0)+np.power(b2,2.0)-np.power(b3,2.0), 2*(b2*b3-b0*b1)],[2*(b1*b3-b0*b2), 2*(b2*b3+b0*b1), np.power(b0,2.0)-np.power(b1,2.0)-np.power(b2,2.0)+np.power(b3,2.0)]]
	
    return R


def rotateVec(p,q):
    ptemp = np.zeros((4,1))
    ptemp[1:4] = p
    pp = multiply(multiply(q,ptemp),inverse(q))
    return pp[1:4]


def multiply(q1,q2):
    q3 = np.zeros((4,1))
    q3[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q3[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q3[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q3[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
	
    return q3


def conjugate(q):
    qp = np.zeros((4,1))
    qp[0] = q[0]
    qp[1:4] = -q[1:4]
    return qp


def unit(q):
    uq = q/np.linalg.norm(q)
    return uq


def inverse(q):
    #print conjugate(q)
    #print (np.power(np.linalg.norm(q),2.0))
    qinv = conjugate(q)/(np.power(np.linalg.norm(q),2.0))
    return qinv

