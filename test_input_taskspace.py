# -*- coding: utf-8 -*-
"""
Created on Fri Nov 09 03:18:53 2018

@author: keerthi
"""

import numpy as np
from IK_PKM import IK_PKM
import IK_PKM_params

from serialarm_IK import *



end_effector_pose = np.array([0.5,-0.5,0.7,2,2,2])


serial_arm_obj = serialarm_IK()
OS,EF_z_OS = serial_arm_obj.IK(end_effector_pose,0)

xd = OS[0,0]
yd = OS[1,0]
zd = OS[2,0]

'''
xd = end_effector_pose[0]
yd = end_effector_pose[1]
zd = end_effector_pose[2]
'''

#for i in xrange(0,3):
alpha = 0
beta =0
gamma = 0

r = euler_angles(alpha,beta,gamma,0)
p_centre = np.zeros(shape = [3,1])

p_centre[0,0] = xd
p_centre[1,0] = yd
p_centre[2,0] = zd

r4 = np.zeros(shape = [1,4])
r4[0,3] = 1



T = np.vstack((np.hstack((r,p_centre)),r4))
print('T isssssssssss')
print(T)
#T = np.matrix([[1, -1, 0, xd],[ -1, 1, 0, yd],[0, -1, 1, zd],[0, 0, 0, 0]])

ax_frame = kinematics_plot()
ax = ax_frame.plot_frame()

a = IK_PKM()
joints,ax = a.inverse_kinematics_pkm(ax,T,end_effector_pose,EF_z_OS,2) # T- Homogenoeous matrix, EF_pose - End effector pose

end_effector_pose = np.array([0.5,0.5,0.7,1,0,1])


serial_arm_obj = serialarm_IK()
OS,EF_z_OS = serial_arm_obj.IK(end_effector_pose,0)

xd = OS[0,0]
yd = OS[1,0]
zd = OS[2,0]

'''
xd = end_effector_pose[0]
yd = end_effector_pose[1]
zd = end_effector_pose[2]
'''

#for i in xrange(0,3):
alpha = 0
beta =0
gamma = 0

r = euler_angles(alpha,beta,gamma,0)
p_centre = np.zeros(shape = [3,1])

p_centre[0,0] = xd
p_centre[1,0] = yd
p_centre[2,0] = zd

r4 = np.zeros(shape = [1,4])
r4[0,3] = 1



T = np.vstack((np.hstack((r,p_centre)),r4))
print('T isssssssssss')
print(T)
#T = np.matrix([[1, -1, 0, xd],[ -1, 1, 0, yd],[0, -1, 1, zd],[0, 0, 0, 0]])

b = IK_PKM()
joints,ax = a.inverse_kinematics_pkm(ax,T,end_effector_pose,EF_z_OS,2)
