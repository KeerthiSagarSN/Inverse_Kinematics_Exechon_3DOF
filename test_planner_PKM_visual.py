# -*- coding: utf-8 -*-
"""
Created on Fri Nov 09 03:18:53 2018

@author: keerthi
"""

from test_planner_lib import *



ax_frame = kinematics_plot()
ax = ax_frame.plot_frame()


'''
base_1 = SaD_base()
ax = base_1.SaD_base_plot(ax)



from serialarm_IK import *



agent_1 = SaD_agent()
ax,Ox,Oy,Oz,T_saD_O,T_PKM_O = agent_1.SaD_agent_kinematics(ax,base_1,3,3,10,11)


end_effector_pose_input = np.array([0.1,1.25,2,0,0,0])

EF_1 = EF_plan()
ax,end_effector_pose_PKM = EF_1.plan_EF(ax,end_effector_pose_input,T_PKM_O)


serial_arm_obj = serialarm_IK()
OS,EF_z_OS = serial_arm_obj.IK(end_effector_pose_PKM)
'''

wp = np.matrix([[-1,-1,-1]])
T = np.matrix([[1.0, 0.0, 0.0, 0.0],[ 0.0, 1.0, 0.0, 0.0],[0.0, 0.0, 1.0, 0.0],[0.0, 0.0, 0, 1]])

stop_cnt = 0
#while stop_cnt != 1:
'''
for cnt3 in xrange(320,325,1):
    wp = np.matrix([[-1,-1,-1]])
    p_z = cnt3/1000.0
    for cnt1 in xrange(-450,450,1):
        p_x = cnt1/1000.0
    #print(p_x)
        for cnt2 in xrange(-450,450,1):
            p_y = cnt2/1000.0
        #print(p_y)
'''
p_x = 0.3
p_y = 0.3
p_z = 0.35
T[0,3] = p_x
T[1,3] = p_y
T[2,3] = p_z


'''
#for i in xrange(0,3):
alpha = 0
beta = 0
gamma = 0

r = zyz_euler(alpha,beta,gamma)
p_centre = np.zeros(shape = [3,1])

p_centre[0,0] = xd
p_centre[1,0] = yd
p_centre[2,0] = zd

r4 = np.zeros(shape = [1,4])
r4[0,3] = 1
'''



#print('T iss')
#print(T)


#T_PKM_O = np.matrix([[1, 0, 0, 0],[ 0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])

#end_effector_pose_PKM = end_effector_pose_input
#EF_z_OS = end_effector_pose_PKM[0:3]

obj_pkm = IK_PKM()
joints,e,ax = obj_pkm.inverse_kinematics_pkm(ax,T,1) # T- Homogenoeous matrix
#print(joints[0])
#print(joints[1])
#print(joints[2])
'''
if (joints[0] > 0.240 and joints[0] < 0.390 and  joints[1] > 0.240 and joints[1] < 0.390 and joints[2] > 0.240 and joints[2] < 0.390):
    # and acos(e[2]) < 1.04719 and acos(e[2]) > -1.04719
     #and asin(e[0]) < 0.52359 and asin(e[0]) > -0.52359):
    
    #ax.scatter(p_centr[0,0],p_centr[1,0],p_centr[2,0],'r',s = 20)
    #ax.pause(0.001)
    #print('OOOOOOOOOeeeeeeee')
    wp = np.vstack((wp,np.matrix([[p_x,p_y,p_z]])))
    #print(wp)
    #print(joints[0],joints[1],joints[2])
    stop_cnt = 1


del obj_pkm

'''
#    np.savez('workspace_'+str(cnt1)+str('_')+ str(cnt2)+str('_')+str(cnt3)+str('_')+str(40),wpoints = wp)


'''
T = np.vstack((np.hstack((r,p_centre)),r4))

#T = np.matrix([[1, -1, 0, xd],[ -1, 1, 0, yd],[0, -1, 1, zd],[0, 0, 0, 0]])

b = IK_PKM()
joints,ax = a.inverse_kinematics_pkm(ax,T,end_effector_pose_input,EF_z_OS,2)
'''