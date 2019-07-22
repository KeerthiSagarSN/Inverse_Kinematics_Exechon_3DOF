# -*- coding: utf-8 -*-
"""
Created on Fri Dec 07 16:07:58 2018

@author: keerthi
"""
from test_planner_lib import *


ax_frame = kinematics_plot()
ax = ax_frame.plot_frame()


#for i in xrange(100,101,1):
workspace_pts = np.load('workspace_'+str(1000)+str('_')+ str(1000)+str('_')+str(400)+str('.npz'))
wp = workspace_pts['wpoints']
#for j in xrange(0,np.size(wp)):
ax.scatter(wp[:,0],wp[:,1],wp[:,2],'b',s = 20)
    

    