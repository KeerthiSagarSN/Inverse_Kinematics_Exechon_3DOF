# -*- coding: utf-8 -*-
"""
Created on Thu Nov 29 12:55:07 2018

@author: keerthi
"""
import numpy as np

a = np.matrix([[1,0,0,10],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

b = np.matrix([[5],[0],[0],[1]])

orig_2 = a*b

orig_1 = np.linalg.inv(a)*orig_2



